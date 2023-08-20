/*
 * Raspberry Pi Pico interfacing to 4 rotary encoders and switches and touchscreen.
 *
 * Communicate externally over i2c as a slave
 *
 * Thread 0 initializes the hardware and then handles interrupts from the encoders, switches and touchscreen.

 * Thread 1 handles the i2c interface to the Teensy 4.1.
 *
 * I2C_SLAVE_INT_PIN is set Active when there is an encoder change, button pressed/released or a touch.
 *
*/

#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/multicore.h"
#include "i2c_fifo.h"
#include "i2c_slave.h"


//#define DEBUG_MESSAGES

#ifdef DEBUG_MESSAGES
#define Debug(x) Serial.println(x)
#else
#define Debug(x)
#endif

#include <Wire.h>
#include <GSL1680.h>

#define TOUCH_SCL 27
#define TOUCH_SDA 26
#define TOUCH_WAKE 19
#define TOUCH_INTRPT 18
GSL1680 TS = GSL1680(false, false);  // disable error and info messages
static bool touched;
static int X;
static int Y;
static bool released = false;
static int lastX;
static int lastY;
static uint8_t touch_status;
static uint8_t last_status;

#define CONFIG_REG 0x00
#define RESET_REG 0x01
#define READ_INTERRUPT_MASK_REG 0x02
#define READ_ENCODER_REG 0x03
#define READ_SWITCH_REG 0x04
#define READ_TOUCH_REG 0x05
#define LED_REG 0x06

#define ENC0_A 3
#define ENC0_B 4
#define ENC0_SW 5

#define ENC1_A 6
#define ENC1_B 7
#define ENC1_SW 8

#define ENC2_A 9
#define ENC2_B 10
#define ENC2_SW 11

#define ENC3_A 12
#define ENC3_B 13
#define ENC3_SW 14

#define LED1 16
#define LED2 17

#define MIC_DOWN 20
#define MIC_UP 21

static bool ccw_fall[4] = { 0, 0, 0, 0 };
static bool cw_fall[4] = { 0, 0, 0, 0 };

static int led1 = 0;
static int led2 = 0;

static unsigned char previous_state[4] = { 15, 15, 15, 15 };
static int16_t enc_val[4] = { 0, 0, 0, 0 };
static int16_t enc_int_val[4] = { 0, 0, 0, 0 };
static uint8_t enc_sw[4] = { 1, 1, 1, 1 };
static int encoder;

// I2C0 -- SLAVE connected to Teensy4.1
static const uint I2C_SLAVE_SDA_PIN = 0;
static const uint I2C_SLAVE_SCL_PIN = 1;
static const uint I2C_SLAVE_INT_PIN = 15;

#define I2C_SLAVE_ADDRESS 0x20
#define I2C_BAUDRATE 100000

enum _slave_state_t {
  IDLE,
  WAITING_FOR_CONFIG_HIGH,
  WAITING_FOR_CONFIG_LOW,
  WAITING_FOR_ENCODER,
  READING_INTERRUPT_MASK_LOW,
  READING_INTERRUPT_MASK_HIGH,
  READING_COUNTER_LOW,
  READING_COUNTER_HIGH,
  WAITING_FOR_SWITCH,
  READING_SWITCH,
  READING_TOUCH_STATE,
  READING_TOUCH_X_LOW,
  READING_TOUCH_X_HIGH,
  READING_TOUCH_Y_LOW,
  READING_TOUCH_Y_HIGH,
  WAITING_FOR_LED,
  WAITING_FOR_LED_STATE
};

static _slave_state_t slave_state = IDLE;
static int byte_count = 0;

// the following can be configured from the i2c interface using reg addr 0x00
static bool fwd[4] = { false, false, false, false };     // forward or reverse: encoder config bit 0,1,2,3
static bool invert[4] = { true, true, true, true };      // invert switch: config bits 4,5,6,7
static bool int_active_high = false;                     // interrupt active high/low config bit 8

static uint16_t int_mask = 0;  // 0..3 encoder interrupts, 4..7 switch interrupts, 8 touchscreen, 15 ready
static bool int_triggered=false;

static void clear_interrupt();
static void trigger_enc_interrupt(int enc);
static void trigger_ready_interrupt();

static uint8_t config;
static uint8_t reg;
static uint8_t enc;
static uint8_t sw;
static uint8_t led;
static uint8_t led_state;

static unsigned long debounce_delay = 100L;
static unsigned long debounce_millis;
static struct repeating_timer timer;
static bool timer_active=false;

//
// ---- Core 1 code ---- I2C interface
//
void set_led(int led,int state) {
  switch(led) {
    case 0:
      gpio_put(LED1,state);
      break;
    case 1:
      gpio_put(LED2,state);
      break;
  }
}

static void i2c_slave_handler(i2c_inst *i2c, i2c_slave_event_t event) {
  Debug(String(__FUNCTION__)+": Event="+String(event));
  switch (event) {
    case I2C_SLAVE_RECEIVE:  // master has written some data
      Debug(String(__FUNCTION__)+": I2C_SLAVE_RECEIVE");
      switch (slave_state) {
        case IDLE:  // read the register that is to be read or written to
          reg = i2c_read_byte(i2c);
          Debug(String(__FUNCTION__)+": IDLE: reg=" + String(reg));
          switch (reg) {
            case CONFIG_REG:  // config
              slave_state = WAITING_FOR_CONFIG_LOW;
              break;
            case RESET_REG:  // reset encoder count
              slave_state = WAITING_FOR_ENCODER;
              break;
            case READ_INTERRUPT_MASK_REG:  // read interrupt mask
              slave_state = READING_INTERRUPT_MASK_LOW;
              break;
            case  READ_ENCODER_REG:  // read encoder value
              slave_state = WAITING_FOR_ENCODER;
              break;
            case READ_SWITCH_REG:  // read switch state
              slave_state = WAITING_FOR_SWITCH;
              break;
            case READ_TOUCH_REG:  // read touch
              slave_state= READING_TOUCH_STATE;
              break;
            case LED_REG:
              slave_state= WAITING_FOR_LED;
              break;
            default:
              slave_state = IDLE; 
              break;
          }
          break;
        case WAITING_FOR_CONFIG_LOW:
          config = i2c_read_byte(i2c);
          slave_state = WAITING_FOR_CONFIG_HIGH;
          break;
        case WAITING_FOR_CONFIG_HIGH:
          config |= i2c_read_byte(i2c) << 8;
          Debug(String(__FUNCTION__)+": config=0x" + String(config, HEX));
          for (int i = 0; i < 4; i++) {
            fwd[i] = (config >> i) & 0x01;
          }
          for (int i = 0; i < 4; i++) {
            invert[i] = (config >> (i + 4)) & 0x01;
            enc_sw[i]=(uint8_t)invert[i]; // set inital value
          }
          int_active_high = (config >> 8) & 0x01;
          clear_interrupt();
          slave_state = IDLE;
          break;
        case WAITING_FOR_ENCODER:
          enc = i2c_read_byte(i2c);
          Debug(String(__FUNCTION__)+": WAITING_FOR_ENCODER: " + String(enc));
          switch (reg) {
            case RESET_REG:
              enc_val[enc] = 0;
              slave_state = IDLE;
              break;
            case READ_ENCODER_REG:
              slave_state = READING_COUNTER_LOW;
              byte_count = 0;
              break;
          }
          break;
        case WAITING_FOR_SWITCH:
          sw = i2c_read_byte(i2c);
          Debug(String(__FUNCTION__)+": WAITING_FOR_SWITCH: " + String(sw));
          slave_state = READING_SWITCH;
          break;
        case WAITING_FOR_LED:
          led = i2c_read_byte(i2c);
          Debug(String(__FUNCTION__)+": WAITING_FOR_LED: " + String(led));
          slave_state = WAITING_FOR_LED_STATE;
          break;
        case WAITING_FOR_LED_STATE:
          led_state = i2c_read_byte(i2c);
          Debug(String(__FUNCTION__)+": WAITING_FOR_LED_State: " + String(led_state));
          set_led(led,led_state);
          slave_state = IDLE;
          break;
      }
      break;
    case I2C_SLAVE_REQUEST:
      Debug(String(__FUNCTION__)+": I2C_SLAVE_REQUEST");
      switch (slave_state) {

        case READING_INTERRUPT_MASK_LOW:
          Debug(String(__FUNCTION__)+": READING_INTERRUPT_MASK_LOW");
          i2c_write_byte(i2c, int_mask & 0xFF);
          slave_state = READING_INTERRUPT_MASK_HIGH;
          break;
        case READING_INTERRUPT_MASK_HIGH:
          Debug(String(__FUNCTION__)+": READING_INTERRUPT_MASK_HIGH");
          i2c_write_byte(i2c, (int_mask >> 8) & 0xFF);
          slave_state = IDLE;
          if(int_mask==0x0000 || int_mask==0x8000) {
            clear_interrupt();
          }
          break;
        case READING_COUNTER_LOW:
          Debug(String(__FUNCTION__)+": READING_COUNTE_LOW");
          i2c_write_byte(i2c, (enc_int_val[enc] & 0xFF));
          slave_state = READING_COUNTER_HIGH;
          break;
        case READING_COUNTER_HIGH:
          Debug(String(__FUNCTION__)+": READING_COUNTER_HIGH");
          i2c_write_byte(i2c, ((enc_int_val[enc]>>8) & 0xFF));
          int_mask=0;
          slave_state = IDLE;
          clear_interrupt();
          if(enc_val[enc]!=0) {
            trigger_enc_interrupt(enc);
          }
          break;
        case READING_SWITCH:
          Debug(String(__FUNCTION__)+": READING_SWITCH");
          if (invert[sw]) {
            i2c_write_byte(i2c, enc_sw[sw] == 0 ? 1 : 0);
          } else {
            i2c_write_byte(i2c, enc_sw[sw]);
          }
          slave_state = IDLE;
          clear_interrupt();
          break;
        case READING_TOUCH_STATE:
          i2c_write_byte(i2c, touch_status);
          slave_state = READING_TOUCH_X_LOW;
          break;
        case READING_TOUCH_X_LOW:
          i2c_write_byte(i2c, X&0xFF);
          slave_state = READING_TOUCH_X_HIGH;
          break;
        case READING_TOUCH_X_HIGH:
          i2c_write_byte(i2c, (X>>8)&0xFF);
          slave_state = READING_TOUCH_Y_LOW;
          break;
        case READING_TOUCH_Y_LOW:
          i2c_write_byte(i2c, Y&0xFF);
          slave_state = READING_TOUCH_Y_HIGH;
          break;
        case READING_TOUCH_Y_HIGH:
          i2c_write_byte(i2c, (Y>>8)&0xFF);
          slave_state = IDLE;
          clear_interrupt();
          break;
        default:
          Debug(String(__FUNCTION__)+": UNKNOWN!");
          break;
      }
      break;
    case I2C_SLAVE_FINISH:  // master has signalled Stop / Restart
      Debug(String(__FUNCTION__)+": I2C_SLAVE_FINISH");
      break;
    default:
      Debug(String(__FUNCTION__)+": Unknown event: " + String(event));
      break;
  }
}

// initialize the slave thread on core 1
void i2c_slave_thread() {

  Debug(String(__FUNCTION__)+": Address=0x"+String(I2C_SLAVE_ADDRESS, HEX));

  // init the i2c interface
  i2c_init(i2c0, I2C_BAUDRATE);
  i2c_set_slave_mode(i2c0, true, I2C_SLAVE_ADDRESS);

  // setup pin for slave SDA
  gpio_set_function(I2C_SLAVE_SDA_PIN, GPIO_FUNC_I2C);

  // setup pin for slave SCL
  gpio_set_function(I2C_SLAVE_SCL_PIN, GPIO_FUNC_I2C);

  i2c0->hw->intr_mask = (I2C_IC_INTR_MASK_M_RD_REQ_BITS | I2C_IC_INTR_MASK_M_RX_FULL_BITS);

  // start with slave state in IDLE
  slave_state = IDLE;

  // enable IRQ
  irq_set_enabled(I2C0_IRQ, true);

  // configure I2C0 for slave mode
  i2c_slave_init(i2c0, I2C_SLAVE_ADDRESS, &i2c_slave_handler);

  Debug(String(__FUNCTION__)+": slave initialized");

  // just sleep waiting for an interrupt
  while (1) {
    sleep_ms(1000);
  }
}

//
// ---- Core 0 code ---- interrupt handler
//

// clear the interrupt pin
static void clear_interrupt() {
  Debug(String(__FUNCTION__)+": "+String(int_active_high?0:1));
  if (int_active_high) {
    gpio_put(I2C_SLAVE_INT_PIN, 0);
  } else {
    gpio_put(I2C_SLAVE_INT_PIN, 1);
  }
  int_triggered=false;
}

// trigger the interrupt pin
static void trigger_interrupt() {
  Debug(String(__FUNCTION__)+": "+String(int_active_high?1:0));
  if(int_triggered) {
    if (int_active_high) {
      gpio_put(I2C_SLAVE_INT_PIN, 0);
    } else {
      gpio_put(I2C_SLAVE_INT_PIN, 1);
    }
    delay(100);
  }
  if (int_active_high) {
    gpio_put(I2C_SLAVE_INT_PIN, 1);
  } else {
    gpio_put(I2C_SLAVE_INT_PIN, 0);
  }
  int_triggered=true;
}

// trigger an interrupt ot the Master for an encoder
static void trigger_enc_interrupt(int enc) {
  Debug(String(__FUNCTION__)+": enc="+String(enc));
  int_mask = 0x01 << enc;
  enc_int_val[enc]=enc_val[enc];
  enc_val[enc]=0;
  trigger_interrupt();
}

// trigger an interrupt ot the Master for a switch
static void trigger_sw_interrupt(int sw) {
  Debug(String(__FUNCTION__)+": sw="+String(sw));
  int_mask = 0x10 << sw;
  trigger_interrupt();
}

// trigger an interrupt ot the Master to for a touch
static void trigger_touch_interrupt() {
  Debug(String(__FUNCTION__));
  int_mask = 0x100;
  trigger_interrupt();
}

// trigger an interrupt ot the Master to say we are ready
static void trigger_ready_interrupt() {
  Debug(String(__FUNCTION__));
  int_mask = 0x8000;
  trigger_interrupt();
}

// timer callback for when microphone up/down are held down
bool repeating_timer_callback(struct repeating_timer *t) {
  int v=(int)t->user_data;
  enc_val[2]=enc_val[2]+v;
  trigger_enc_interrupt(2);
  return true;
}

// process interrupt from encoders, encoder switches, microphone up/down and touch screen
void irq_callback(uint gpio, uint32_t events) {
  uint32_t gpio_state = 0;
  int NBFinger;

  // get the gpio state for all pins
  gpio_state = gpio_get_all();

  // extract out the GPIO states for the encoder and switches
  switch (gpio) {
    case ENC0_A:
    case ENC0_B:
      encoder = 0;
      gpio_state = (gpio_state >> 3) & 0x03;
      break;
    case ENC1_A:
    case ENC1_B:
      encoder = 1;
      gpio_state = (gpio_state >> 6) & 0x03;
      break;
    case ENC2_A:
    case ENC2_B:
      encoder = 2;
      gpio_state = (gpio_state >> 9) & 0x03;
      break;
    case ENC3_A:
    case ENC3_B:
      encoder = 3;
      gpio_state = (gpio_state >> 12) & 0x03;
      break;

    case ENC0_SW:
      encoder = 0;
      gpio_state = (gpio_state >> 5) & 0x01;
      break;
    case ENC1_SW:
      encoder = 1;
      gpio_state = (gpio_state >> 8) & 0x01;
      break;
    case ENC2_SW:
      encoder = 2;
      gpio_state = (gpio_state >> 11) & 0x01;
      break;
    case ENC3_SW:
      encoder = 3;
      gpio_state = (gpio_state >> 14) & 0x01;
      break;
    case MIC_DOWN:
      gpio_state = (gpio_state >> 20) & 0x01;
      break;
    case MIC_UP:
      gpio_state = (gpio_state >> 21) & 0x01;
      break;
    // handle touch interrutps
    case TOUCH_INTRPT:
      touched = false;
      released = false;
      NBFinger = TS.dataread();
      if (NBFinger > 0) {
        X = TS.readFingerX(0);
        Y = TS.readFingerY(0);
        touched = true;
        Debug("Touched: "+String(X)+":"+String(Y));
      } else {
        released = true;
      }
      touch_status = (touched<<1)|released;
      if(touch_status!=last_status || X!=lastX || Y!=lastY) {
        last_status=touch_status;
        lastX=X;
        lastY=Y;
        trigger_touch_interrupt();
      }
      break;
  }

  // process the interrupt state for the gpio
  switch (gpio) {
    case ENC0_A:
    case ENC1_A:
    case ENC2_A:
    case ENC3_A:
      if ((!cw_fall[encoder]) && (gpio_state == 0b10))  // cw_fall is set to TRUE when phase A interrupt is triggered
        cw_fall[encoder] = true;

      if ((ccw_fall[encoder]) && (gpio_state == 0b00))  // if ccw_fall is already set to true from a previous B phase trigger, the ccw event will be triggered
      {
        cw_fall[encoder] = false;
        ccw_fall[encoder] = false;
        if (fwd[encoder]) {
          enc_val[encoder]++;
        } else {
          enc_val[encoder]--;

        }
        trigger_enc_interrupt(encoder);
      }
      break;
    case ENC0_B:
    case ENC1_B:
    case ENC2_B:
    case ENC3_B:
      if ((!ccw_fall[encoder]) && (gpio_state == 0b01))  //ccw leading edge is true
        ccw_fall[encoder] = true;

      if ((cw_fall[encoder]) && (gpio_state == 0b00))  //cw trigger
      {
        cw_fall[encoder] = false;
        ccw_fall[encoder] = false;
        if (fwd[encoder]) {
          enc_val[encoder]--;
        } else {
          enc_val[encoder]++;
        }
        trigger_enc_interrupt(encoder);
      }
      break;

    // process encoder switch interrupts
    case ENC0_SW:
    case ENC1_SW:
    case ENC2_SW:
    case ENC3_SW:
      if(millis()>=debounce_millis) {
        enc_sw[encoder] = gpio_state;
        trigger_sw_interrupt(encoder);
        debounce_millis = millis()+debounce_delay;
      }
      break;

    // process MIC Down
    case MIC_DOWN:
      // simulate Center Tune encoder
      if(millis()>=debounce_millis) {
        if(gpio_state==0) {
          enc_val[2]--;
          trigger_enc_interrupt(2);
          if(timer_active) {
            cancel_repeating_timer(&timer);
            timer_active=false;
          }
          add_repeating_timer_ms(250, repeating_timer_callback, (void *)-1, &timer);
          timer_active=true;
        } else if(gpio_state==1) {
          cancel_repeating_timer(&timer);
          timer_active=false;
        }
        debounce_millis = millis()+debounce_delay;
      }
      break;
    // process Mic Up
    case MIC_UP:
      // simulate Center Tune encoder
      if(millis()>=debounce_millis) {
        if(gpio_state==0) {
          enc_val[2]++;
          trigger_enc_interrupt(2);
          if(timer_active) {
            cancel_repeating_timer(&timer);
            timer_active=false;
          }
          add_repeating_timer_ms(250, repeating_timer_callback, (void *)1, &timer);
          timer_active=true;
        } else if(gpio_state==1) {
          cancel_repeating_timer(&timer);
          timer_active=false;
        }
        debounce_millis = millis()+debounce_delay;
      }
      break;
    // nothing more to do for touch interrupts
    case TOUCH_INTRPT:
      break;
  }
}

void setup() {

  int tries;
  bool foundTS;
  uint8_t res;

#ifdef DEBUG_MESSAGES
  Serial.begin(115200);
  while (!Serial)
    ;
#endif

  Debug(String(__FUNCTION__));

  // setup LEDs
  gpio_init(LED1);
  gpio_set_dir(LED1, GPIO_OUT);
  set_led(LED1, LOW);

  gpio_init(LED2);
  gpio_set_dir(LED2, GPIO_OUT);
  set_led(LED2, LOW);

  // setup encoder pins
  gpio_init(ENC0_A);
  gpio_set_dir(ENC0_A, GPIO_IN);
  gpio_pull_up(ENC0_A);

  gpio_init(ENC0_B);
  gpio_set_dir(ENC0_B, GPIO_IN);
  gpio_pull_up(ENC0_B);

  gpio_init(ENC0_SW);
  gpio_set_dir(ENC0_SW, GPIO_IN);
  gpio_pull_up(ENC0_SW);

  gpio_init(ENC1_A);
  gpio_set_dir(ENC1_A, GPIO_IN);
  gpio_pull_up(ENC1_A);

  gpio_init(ENC1_B);
  gpio_set_dir(ENC1_B, GPIO_IN);
  gpio_pull_up(ENC1_B);

  gpio_init(ENC1_SW);
  gpio_set_dir(ENC1_SW, GPIO_IN);
  gpio_pull_up(ENC1_SW);

  gpio_init(ENC2_A);
  gpio_set_dir(ENC2_A, GPIO_IN);
  gpio_pull_up(ENC2_A);

  gpio_init(ENC2_B);
  gpio_set_dir(ENC2_B, GPIO_IN);
  gpio_pull_up(ENC2_B);

  gpio_init(ENC2_SW);
  gpio_set_dir(ENC2_SW, GPIO_IN);
  gpio_pull_up(ENC2_SW);

  gpio_init(ENC3_A);
  gpio_set_dir(ENC3_A, GPIO_IN);
  gpio_pull_up(ENC3_A);

  gpio_init(ENC3_B);
  gpio_set_dir(ENC3_B, GPIO_IN);
  gpio_pull_up(ENC3_B);

  gpio_init(ENC3_SW);
  gpio_set_dir(ENC3_SW, GPIO_IN);
  gpio_pull_up(ENC3_SW);


  // setup encoder interrupts
  gpio_set_irq_enabled_with_callback(ENC0_A, GPIO_IRQ_EDGE_FALL, true, &irq_callback);
  gpio_set_irq_enabled(ENC0_B, GPIO_IRQ_EDGE_FALL, true);
  gpio_set_irq_enabled(ENC0_SW, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

  gpio_set_irq_enabled(ENC1_A, GPIO_IRQ_EDGE_FALL, true);
  gpio_set_irq_enabled(ENC1_B, GPIO_IRQ_EDGE_FALL, true);
  gpio_set_irq_enabled(ENC1_SW, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

  gpio_set_irq_enabled(ENC2_A, GPIO_IRQ_EDGE_FALL, true);
  gpio_set_irq_enabled(ENC2_B, GPIO_IRQ_EDGE_FALL, true);
  gpio_set_irq_enabled(ENC2_SW, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

  gpio_set_irq_enabled(ENC3_A, GPIO_IRQ_EDGE_FALL, true);
  gpio_set_irq_enabled(ENC3_B, GPIO_IRQ_EDGE_FALL, true);
  gpio_set_irq_enabled(ENC3_SW, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

  // setup the Microphone Up/Down buttons on the Kenwood MC-44
  gpio_init(MIC_DOWN);
  gpio_set_dir(MIC_DOWN, GPIO_IN);
  gpio_pull_up(MIC_DOWN);
  gpio_init(MIC_UP);
  gpio_set_dir(MIC_UP, GPIO_IN);
  gpio_pull_up(MIC_UP);
  gpio_set_irq_enabled(MIC_DOWN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
  gpio_set_irq_enabled(MIC_UP, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

  // setup the slave interrupt pin to the Master device (Teensy 4.1)
  gpio_init(I2C_SLAVE_INT_PIN);
  gpio_set_dir(I2C_SLAVE_INT_PIN, GPIO_OUT);

  clear_interrupt();

  // Initialize the touch screen connected to Wire1
  Wire1.setSCL(TOUCH_SCL);
  Wire1.setSDA(TOUCH_SDA);
  Wire1.begin();

  res=-1;
  foundTS=false;
  tries=0;
  while(!foundTS && tries<5) {
    Wire1.beginTransmission(0x40);
    res=Wire1.endTransmission();
    if(res!=0) {
      Debug("Did not find TS on Wire1@0x40: result="+String(res,HEX));
      delay(250);
    } else {
      foundTS=true;
      Debug("Found TS Wire1 @0x40");
    }
  }

  if(foundTS) {
    lastX = 0;
    lastY = 0;
    last_status=0;
    TS.begin(TOUCH_WAKE, TOUCH_INTRPT, &Wire1);
    gpio_init(TOUCH_INTRPT);
    gpio_set_dir(TOUCH_INTRPT, GPIO_IN);
    gpio_pull_up(TOUCH_INTRPT);
    gpio_set_irq_enabled(TOUCH_INTRPT, GPIO_IRQ_EDGE_FALL, true);
  }

  // initialize the switch debounce
  debounce_millis = millis()+debounce_delay;

  // start the thread to manage the i2c interface as a peripheral (slave)
  multicore_launch_core1(i2c_slave_thread);
}

void loop() {
  // main loop runs off interrupts for encoders, encoder switches, touch screen, Mic Up/Down
  sleep_ms(1000);
}
