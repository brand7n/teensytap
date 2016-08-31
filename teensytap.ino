/*
 * This program attempts to initialize an SD card and analyze its structure.
 */
//#include <SPI.h>
//#include "SdFat.h"
#include "Arduino.h"
#include <Adafruit_NeoPixel.h>
//#include "tape.h"
void microTick();
void doSample();
void doWrite();

/*
 * SD chip select pin.  Common values are:
 *
 * Arduino Ethernet shield, pin 4.
 * SparkFun SD shield, pin 8.
 * Adafruit SD shields and modules, pin 10.
 * Default SD chip select is the SPI SS pin.
 */
//const uint8_t SD_CHIP_SELECT = 10;
/*
 * Set DISABLE_CHIP_SELECT to disable a second SPI device.
 * For example, with the Ethernet shield, set DISABLE_CHIP_SELECT
 * to 10 to disable the Ethernet controller.
 */
//const int8_t DISABLE_CHIP_SELECT = -1;
//SdFat sd;

// global for card size
//uint32_t cardSize;

// global for card erase size
//uint32_t eraseSize;
//------------------------------------------------------------------------------

#define HWSERIAL Serial1
#define READ_PIN 5 // writing from C128
#define WRITE_PIN 6// reading from C128
#define SENSE_PIN 2
#define MOTOR_PIN 22
#define TEST_PIN 23
#define DATA_BUF_SIZE 50000

//------------------------------------------------------------------------------
Adafruit_NeoPixel strip;
IntervalTimer poller;
uint8_t curRead = 0;
uint8_t lastRead = 0;
uint8_t ticks = 0;
int8_t data_buf[DATA_BUF_SIZE];
uint16_t data_write_idx = 0;
uint32_t wavCount = 0;
bool writeFlag = false;

void setup() {
  Serial.begin(9600);
  HWSERIAL.begin(9600);
  
  // Wait for USB Serial 
  //while (!Serial) {
    //SysCall::yield();
  //}

  // use uppercase in hex and use 0X base prefix
  // cout << uppercase << showbase << endl;

  strip = Adafruit_NeoPixel(1, 23, NEO_RGB + NEO_KHZ800);
  strip.begin();
  //strip.setPixelColor(0, strip.Color(0, 0, 0));
  strip.show();

  poller.begin(microTick, 10); // 10us

  pinMode(TEST_PIN, OUTPUT);
  pinMode(READ_PIN, INPUT);
  pinMode(SENSE_PIN, OUTPUT);
  pinMode(WRITE_PIN, OUTPUT);
  pinMode(MOTOR_PIN, INPUT);
  digitalWrite(SENSE_PIN, LOW);
  digitalWrite(WRITE_PIN, LOW);
}
//------------------------------------------------------------------------------
void loop() {
  // strip.setPixelColor(0, strip.Color(255, 0, 0));
  // strip.show();
  // Read any existing Serial data.
  do {
    delay(10);
  } while (Serial.available() && Serial.read() >= 0);

  // F stores strings in flash to save RAM
  Serial.println("type any character to start");
  while (!Serial.available()) {
  //   SysCall::yield();
  }
  HWSERIAL.print("\x1B[j");
  HWSERIAL.print("\x1B[0;0H");
  HWSERIAL.print("HELL WORLD");
  strip.setPixelColor(0, strip.Color(0, 255, 255));
  strip.show();


  
  HWSERIAL.print("\x1B[0;0H");
  HWSERIAL.println("END");

  // for (int i=0; i<data_write_idx; i++) {
  //   if (data_buf[i] > 20 || data_buf[i] < -20) {
  //     cout << i << " " << (int32_t)data_buf[i] << endl;
  //   }
  // }
  // cout << data_write_idx << endl;
  // cout << wavCount << endl;
  // int test = digitalRead(MOTOR_PIN);
  // if (test == HIGH) {
  //   cout << "MOTOR ON" << endl;
  // }

  data_write_idx = 0;
  wavCount = 0;
  writeFlag = !writeFlag;
  // if (writeFlag) {
  //   cout << "entering write mode" << endl;
  // }
}

void microTick(void) {
  if (!writeFlag) {
    doSample();
  } else {
    doWrite();
  }
}

void doSample(void) {
  curRead = digitalRead(READ_PIN);
  digitalWrite(TEST_PIN, curRead);
  if (curRead == lastRead) { // no change in signal
    if (ticks < 0x7F) { // 0xFFFF is the maximum
      ticks++;
    }
    return;
  }
  int8_t val = 0;
  if (lastRead == HIGH) {
    val = ticks;
  } else { 
    val = -ticks;
  }
  if (data_write_idx < DATA_BUF_SIZE && wavCount > 40000) {
    data_buf[data_write_idx++] = val;
  }
  wavCount++;
  ticks = 0;
  lastRead = curRead;
}

int8_t curVal = 0;
enum {NEWVAL = 0, DELAY, DONE} state;

void doWrite(void) 
{
  if (digitalRead(MOTOR_PIN) != HIGH) {
    return;
  }
  switch(state) {
    case NEWVAL:
    if (data_write_idx >= DATA_BUF_SIZE) {
      state = DONE;
      return;
    }
    curVal = data_buf[data_write_idx++];
    if (curVal > 0) {
      digitalWrite(WRITE_PIN, LOW);
    } else if (curVal < 0) {
      digitalWrite(WRITE_PIN, HIGH);
    } else { // zero
      curVal = 1; // shouldn't happen
    }
    curVal = abs(curVal);
    curVal--;
    if (curVal != 0) {
      state = DELAY;
    }
    break;

    case DELAY:
    curVal--;
    if (curVal != 0) {
      state = DELAY;
    } else {
      state = NEWVAL;
    }
    break;

    default:
    break;
  }
}
