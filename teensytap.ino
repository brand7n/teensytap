/*
 * This program attempts to initialize an SD card and analyze its structure.
 */
#include <SPI.h>
#include "SdFat.h"
#include <Adafruit_NeoPixel.h>
#include "tape.h"

/*
 * SD chip select pin.  Common values are:
 *
 * Arduino Ethernet shield, pin 4.
 * SparkFun SD shield, pin 8.
 * Adafruit SD shields and modules, pin 10.
 * Default SD chip select is the SPI SS pin.
 */
const uint8_t SD_CHIP_SELECT = 10;
/*
 * Set DISABLE_CHIP_SELECT to disable a second SPI device.
 * For example, with the Ethernet shield, set DISABLE_CHIP_SELECT
 * to 10 to disable the Ethernet controller.
 */
const int8_t DISABLE_CHIP_SELECT = -1;
SdFat sd;

// serial output steam
ArduinoOutStream cout(Serial);

// global for card size
uint32_t cardSize;

// global for card erase size
uint32_t eraseSize;
//------------------------------------------------------------------------------
// store error strings in flash
#define sdErrorMsg(msg) sdErrorMsg_F(F(msg));
void sdErrorMsg_F(const __FlashStringHelper* str) {
  cout << str << endl;
  if (sd.card()->errorCode()) {
    cout << F("SD errorCode: ");
    cout << hex << int(sd.card()->errorCode()) << endl;
    cout << F("SD errorData: ");
    cout << int(sd.card()->errorData()) << dec << endl;
  }
}
//------------------------------------------------------------------------------
uint8_t cidDmp() {
  cid_t cid;
  if (!sd.card()->readCID(&cid)) {
    sdErrorMsg("readCID failed");
    return false;
  }
  cout << F("\nManufacturer ID: ");
  cout << hex << int(cid.mid) << dec << endl;
  cout << F("OEM ID: ") << cid.oid[0] << cid.oid[1] << endl;
  cout << F("Product: ");
  for (uint8_t i = 0; i < 5; i++) {
    cout << cid.pnm[i];
  }
  cout << F("\nVersion: ");
  cout << int(cid.prv_n) << '.' << int(cid.prv_m) << endl;
  cout << F("Serial number: ") << hex << cid.psn << dec << endl;
  cout << F("Manufacturing date: ");
  cout << int(cid.mdt_month) << '/';
  cout << (2000 + cid.mdt_year_low + 10 * cid.mdt_year_high) << endl;
  cout << endl;
  return true;
}
//------------------------------------------------------------------------------
uint8_t csdDmp() {
  csd_t csd;
  uint8_t eraseSingleBlock;
  if (!sd.card()->readCSD(&csd)) {
    sdErrorMsg("readCSD failed");
    return false;
  }
  if (csd.v1.csd_ver == 0) {
    eraseSingleBlock = csd.v1.erase_blk_en;
    eraseSize = (csd.v1.sector_size_high << 1) | csd.v1.sector_size_low;
  } else if (csd.v2.csd_ver == 1) {
    eraseSingleBlock = csd.v2.erase_blk_en;
    eraseSize = (csd.v2.sector_size_high << 1) | csd.v2.sector_size_low;
  } else {
    cout << F("csd version error\n");
    return false;
  }
  eraseSize++;
  cout << F("cardSize: ") << 0.000512*cardSize;
  cout << F(" MB (MB = 1,000,000 bytes)\n");

  cout << F("flashEraseSize: ") << int(eraseSize) << F(" blocks\n");
  cout << F("eraseSingleBlock: ");
  if (eraseSingleBlock) {
    cout << F("true\n");
  } else {
    cout << F("false\n");
  }
  return true;
}
//------------------------------------------------------------------------------
// print partition table
uint8_t partDmp() {
  cache_t *p = sd.vol()->cacheClear();
  if (!p) {
    sdErrorMsg("cacheClear failed");
    return false;
  }
  if (!sd.card()->readBlock(0, p->data)) {
    sdErrorMsg("read MBR failed");
    return false;
  }
  for (uint8_t ip = 1; ip < 5; ip++) {
    part_t *pt = &p->mbr.part[ip - 1];
    if ((pt->boot & 0X7F) != 0 || pt->firstSector > cardSize) {
      cout << F("\nNo MBR. Assuming Super Floppy format.\n");
      return true;
    }
  }
  cout << F("\nSD Partition Table\n");
  cout << F("part,boot,type,start,length\n");
  for (uint8_t ip = 1; ip < 5; ip++) {
    part_t *pt = &p->mbr.part[ip - 1];
    cout << int(ip) << ',' << hex << int(pt->boot) << ',' << int(pt->type);
    cout << dec << ',' << pt->firstSector <<',' << pt->totalSectors << endl;
  }
  return true;
}
//------------------------------------------------------------------------------
void volDmp() {
  cout << F("\nVolume is FAT") << int(sd.vol()->fatType()) << endl;
  cout << F("blocksPerCluster: ") << int(sd.vol()->blocksPerCluster()) << endl;
  cout << F("clusterCount: ") << sd.vol()->clusterCount() << endl;
  // cout << F("freeClusters: ");
  // uint32_t volFree = sd.vol()->freeClusterCount();
  // cout <<  volFree << endl;
  // float fs = 0.000512*volFree*sd.vol()->blocksPerCluster();
  //cout << F("freeSpace: ") << fs << F(" MB (MB = 1,000,000 bytes)\n");
  cout << F("fatStartBlock: ") << sd.vol()->fatStartBlock() << endl;
  cout << F("fatCount: ") << int(sd.vol()->fatCount()) << endl;
  cout << F("blocksPerFat: ") << sd.vol()->blocksPerFat() << endl;
  cout << F("rootDirStart: ") << sd.vol()->rootDirStart() << endl;
  cout << F("dataStartBlock: ") << sd.vol()->dataStartBlock() << endl;
  if (sd.vol()->dataStartBlock() % eraseSize) {
    cout << F("Data area is not aligned on flash erase boundaries!\n");
    cout << F("Download and use formatter from www.sdcard.org!\n");
  }
}
#define HWSERIAL Serial1
#define READ_PIN 5 // writing from C128
#define WRITE_PIN 6// reading from C128
#define SENSE_PIN 2
#define MOTOR_PIN 22
#define TEST_PIN 3
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
  while (!Serial) {
    SysCall::yield();
  }

  // use uppercase in hex and use 0X base prefix
  cout << uppercase << showbase << endl;

  // F stores strings in flash to save RAM
  cout << F("SdFat version: ") << SD_FAT_VERSION << endl;
  if (DISABLE_CHIP_SELECT < 0) {
    cout << F(
           "\nAssuming the SD is the only SPI device.\n"
           "Edit DISABLE_CHIP_SELECT to disable another device.\n");
  } else {
    cout << F("\nDisabling SPI device on pin ");
    cout << int(DISABLE_CHIP_SELECT) << endl;
    pinMode(DISABLE_CHIP_SELECT, OUTPUT);
    digitalWrite(DISABLE_CHIP_SELECT, HIGH);
  }
  cout << F("\nAssuming the SD chip select pin is: ") <<int(SD_CHIP_SELECT);
  cout << F("\nEdit SD_CHIP_SELECT to change the SD chip select pin.\n");
  strip = Adafruit_NeoPixel(1, 23, NEO_RGB + NEO_KHZ400);
  strip.begin();
  //strip.setPixelColor(0, strip.Color(0, 255, 0));
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
  strip.setPixelColor(0, strip.Color(255, 0, 0));
  strip.show();
  // Read any existing Serial data.
  do {
    delay(10);
  } while (Serial.available() && Serial.read() >= 0);

  // F stores strings in flash to save RAM
  cout << F("\ntype any character to start\n");
  while (!Serial.available()) {
    SysCall::yield();
  }
  HWSERIAL.print("\x1B[j");
  HWSERIAL.print("\x1B[0;0H");
  HWSERIAL.print("HELL WORLD");
  strip.setPixelColor(0, strip.Color(0, 0, 255));
  strip.show();


  uint32_t t = millis();
  // initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
  // breadboards.  use SPI_FULL_SPEED for better performance.
  if (!sd.cardBegin(SD_CHIP_SELECT, SPI_FULL_SPEED)) {
    sdErrorMsg("\ncardBegin failed");
    return;
  }
  t = millis() - t;

  cardSize = sd.card()->cardSize();
  if (cardSize == 0) {
    sdErrorMsg("cardSize failed");
    return;
  }
  cout << F("\ninit time: ") << t << " ms" << endl;
  cout << F("\nCard type: ");
  switch (sd.card()->type()) {
  case SD_CARD_TYPE_SD1:
    cout << F("SD1\n");
    break;

  case SD_CARD_TYPE_SD2:
    cout << F("SD2\n");
    break;

  case SD_CARD_TYPE_SDHC:
    if (cardSize < 70000000) {
      cout << F("SDHC\n");
    } else {
      cout << F("SDXC\n");
    }
    break;

  default:
    cout << F("Unknown\n");
  }
  if (!cidDmp()) {
    return;
  }
  if (!csdDmp()) {
    return;
  }
  uint32_t ocr;
  if (!sd.card()->readOCR(&ocr)) {
    sdErrorMsg("\nreadOCR failed");
    return;
  }
  cout << F("OCR: ") << hex << ocr << dec << endl;
  if (!partDmp()) {
    return;
  }
  if (!sd.fsBegin()) {
    sdErrorMsg("\nFile System initialization failed.\n");
    return;
  }
  volDmp();
  HWSERIAL.print("\x1B[0;0H");
  HWSERIAL.println("END");

  for (int i=0; i<data_write_idx; i++) {
    if (data_buf[i] > 20 || data_buf[i] < -20) {
      cout << i << " " << (int32_t)data_buf[i] << endl;
    }
  }
  cout << data_write_idx << endl;
  cout << wavCount << endl;
  // int test = digitalRead(MOTOR_PIN);
  // if (test == HIGH) {
  //   cout << "MOTOR ON" << endl;
  // }

  data_write_idx = 0;
  wavCount = 0;
  writeFlag = !writeFlag;
  if (writeFlag) {
    cout << "entering write mode" << endl;
  }
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
