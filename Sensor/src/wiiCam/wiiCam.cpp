#include "wiiCam.h"
#include "../IrPoint/IrPoint.h"
#include "Arduino.h"
#include <Wire.h>


#define IR_ADDRESS 0x58   //I2C address
#define MSGSIZE 40        //I2C message size
#define I2C_CLK 400000    //I2C clock frequency

wiiCam::wiiCam(uint8_t sda, uint8_t scl){
  _sda = sda;
  _scl = scl;
}

bool wiiCam::begin(){
  Wire.setPins(_sda, _scl);
  Wire.begin();
  Wire.setClock(I2C_CLK);

  while(1) {
    Wire.beginTransmission(IR_ADDRESS);
    uint8_t err = Wire.endTransmission();
    if (err == 0) break;
    else if (err == 4) Serial.println("WiiMote Sensor: Unknown error");
    else Serial.println("WiiMote Sensor: Not found");
    delay(5000);
  }
  
  writeRegister(CONFIG, ENABLE_bm);
  setOutputMode(MODE_FULL);
  setPixelMaxBrightnessThreshold(255);
  setPixelBrightnessThreshold(5);
  writeRegister(MAX_BRIGHTNESS, 0);
  updateRegisters();

  _framePeriodTimer = micros();
  return true;
}

float wiiCam::getFramePeriod(){
  return _framePeriod;
}

void wiiCam::setFramePeriod(float period){
  _framePeriod = period;
}

// Useful values on wiicam seem to be from about 10 to
// 90, so no need to try to compute values between 1
// and 255.  Values less than 12 results in no
// reading either, so keep brightness in detectable range.
// There are big jumps - with testing with DIY LED, sensitity
// of 100 is ~70 brightness, 99 is ~40, 98 is ~30,
// and 90 is ~16, and the change slows down more
// with lower values.
//
// This is based on http://wiibrew.org/wiki/Wiimote#IR_Camera,
// The max sensitivity there has sb1[8] set to 0x0C (12),
// which matches with observation noted above.
//
// Other values in the sb1 block did not result in any change
// in behavior with testing - only one of note is
// sb1[6] which if set too low results in the camera
// not detecting any IR sources, and it being
// 0x20 results in IR source popping in and out but
// with same brightness as if set with other values.
// Perhaps sensitivity related in some way.  0x90
// chosen here as something basically in the middle
// of usable values.  The test environment was not noisy,
// so unable to experiment with different values here
// to see what may work better.
//
// Unclear what exactly the sb2 values are used for -
// it must be less than sb1[8], but playing with
// different values resulted in no difference in
// noise or brightness levels.
void wiiCam::setSensitivity(uint8_t val) {
  uint8_t brightness = 112 - val;
  uint8_t sb1[9] = {0x02, 0x00, 0x00, 0x71, 0x01, 0x00, 0x90, 0x00, 0xff}, sb2[2] = {0x00, 0x00};

  sb1[8] = brightness;
  sb2[0] = brightness - 1;

  Wire.beginTransmission(IR_ADDRESS);
  Wire.write(SENSITIVITY_BLOCK1);
  for (int i=0; i<9; i++)
      Wire.write(sb1[i]);
  Wire.endTransmission();
  delay(10);

  Wire.beginTransmission(IR_ADDRESS);
  Wire.write(MAX_BRIGHTNESS);
  Wire.write(sb2[0]);
  Wire.write(sb2[1]);
  Wire.endTransmission();
  delay(10);

  Wire.beginTransmission(IR_ADDRESS);
  Wire.write(CONFIG);
  Wire.write(0x08);
  Wire.endTransmission();
  delay(10);
}

void wiiCam::setPixelBrightnessThreshold(uint8_t value){
  writeRegister(MIN_BRIGHTNESS_THR,value);
}

uint8_t wiiCam::getPixelBrightnessThreshold(){
  return readRegister(MIN_BRIGHTNESS_THR);
}

void wiiCam::setPixelMaxBrightnessThreshold(uint8_t value){
  writeRegister(MAX_BRIGHTNESS_THR,value);
}

uint8_t wiiCam::getPixelMaxBrightnessThreshold(){
  return readRegister(MAX_BRIGHTNESS_THR);
}

void wiiCam::setResolution(uint8_t x, uint8_t y) {
  if (x > 128) x = 128;
  if (y > 96) y = 96;
  writeRegister(HOR_RES,x);
  writeRegister(VERT_RES,y);
}

bool wiiCam::getInterruptState() {
  if (micros() - _framePeriodTimer >= _framePeriod*1000) {
    _framePeriodTimer = micros();
    return true;
  }
  return false;
}

void wiiCam::setOutputMode(output_mode_t mode) {
  writeRegister(OUTPUT_MODE, mode);
}

bool wiiCam::getOutput(IrPoint *irPoints){
  detectedPoints = 0;

  //IR sensor read
  Wire.beginTransmission(IR_ADDRESS);
  Wire.write(OUTPUT);
  Wire.endTransmission();

  // Request the 2 byte heading (MSB comes first)
  uint8_t bytesToRead = Wire.requestFrom(IR_ADDRESS, MSGSIZE);
  uint8_t outputBuffer[bytesToRead];

  int i=0;
  while(Wire.available() && i < bytesToRead) {
    outputBuffer[i] = Wire.read();
    i++;
  }
 
  for (int i=0; i<4; i++) {
    uint8_t brightness = outputBuffer[9+i*9];
    if (brightness == 255) {
      irPoints[i].invalidCount++;
    }
    else {
      detectedPoints++;
      irPoints[i].invalidCount = 0;
      irPoints[i].setArea(outputBuffer[3+i*9]&&15);
      irPoints[i].setXRaw((((outputBuffer[3+i*9]>>4)&3) << 8 | outputBuffer[1+i*9])*4);
      irPoints[i].setYRaw((((outputBuffer[3+i*9]>>6)&3) << 8 | outputBuffer[2+i*9])*4);
      irPoints[i].setMaxBrightness(brightness);
      irPoints[i].setAvgBrightness(brightness);
    }
    irPoints[i].updateData();
  }
  return detectedPoints;
}

uint32_t wiiCam::readRegister(uint8_t reg){
  Wire.beginTransmission(IR_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(IR_ADDRESS, 1);
  return Wire.read();
}

/*
 * Write to the sensor register
 */
void wiiCam::writeRegister(uint8_t reg, uint32_t val){
  Wire.beginTransmission(IR_ADDRESS);
  Wire.write(reg); 
  Wire.write(val);
  Wire.endTransmission();
}

void wiiCam::updateRegisters() {
  writeRegister(CONFIG, UPDATE_bm);
}
