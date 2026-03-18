#include <Wire.h>
#include <DynamixelShield.h>

#define DXL_ID      1
#define DXL_DIR_PIN 2
#define I2C_ADDR    0x08

DynamixelShield dxl(Serial, DXL_DIR_PIN);
int velocity = 0;

void receiveCommand(int bytes) {
  String cmd = "";
  while (Wire.available()) cmd += (char)Wire.read();
  if      (cmd == "UP")   velocity =  256;
  else if (cmd == "DOWN") velocity = -256;
  else if (cmd == "STOP") velocity =  0;
}

void setup() {
  Wire.begin(I2C_ADDR);
  Wire.onReceive(receiveCommand);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);
  dxl.torqueOff(DXL_ID);
  dxl.writeControlTableItem(68, DXL_ID, 1);
  dxl.setOperatingMode(DXL_ID, OP_VELOCITY);
  dxl.torqueOn(DXL_ID);
}

void loop() {
  dxl.setGoalVelocity(DXL_ID, velocity);
  delay(20);
}