#include <HX711_ADC.h>
#include <EEPROM.h>
#include <Wire.h>

//HX711 pins:
const int HX711_dout_1 = 53;
const int HX711_sck_1 = 51;
const int HX711_dout_2 = 49;
const int HX711_sck_2 = 47;
const int HX711_dout_3 = 45;
const int HX711_sck_3 = 43;
const int HX711_dout_4 = 41;
const int HX711_sck_4 = 39;

//HX711 constructor (dout pin, sck pin)
HX711_ADC LoadCell_1(HX711_dout_1, HX711_sck_1);
HX711_ADC LoadCell_2(HX711_dout_2, HX711_sck_2);
HX711_ADC LoadCell_3(HX711_dout_3, HX711_sck_3);
HX711_ADC LoadCell_4(HX711_dout_4, HX711_sck_4);

const int calVal_eepromAdress_1 = 0;
const int calVal_eepromAdress_2 = 4;
const int calVal_eepromAdress_3 = 8;
const int calVal_eepromAdress_4 = 12;
unsigned wait = 0;

// Smoother Calc Variables
const int numReadings_1 = 5;
int readings_1[numReadings_1];
int sIndex_1 = 0;
float sTotal_1 = 0;
float sAverage_1 = 0;

const int numReadings_2 = 5;
int readings_2[numReadings_2];
int sIndex_2 = 0;
float sTotal_2 = 0;
float sAverage_2 = 0;

const int numReadings_3 = 5;
int readings_3[numReadings_3];
int sIndex_3 = 0;
float sTotal_3 = 0;
float sAverage_3 = 0;

const int numReadings_4 = 5;
int readings_4[numReadings_4];
int sIndex_4 = 0;
float sTotal_4 = 0;
float sAverage_4 = 0;

// Change Variables
float t_1;
float tChange_1;
boolean Change_1 = false;

float t_2;
float tChange_2;
boolean Change_2 = false;

float t_3;
float tChange_3;
boolean Change_3 = false;

float t_4;
float tChange_4;
boolean Change_4 = false;

//Coordinate
float x, y;

void setup() {
  Serial.begin (115200);
  LoadCell_1.begin();
  LoadCell_2.begin();
  LoadCell_3.begin();
  LoadCell_4.begin();

  float calibrationValue_1;
  float calibrationValue_2;
  float calibrationValue_3;
  float calibrationValue_4;

  //Get the calibration value for each HX711
  EEPROM.get(calVal_eepromAdress_1, calibrationValue_1);
  EEPROM.get(calVal_eepromAdress_2, calibrationValue_2);
  EEPROM.get(calVal_eepromAdress_3, calibrationValue_3);
  EEPROM.get(calVal_eepromAdress_4, calibrationValue_4);

  unsigned long stabilizingtime = 2000;
  boolean _tare = true;

  byte loadcell1_rdy = 0;
  byte loadcell2_rdy = 0;
  byte loadcell3_rdy = 0;
  byte loadcell4_rdy = 0;

  while ((loadcell1_rdy + loadcell2_rdy + loadcell3_rdy + loadcell4_rdy) < 4)
  {
    if (!loadcell1_rdy) loadcell1_rdy = LoadCell_1.startMultiple(stabilizingtime, _tare);
    if (!loadcell2_rdy) loadcell2_rdy = LoadCell_2.startMultiple(stabilizingtime, _tare);
    if (!loadcell3_rdy) loadcell3_rdy = LoadCell_3.startMultiple(stabilizingtime, _tare);
    if (!loadcell4_rdy) loadcell4_rdy = LoadCell_4.startMultiple(stabilizingtime, _tare);
  }
  if (LoadCell_1.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.1 wiring and pin designations");
  }
  if (LoadCell_2.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.2 wiring and pin designations");
  }
  if (LoadCell_3.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.3 wiring and pin designations");
  }
  if (LoadCell_4.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.4 wiring and pin designations");
  }
  LoadCell_1.setCalFactor(calibrationValue_1);
  LoadCell_2.setCalFactor(calibrationValue_2);
  LoadCell_3.setCalFactor(calibrationValue_3);
  LoadCell_4.setCalFactor(calibrationValue_4);

  //Initialize all the index readings used in Smoother() to 0:
  for (int thisReading_1 = 0; thisReading_1 < numReadings_1; thisReading_1++)
    readings_1[thisReading_1] = 0;

  for (int thisReading_2 = 0; thisReading_2 < numReadings_2; thisReading_2++)
    readings_2[thisReading_2] = 0;

  for (int thisReading_3 = 0; thisReading_3 < numReadings_3; thisReading_3++)
    readings_3[thisReading_3] = 0;

  for (int thisReading_4 = 0; thisReading_4 < numReadings_4; thisReading_4++)
    readings_4[thisReading_4] = 0;

  wait = millis();
}

void loop() {
  LoadCell_1.update();
  LoadCell_2.update();
  LoadCell_3.update();
  LoadCell_4.update();

  t_1 = LoadCell_1.getData();
  t_2 = LoadCell_2.getData();
  t_3 = LoadCell_3.getData();
  t_4 = LoadCell_4.getData();

  Smoother_1();
  Smoother_2();
  Smoother_3();
  Smoother_4();

  if (tChange_1 != t_1) {
    tChange_1 = t_1;
    Change_1 = false;
  }
  if (tChange_2 != t_2) {
    tChange_2 = t_2;
    Change_2 = false;
  }
  if (tChange_3 != t_3) {
    tChange_3 = t_3;
    Change_3 = false;
  }
  if (tChange_4 != t_4) {
    tChange_4 = t_4;
    Change_4 = false;
  }

  //Sum of the 4 HX711 sensor
  float sum = (sAverage_1 + sAverage_2 + sAverage_3 + sAverage_4);

  /* Calculate this to get the position for center of pressure (x and y)
      FL = Foot Length ; FW = Foot Width

      xa = FL-dxa ; ya = FW-dya
      xb = FL-dxb ; yb = dyb
      xc = dxc ; yc = FW-dxc
      xd = dxd ; yd = dxd

  */

  float xa = 158;
  float ya = 100;
  float xb = 158;
  float yb = 10;
  float xc = 11;
  float yc = 10;
  float xd = 11;
  float yd = 100;

  x = (sAverage_1 * xa + sAverage_2 * xb + sAverage_3 * xc + sAverage_4 * xd) / sum;
  y = (sAverage_1 * ya + sAverage_2 * yb + sAverage_3 * yc + sAverage_4 * yd) / sum;

  Serial.print(x);
  Serial.print("/");
  Serial.print(y);
  Serial.println();

  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't')
      LoadCell_1.tareNoDelay();
    LoadCell_2.tareNoDelay();
    LoadCell_3.tareNoDelay();
    LoadCell_4.tareNoDelay();
  }
}

void Smoother_1(void) {
  sTotal_1 = sTotal_1 - readings_1[sIndex_1];
  readings_1[sIndex_1] = t_1;
  sTotal_1 = sTotal_1 + readings_1[sIndex_1];
  sIndex_1 = sIndex_1 + 1;

  if (sIndex_1 >= numReadings_1)
    sIndex_1 = 0;
  sAverage_1 = sTotal_1 / numReadings_1;

  if (sAverage_1 != t_1)
    Change_1 = true;
}

void Smoother_2(void) {
  sTotal_2 = sTotal_2 - readings_2[sIndex_2];
  readings_2[sIndex_2] = t_2;
  sTotal_2 = sTotal_2 + readings_2[sIndex_2];
  sIndex_2 = sIndex_2 + 1;

  if (sIndex_2 >= numReadings_2)
    sIndex_2 = 0;
  sAverage_2 = sTotal_2 / numReadings_2;

  if (sAverage_2 != t_2)
    Change_2 = true;
}

void Smoother_3(void) {
  sTotal_3 = sTotal_3 - readings_3[sIndex_3];
  readings_3[sIndex_3] = t_3;
  sTotal_3 = sTotal_3 + readings_3[sIndex_3];
  sIndex_3 = sIndex_3 + 1;


  if (sIndex_3 >= numReadings_3)
    sIndex_3 = 0;
  sAverage_3 = sTotal_3 / numReadings_3;

  if (sAverage_3 != t_3)
    Change_3 = true;
}

void Smoother_4(void) {
  sTotal_4 = sTotal_4 - readings_4[sIndex_4];
  readings_4[sIndex_4] = t_4;
  sTotal_4 = sTotal_4 + readings_4[sIndex_4];
  sIndex_4 = sIndex_4 + 1;

  if (sIndex_4 >= numReadings_4)
    sIndex_4 = 0;
  sAverage_4 = sTotal_4 / numReadings_4;

  if (sAverage_4 != t_4)
    Change_4 = true;
}
