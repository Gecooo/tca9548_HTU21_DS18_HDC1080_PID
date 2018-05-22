
///////////////////

//НА платах nano, uno не работает!!!

///////////////////




#include <PID_v1.h>

#include <PWM.h>
//#include "SparkFunHTU21D.h"
#include <ArduinoJson.h>
#include <Wire.h>
#include <HTU21D.h>
#include <DallasTemperature.h>
#include <ClosedCube_HDC1080.h>
 
#define TCAADDR 0x70

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2

#define INT1 6 // H-bridge leg 1 ->INT1
#define INT2 7 // H-bridge leg 2 ->INT2

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress hottemp = {0x28, 0xFF, 0x55, 0x6E, 0x80, 0x16, 0x05, 0x91}; // горячая сторона пелетье
DeviceAddress uptemp = {0x28, 0xFF, 0x06, 0x54, 0x75, 0x16, 0x03, 0x7C};
DeviceAddress seredtemp = {0x28, 0xFF, 0x80, 0x50, 0x75, 0x16, 0x03, 0x25};
DeviceAddress niztemp = {0x28, 0xFF, 0x0D, 0xBB, 0x15, 0x15, 0x03, 0x2F};

/* Assign a unique ID to this sensor at the same time */
HTU21D myHumidity1, myHumidity2;
ClosedCube_HDC1080 hdc1080;

unsigned long prevMillissens = 0;

///////Memory////////
extern void *__brkval;
extern int __bss_end;
///////////////////////////

float Hottemp, Uptemp, Seredtemp, Niztemp;
double Input, Output, Setpoint;
double aggKp=8000.0, aggKi=400.0, aggKd=100.0;       //настройки PID-регулятора
//double consKp=8000, consKi=153.0, consKd=10.3;
double consKp=8000, consKi=53.0, consKd=10.3;
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, REVERSE);       //PID-регулятор элемента пелетье


void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial3.begin(115200);
  Serial.println("HTU21D Example!");
  InitTimersSafe();
  SetPinFrequencySafe(INT1, 25000);
  SetPinFrequencySafe(INT2, 25000);
   myPID.SetOutputLimits(10000,65000);    //лимит PID  
   myPID.SetSampleTime(1000);
  sensors.begin();
  //delay(50);
  /* Initialise the 1st sensor */
  tcaselect(0);
myHumidity1.begin();
myHumidity1.setResolution(HTU21D_RES_RH12_TEMP14);
//Serial.println(F("DEMO 3: Battery1 Status"));
//  if   (myHumidity1.batteryStatus() == true) Serial.println(F("Battery1.........: OK.  Level > 1.90v"));
//  else                                    Serial.println(F("Battery1.........: LOW. Level < 1.90v"));
//
//
//  /* DEMO - 4 */
//  Serial.println(F("DEMO 4:"));
//  Serial.print(F("Firmware version1....: ")); Serial.println(myHumidity1.readFirmwareVersion());
//
//
//  /* DEMO - 5 */
//  Serial.println(F("DEMO 5:"));
//  Serial.print(F("Sensor's1 ID.........: ")); Serial.println(myHumidity1.readDeviceID());

  tcaselect(2); 
  hdc1080.begin(0x40); 
  hdc1080.setResolution(1, 01);
    
    /* Initialise the 2nd sensor */
  tcaselect(5);
   myHumidity2.begin();
   myHumidity1.setResolution(HTU21D_RES_RH12_TEMP14);

Serial.println("HTU21D --- OK");
delay(1000);
}

void loop() {
  if (millis() - prevMillissens >= 1000) {
    sensors.requestTemperatures();
    Hottemp = sensors.getTempC(hottemp);
    Uptemp = sensors.getTempC(uptemp);
    Seredtemp = sensors.getTempC(seredtemp);
    Niztemp = sensors.getTempC(niztemp);
    
  // put your main code here, to run repeatedly:
  tcaselect(0); // с впаяным кондером
  float humd = myHumidity1.readHumidity();
  float temp = myHumidity1.readTemperature();
  float hum_compens = myHumidity1.readCompensatedHumidity();
//  Serial.print("Humidity1:    "); Serial.print(humd, 1);
//  Serial.print("\tTemperature1: "); Serial.println(temp, 1);
//  Serial.print("Comp Hum1:    "); Serial.println(hum_compens, 1);
//  Serial.println("");
 
  tcaselect(5);
 float humd1 = myHumidity2.readHumidity();
  float temp1 = myHumidity2.readTemperature();
  float hum_compens1 = myHumidity2.readCompensatedHumidity();
//  Serial.print("Humidity2:    "); Serial.print(humd1-0.2, 1);
//  Serial.print("\tTemperature2: "); Serial.println(temp1-0.4, 1);
//  Serial.print("Comp Hum2:    "); Serial.print(hum_compens1, 1);
// Serial.println("");
 

 tcaselect(2);
 float hum_hdc1080 = hdc1080.readHumidity();
 float temp_hdc1080 = hdc1080.readTemperature();
// Serial.print("Hum hdc1080:  "); Serial.print(hum_hdc1080, 1);
//  Serial.print("\tTemp hdc1080: "); Serial.println(temp_hdc1080);
//  Serial.println("");
//  
// Serial.print("Hottemp:    "); Serial.println(Hottemp);
//  Serial.print("Uptemp:    "); Serial.println(Uptemp);
//  Serial.print("Seredtemp: "); Serial.println(Seredtemp);
//  Serial.print("Niztemp:   "); Serial.println(Niztemp);
// 
// Serial.println("");
// Serial.println("");
  // delay(1000);

  const size_t bufferSize = JSON_ARRAY_SIZE(3) + JSON_ARRAY_SIZE(7) + JSON_OBJECT_SIZE(4);
DynamicJsonBuffer jsonBuffer(bufferSize);

JsonObject& root = jsonBuffer.createObject();
root["sensor"] = "termokamera";
root["time"] = millis();

JsonArray& temperatura = root.createNestedArray("temperatura");
temperatura.add(temp);
temperatura.add(temp1);
temperatura.add(temp_hdc1080);
temperatura.add(Hottemp);
temperatura.add(Uptemp);
temperatura.add(Seredtemp);
temperatura.add(Niztemp);

JsonArray& humidity = root.createNestedArray("humidity");
humidity.add(hum_compens);
humidity.add(hum_compens1);
humidity.add(hum_hdc1080);

root.printTo(Serial3);
Serial3.println();

  prevMillissens = millis();
  }
  Pidregulator();
}

///// показывает свободную память ///////
int memoryFree() {
  int freeValue;
  if ((int)__brkval == 0)
    freeValue = ((int)&freeValue) - ((int)&__bss_end);
  else
    freeValue = ((int)&freeValue) - ((int)__brkval);
  return freeValue;
}

