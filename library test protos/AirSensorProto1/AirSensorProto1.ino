#include <pms5003.h>
#ifndef SOFTWARE_SERIAL
#include <SoftwareSerial.h>
#define SOFTWARE_SERIAL
#endif

const byte air1Rx = 2;
const byte air1Tx = 3;
long prevTrigger = 30000;
int triggerWidth = 5000;
long currentMillis;

SoftwareSerial air1Pins (air1Rx, air1Tx);
pms5003 air1;

void setup() {
  pinMode(air1Tx, OUTPUT);
  pinMode(air1Rx, INPUT);
  air1Pins.begin(9600);
  air1.setPins(&air1Pins);
  air1.wakeup();
  air1.activeRead();
  Serial.begin(9600);
  Serial.println("Beginning program...");
}

void loop() {
  currentMillis = millis();
  if(currentMillis - prevTrigger >= triggerWidth){
    prevTrigger = currentMillis;
    Serial.print("\nProgram running for ");
    Serial.print(currentMillis / 1000);
    Serial.println(" seconds.");
    Serial.print("Air Sensor Read Result: ");
    int8_t result = air1.read();
    switch(result){
      case PMSLIB_OK : 
        Serial.println("Success");
        break;
      case PMSLIB_ERROR_TIMEOUT :
        Serial.println("Timeout Error. Check connections. Maybe lengthen Timer value");
        break;
      case PMSLIB_ERROR_CHECKSUM :
        Serial.println("Check sum error. Debug library.");
        break;
      defualt :
        Serial.println("Unknown error. Big oops.");
        break;
    }
    Serial.print("PMS 1.0: ");
    Serial.println(air1.pm_1_0);
    Serial.print("PMS 2.5: ");
    Serial.println(air1.pm_2_5);
    Serial.print("PMS 10.0: ");
    Serial.println(air1.pm_10);
    Serial.print("PMS 1.0 (Atmospheric Environment): ");
    Serial.println(air1.pm_1_0_ae);
    Serial.print("PMS 2.5 (Atmospheric Environment): ");
    Serial.println(air1.pm_2_5_ae);
    Serial.print("PMS 10.0 (Atmospheric Environment): ");
    Serial.println(air1.pm_10_ae);
    Serial.print("Count of particles beyond 0.3 um: ");
    Serial.println(air1.bin_0_3);
    Serial.print("Count of particles beyond 0.5 um: ");
    Serial.println(air1.bin_0_5);
    Serial.print("Count of particles beyond 1.0 um: ");
    Serial.println(air1.bin_1_0);
    Serial.print("Count of particles beyond 2.5 um: ");
    Serial.println(air1.bin_2_5);
    Serial.print("Count of particles beyond 5.0 um: ");
    Serial.println(air1.bin_5_0);
    Serial.print("Count of particles beyond 10.0 um: ");
    Serial.println(air1.bin_10);
    Serial.println("*****END OF DATA*****");
  }
}
