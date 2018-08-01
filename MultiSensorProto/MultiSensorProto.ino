#include <pms5003.h>
#include <dht11.h>
#ifndef SOFTWARE_SERIAL
#include <SoftwareSerial.h>
#define SOFTWARE_SERIAL
#endif

const byte humiPin = 8;
const byte air1Rx = 2;
const byte air1Tx = 3;
const byte air2Rx = 5;
const byte air2Tx = 6;
long prevTrigger = 30000;
const int triggerWidth = 5000;
long currentMillis;

SoftwareSerial air1Pins (air1Rx, air1Tx);
SoftwareSerial air2Pins (air2Rx, air2Tx);
pms5003 AIR;
dht11 HUMI;

void setup() {
  pinMode(air1Tx, OUTPUT);
  pinMode(air1Rx, INPUT);
  pinMode(air2Tx, OUTPUT);
  pinMode(air2Rx, INPUT);
  air1Pins.begin(9600);
  air2Pins.begin(9600);
  AIR.setPins(&air1Pins);
  AIR.wakeup();
  AIR.activeRead();
  AIR.setPins(&air2Pins);
  AIR.wakeup();
  AIR.activeRead();
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
    int8_t result;
    for(int8_t i = 1; i <= 2; i++){
      Serial.print("Air Sensor ");
      Serial.print(i);
      Serial.print(" Read Result: ");
      result = AIR.read();
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
      Serial.println(AIR.pm_1_0);
      Serial.print("PMS 2.5: ");
      Serial.println(AIR.pm_2_5);
      Serial.print("PMS 10.0: ");
      Serial.println(AIR.pm_10);
      Serial.print("PMS 1.0 (Atmospheric Environment): ");
      Serial.println(AIR.pm_1_0_ae);
      Serial.print("PMS 2.5 (Atmospheric Environment): ");
      Serial.println(AIR.pm_2_5_ae);
      Serial.print("PMS 10.0 (Atmospheric Environment): ");
      Serial.println(AIR.pm_10_ae);
      Serial.print("Count of particles beyond 0.3 um: ");
      Serial.println(AIR.bin_0_3);
      Serial.print("Count of particles beyond 0.5 um: ");
      Serial.println(AIR.bin_0_5);
      Serial.print("Count of particles beyond 1.0 um: ");
      Serial.println(AIR.bin_1_0);
      Serial.print("Count of particles beyond 2.5 um: ");
      Serial.println(AIR.bin_2_5);
      Serial.print("Count of particles beyond 5.0 um: ");
      Serial.println(AIR.bin_5_0);
      Serial.print("Count of particles beyond 10.0 um: ");
      Serial.println(AIR.bin_10);
      Serial.println();
      switchAirSensor();
    }//end for loop

    Serial.print("Sensor Read: ");
    result = HUMI.read(humiPin);
    switch (result){
      case DHTLIB_OK : 
        Serial.println("Success");
        break;
      case DHTLIB_ERROR_TIMEOUT : 
        Serial.println("Failure due to timeout. Check connection");
        break;
      case DHTLIB_ERROR_CHECKSUM :
        Serial.println("Failure due to checksum bits. Possible library incompatibilities.");
        break;
    }
    Serial.print("Temp read out: ");
    Serial.println(HUMI.temperature);
    Serial.print("Humidity read out: ");
    Serial.println(HUMI.humidity);
  }
}

void switchAirSensor(){
  if(air1Pins.isListening()){
    air2Pins.listen();
  }
  else{
    air1Pins.listen();
  }
}
