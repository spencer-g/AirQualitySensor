#include <pms5003.h>
#include <dht11.h>
#include <SoftwareSerial.h>

#define humiPin  8
#define air1Rx  2
#define air1Tx  3
#define air2Rx  5
#define air2Tx  6
#define triggerWidth  2000
#define stabilizer  45000
#define transmitThreshold  10
#define transmitFrequency  120000
#define percentAllowed  0.05
#define inconsistenciesAllowed  2

SoftwareSerial air1Pins (air1Rx, air1Tx);
SoftwareSerial air2Pins (air2Rx, air2Tx);
pms5003 AIR;
dht11 HUMI;

long prevTrigger = stabilizer;
long prevTransmit = 0;
float averages[14];
bool sensorsAwake = false;
bool dataFlag = false;
uint8_t dataInAvg = 0;
uint8_t inconsistentCount = 0;

void setup() {
  Serial.begin(9600);
  Serial.println(F("Beginning program..."));
  pinMode(air1Tx, OUTPUT);
  pinMode(air1Rx, INPUT);
  pinMode(air2Tx, OUTPUT);
  pinMode(air2Rx, INPUT);
  air1Pins.begin(9600);
  air2Pins.begin(9600);
  AIR.setPins(&air1Pins);
  sensorWake();
  prevTrigger += millis();
}

void sensorWake(){
  for(int8_t i = 0; i < 2; i++){
    int8_t w = AIR.wakeup();
    int8_t a = AIR.activeRead();
    switchAirSensor();
  }
  sensorsAwake = true;
}

void sensorSleep(){
  for(int8_t i = 0; i < 2; i++){
    int8_t p = AIR.passiveRead();
    int8_t s = AIR.sleep();
    switchAirSensor();
  }
  sensorsAwake = false;
}

void loop() {
  long currentMillis = millis();
  //following condition will handle millis() overflow which occurs after roughly 50 days of an Arduino running.
  if (currentMillis < 0 && prevTrigger > 0){
    prevTrigger = currentMillis;
  }
  if( !sensorsAwake && 
   currentMillis - (prevTransmit - (transmitThreshold * triggerWidth)) >= transmitFrequency)
  {
    sensorWake();
    prevTrigger = currentMillis + stabilizer;
    //need a conditional here to deal with possible overflow
    //while (prevTrigger < currentMillis){
    //  delay(1);
    //  currentMillis = millis();
    //}  ??? uncertain
  }
  
  if(sensorsAwake && currentMillis - prevTrigger >= triggerWidth){
    prevTrigger = currentMillis;
    doStuff();
  }
}

void doStuff(){
  Serial.print(F("\nProgram running for "));
  Serial.print(millis() / 1000);
  Serial.println(F(" seconds."));
  int8_t result;
  int dataDiff[12];
  for(int8_t i = 1; i <= 2; i++){
    Serial.print(F("Air Sensor "));
    Serial.print(i);
    Serial.print(F(" Read Result: "));
    result = AIR.read();
    switch(result){
      case PMSLIB_OK : 
        Serial.println(F("Success"));
        break;
      case PMSLIB_ERROR_TIMEOUT :
        Serial.println(F("Timeout Error. Check connections. Maybe lengthen Timer value"));
        break;
      case PMSLIB_ERROR_CHECKSUM :
        Serial.println(F("Check sum error. Debug library."));
        break;
      defualt :
        Serial.println(F("Unknown error. Big oops."));
        break;
    }
    for(int8_t j = 0; j < 12; j++){
      if(i == 1){
        dataDiff[j] = 0;
      }
     dataDiff[j] += (pow(-1, i) * AIR.data[j]);
    }
    switchAirSensor();
  }//end outer for loop
  
  Serial.print(F("Humi Sensor Read: "));
  result = HUMI.read(humiPin);
  switch (result){
    case DHTLIB_OK : 
      Serial.println(F("Success"));
      break;
    case DHTLIB_ERROR_TIMEOUT : 
      Serial.println(F("Failure due to timeout. Check connection"));
      break;
    case DHTLIB_ERROR_CHECKSUM :
      Serial.println(F("Failure due to checksum bits. Debug library."));
      break;
  }
  inconsistentCount += compareAirSensors(dataDiff);
  addToAverage();
  if(inconsistentCount > inconsistenciesAllowed){
    dataFlag = true;
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

int8_t compareAirSensors(int dataDiff[]){
  int8_t idx = 0, err = 0;
  while(err == 0 && idx < 12){
    if (AIR.data[idx] == 0){
      if(abs(dataDiff[idx]) > 1){
        err++;  
      }
    }
    else{
      float dD = (float)dataDiff[idx];
      float dA = (float)AIR.data[idx];
      float percentDiff = dD / dA;
      if(percentDiff > percentAllowed || (-1 * percentDiff) > percentAllowed){
        err++;
      }
    }
    idx++;
  }
  if(err != 0){
    Serial.println(F("Flagged as inaccurate"));
  }
  return err;
}

void addToAverage(){
  for(int8_t i = 0; i < 12; i++){
    averages[i] += AIR.data[i];
  }
  averages[12] += HUMI.humidity;
  averages[13] += HUMI.temperature;
  dataInAvg++;
  
  if(dataInAvg == transmitThreshold){
    sensorSleep();
    for(int8_t i = 0; i < 14; i++){
      averages[i] = averages[i] / (float)dataInAvg;
    }
    sendDataToServer();

    //reset averages and tracking variables
    for(int8_t i = 0; i < 14; i++){
      averages[i] = 0;
    }
    dataInAvg = 0;
    inconsistentCount = 0;
    dataFlag = false;
  }
}

void sendDataToServer(){
  prevTransmit = millis();
  //code to send data should go here.
  //printing to monitor as placeholder
  Serial.println(F("\nFollowing are averages collected over the past 30 sec."));
  Serial.print(F("Data Flagged for Inconsistency: "));
  if(dataFlag){
    Serial.print(F("True "));
  }
  else{
    Serial.print(F("False "));
  }
  Serial.println(inconsistentCount);
  Serial.print(F("PMS 1.0: "));
  Serial.println(averages[0], 2);
  Serial.print(F("PMS 2.5: "));
  Serial.println(averages[1], 2);
  Serial.print(F("PMS 10: "));
  Serial.println(averages[2], 2);
  Serial.print(F("PMS 1.0 (AE): "));
  Serial.println(averages[3], 2);
  Serial.print(F("PMS 2.5 (AE): "));
  Serial.println(averages[4], 2);
  Serial.print(F("PMS 10 (AE): "));
  Serial.println(averages[5], 2);
  Serial.print(F("Bin 0.3: "));
  Serial.println(averages[6], 2);
  Serial.print(F("Bin 0.5: "));
  Serial.println(averages[7], 2);
  Serial.print(F("Bin 1.0: "));
  Serial.println(averages[8], 2);
  Serial.print(F("Bin 2.5: "));
  Serial.println(averages[9], 2);
  Serial.print(F("Bin 5.0: "));
  Serial.println(averages[10], 2);
  Serial.print(F("Bin 10: "));
  Serial.println(averages[11], 2);
  Serial.print(F("Humidity: "));
  Serial.println(averages[12], 2);
  Serial.print(F("Temperature: "));
  Serial.println(averages[13], 2);
}

