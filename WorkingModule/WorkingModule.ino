#include <pms5003.h>
#include <dht11.h>
#include "Adafruit_FONA.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_FONA.h"
#include <SoftwareSerial.h>

/*************************** Air Sensors ************************************/
#define humiPin  8
#define air1Rx  9
#define air1Tx  10
#define air2Rx  11
#define air2Tx  12

SoftwareSerial air1Pins (air1Rx, air1Tx);
SoftwareSerial air2Pins (air2Rx, air2Tx);
pms5003 AIR;
dht11 HUMI;

/*************************** FONA Settings ***********************************/
#define FONA_RX  2
#define FONA_TX  3
#define FONA_RST 4
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

//if changing the FONA_APN, it must be done here as well as in fonaHelper.cpp and MQTT_FONA.cpp
#define FONA_APN       "hologram"
#define FONA_USERNAME  ""
#define FONA_PASSWORD  ""

/************************* Adafruit.io Setup *********************************/
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "sarmavrudhula"
#define AIO_KEY         "1c81409abf1c46b1af79cbdfec6a2e83"

// Setup the FONA MQTT class by passing in the FONA class and MQTT server and login details.
Adafruit_MQTT_FONA mqtt(&fona, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// FONAconnect is a helper function that sets up the FONA and connects to
// the GPRS network. See the fonahelper.cpp tab above for the source!
boolean FONAconnect(const __FlashStringHelper *apn, const __FlashStringHelper *username, const __FlashStringHelper *password);

/*************************** MQTT Feeds **************************************/

// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish airDataPM2_5 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/airdata-pm-2-5/csv");
Adafruit_MQTT_Publish airDataPM10 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/airdata-pm-10/csv");
Adafruit_MQTT_Publish humidityFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity/csv");
Adafruit_MQTT_Publish temperatureFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature/csv");
Adafruit_MQTT_Publish batteryFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/battery/csv");
Adafruit_MQTT_Publish badDataFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/baddata/csv");

/*************************** Sketch Code ************************************/

// How many transmission failures in a row we're willing to be ok with before reset
uint8_t txfailures = 0;
#define MAXTXFAILURES 3

//timing variables for transmission
unsigned long previousMillis;
unsigned long currentMillis;
#define triggerWidth  2000  //time in between measurements of Particulate Mass
#define stabilizer  45000   //time it takes for air sensors to stabilize readings
#define transmitFrequency  5 * 60 * 1000   //time in between transmits
long prevTrigger = stabilizer;
long prevTransmit = 0;

//variables for averaging sensor readings
#define transmitThreshold  10 //number of data points to be averaged per transmission
#define percentAllowed  0.05  //allowable difference between 2 air sensor reading values before data is marked as bad
#define inconsistenciesAllowed  2  //number of bad data points in an average before the entire average is thrown out.
float averages[14];
bool dataFlag = false;
uint8_t dataInAvg = 0;
uint8_t inconsistentCount = 0;

bool sensorsAwake = false; // used to see if sensors are active or in power save mode

void handleSerial() {
  byte incomingChar = Serial.read();
  // a way to stop all activity (requires reset)
  if (incomingChar == '!') halt(F("User stopped operation"));

  // force reconnecting, works well when IP fails and getting DEAC errors
  if (incomingChar == '#') FONAconnect(F(FONA_APN), F(FONA_USERNAME), F(FONA_PASSWORD));

  // Temp pause all activity
  if (incomingChar == '@') {
    Serial.println(F("Pausing..."));
    boolean pause = true;
    while(pause) if (Serial.read() == '@') pause = false;
    Serial.println(F("Resuming..."));
  }
}

void halt(__FlashStringHelper* s) { 
  Serial.println( s ); 
  while(1);
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
  Serial.print("Connecting to MQTT... ");

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("[Connecting] Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    Serial.println(F("[Connecting] Pausing for 5 seconds"));
    delay(5000);  // wait 5 seconds
    handleSerial();
  }
  Serial.println("MQTT Connected!");
}

void setup() {
  // give time for everything to start up
  delay(2000);

  Serial.begin(115200);

 // Initialise the FONA module
  while (! FONAconnect(F(FONA_APN), F(FONA_USERNAME), F(FONA_PASSWORD)))
    Serial.println(F("Retrying FONA"));

  Serial.println(F("Connected to Cellular!"));

  delay(2500);  // wait a few seconds to stabilize connection

  Serial.print(F("Turning on GPS..."));
  float latitude, longitude, speed, heading, altitude;
  fona.enableGPS(true);
  Serial.println(F("done"));
  while(!fona.getGPS(&latitude, &longitude, &speed, &heading, &altitude)){
    Serial.println(F("no gps fix... wait 5 sec"));
    delay(5000);
  }

  //setup air sensors
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

//sends wakeup signal to both air sensors
void sensorWake(){
  for(int8_t i = 0; i < 2; i++){
    int8_t w = AIR.wakeup();
    int8_t a = AIR.activeRead();
    switchAirSensor();
  }
  sensorsAwake = true;
}

//sends sleep signal to both air sensors
void sensorSleep(){
  for(int8_t i = 0; i < 2; i++){
    int8_t p = AIR.passiveRead();
    int8_t s = AIR.sleep();
    switchAirSensor();
  }
  sensorsAwake = false;
}

void loop() {
  handleSerial();
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
    readSensors();
    addToAverage();
    if(dataInAvg == transmitThreshold){
      sensorSleep();
      for(int8_t i = 0; i < 14; i++){
        averages[i] = averages[i] / (float)dataInAvg;
      }
      sendDataToServer();
    }
  }
}

void readSensors(){
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
}
  
void resetVars(){
  //reset averages and tracking variables
  for(int8_t i = 0; i < 14; i++){
    averages[i] = 0;
  }
  dataInAvg = 0;
  inconsistentCount = 0;
  dataFlag = false;
}

void sendDataToServer(){
  if(!fonaSS.isListening() ){
    fonaSS.listen();
  }
  prevTransmit = millis();

  //start debug printing
  Serial.println(F("\nFollowing are averages collected over the past 30 sec."));
  Serial.print(F("Data Flagged for Inconsistency: "));
  if(dataFlag){
    Serial.print(F("True "));
  }
  else{
    Serial.print(F("False "));
  }
  Serial.println(inconsistentCount);
  Serial.print(F("PMS 2.5 (AE): "));
  Serial.println(averages[4], 2);
  Serial.print(F("PMS 10 (AE): "));
  Serial.println(averages[5], 2);
  Serial.print(F("Humidity: "));
  Serial.println(averages[12], 2);
  Serial.print(F("Temperature: "));
  Serial.println(averages[13], 2);
  //end debug printing

  float latitude, longitude, speed, heading, altitude;
  int vBatt;
  fona.getBattPercent(&vBatt);
  fona.getGPS(&latitude, &longitude, &speed, &heading, &altitude);
  bool dataSent;

  String commaSpace = ", ";
  String latStr = String(latitude, 5);
  String longStr = String(longitude, 5);
  String altStr = String(altitude, 2);
  String gpsStr = commaSpace + latStr + commaSpace + longStr + commaSpace + altStr;
  delete &commaSpace;
  delete &latStr;
  delete &longStr;
  delete &altStr;

  String badData;
  if(dataFlag){
    badData = String(1);
  }
  else{
    badData = String(0);
  }
  badData += gpsStr;
  unsigned int len = badData.length();
  char badDataMsg[len];
  badData.toCharArray(badDataMsg, len);
  delete &badData;
  MQTT_connect();
  badDataFeed.publish(badDataMsg);
  delete badDataMsg;
  if(dataFlag){
    return;
  }
  
  String pm25Str = String(averages[4], 2);
  pm25Str += gpsStr;
  len = pm25Str.length();
  char pm25Msg[len];
  pm25Str.toCharArray(pm25Msg, len);
  delete &pm25Str;
  MQTT_connect();
  dataSent = airDataPM2_5.publish(pm25Msg);
  delete pm25Msg;
  
  String pm10Str = String(averages[5], 2);
  pm10Str += gpsStr;
  len = pm10Str.length();
  char pm10Msg[len];
  pm10Str.toCharArray(pm10Msg, len);
  delete &pm10Str;
  MQTT_connect();
  dataSent = airDataPM10.publish(pm10Msg) && dataSent;
  delete pm10Msg;
  
  String humiStr = String(averages[12], 2);
  humiStr += gpsStr;
  len = humiStr.length();
  char humiMsg[len];
  humiStr.toCharArray(humiMsg, len);
  delete &humiStr;
  MQTT_connect();
  dataSent = humidityFeed.publish(humiMsg) && dataSent;
  delete humiMsg;
    
  String tempStr = String(averages[13], 2);
  tempStr += gpsStr;
  len = tempStr.length();
  char tempMsg[len];
  tempStr.toCharArray(tempMsg, len);
  delete &tempStr;
  MQTT_connect();
  dataSent = temperatureFeed.publish(tempMsg) && dataSent;
  delete tempMsg;
  
  String battStr = String(vBatt);
  battStr += gpsStr;
  len = battStr.length();
  char battMsg[len];
  battStr.toCharArray(battMsg, len);
  delete &battStr;
  MQTT_connect();
  dataSent = batteryFeed.publish(battMsg) && dataSent;
  delete battMsg;

  delete &gpsStr;
  
  if(!dataSent){
    Serial.println(F("Data failed to publish"));
    txfailures++;
    if (txfailures >= MAXTXFAILURES) {
      Serial.println(F("Resetting Cellular"));
      FONAconnect(F(FONA_APN), F(FONA_USERNAME), F(FONA_PASSWORD));
      txfailures = 0;
    }
  } 
  else {
    Serial.println(F("Sent Data!"));
    txfailures = 0;
  }  
}

