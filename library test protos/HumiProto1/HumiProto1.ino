#include <dht11.h>
const byte humiPin = 8;
const long humiTriggerWidth = 5000;
unsigned long prevHumiTrigger = 0;
unsigned long currentMillis = 0;
dht11 HUMI;
void setup() {
  Serial.begin(9600);
  Serial.println("Starting Humi tester");
}

void loop() {
  currentMillis = millis();
  if(currentMillis - prevHumiTrigger >= humiTriggerWidth){
    prevHumiTrigger = currentMillis;
    Serial.print("Running for ");
    Serial.print(currentMillis / 1000);
    Serial.println(" seconds."); 
    Serial.print("Sensor Read: ");
    int result = HUMI.read(humiPin);
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
