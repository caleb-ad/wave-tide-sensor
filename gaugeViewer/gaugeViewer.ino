#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include "UnixTime.h"

//Structure example to receive data
//Must match the sender structure
typedef struct receiveData
{
  uint16_t sendYear;
  uint8_t sendMonth;
  uint8_t sendDay;
  uint8_t sendHour;
  uint8_t sendMinute;
  uint8_t sendSecond;
  uint8_t sendDayOfWeek;
  int sendDist;
  float sendTemp;
  float sendHum;
  int gaugeID;
} receiveData;

//Create a struct_message called myData
receiveData myData;

//UnixTime timeStamp = setDateTime(2000 + myData.sendYear, myData.sendMonth, myData.sendDay, myData.sendHour, myData.sendMinute, myData.sendSecond);
  
void myFunc()
{
  // Do nothing
}
//callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
  memcpy(&myData, incomingData, sizeof(myData));
  myFunc();
  Serial.printf("Bytes received: %d\n", len);
  Serial.printf("Year: %d\n", myData.sendYear);
  Serial.printf("Distance: %dmm\n", myData.sendDist);
  Serial.printf("Temperature [F]: %f\n", myData.sendTemp);
  Serial.printf("Humidity [%]: %f\n", myData.sendHum);
  Serial.printf("Gauge ID: %d\n", myData.gaugeID);
  
}
 
void setup()
{
  //Initialize Serial Monitor
  Serial.begin(115200);
  
  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  Serial.print("ESP32 Board MAC Address:  ");
  Serial.println(WiFi.macAddress());

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop()
{

}
