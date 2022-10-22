#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include "UnixTime.h"

#define TARGET_GAUGE 1
#define YEAR 2022

#define ARRAY_SIZE 5000
#define STALE_TIMER 1*1000

bool dataIncoming = false;
uint16_t dataTimer = 0;

uint16_t arrayIndex = 0;


//-------Arrays------------//
uint8_t months[ARRAY_SIZE];
uint8_t days[ARRAY_SIZE];
uint8_t hours[ARRAY_SIZE];
uint8_t minutes[ARRAY_SIZE];
uint8_t seconds[ARRAY_SIZE];
uint16_t milliseconds[ARRAY_SIZE];
uint16_t dists[ARRAY_SIZE];
float temps[ARRAY_SIZE];
float hums[ARRAY_SIZE];
//-------------------------//

//Structure example to receive data
//Must match the sender structure
typedef struct receiveData
{
  uint8_t sendMonth;
  uint8_t sendDay;
  uint8_t sendHour;
  uint8_t sendMinute;
  uint8_t sendSecond;
  uint16_t sendMillis;
  uint16_t sendDist;
  float sendTemp;
  float sendHum;
  int gaugeID;
} receiveData;

//Create a struct_message called myData
receiveData myData;
  
//callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
  memcpy(&myData, incomingData, sizeof(myData));

  // If the gauge ID matches the one we want, save the data
  if (myData.gaugeID == TARGET_GAUGE)
  {
    // Update data incoming
    dataIncoming = true;

    // Append data to arrays
    months[arrayIndex] = myData.sendMonth;
    days[arrayIndex] = myData.sendDay;
    hours[arrayIndex] = myData.sendHour;
    minutes[arrayIndex] = myData.sendMinute;
    seconds[arrayIndex] = myData.sendSecond;
    milliseconds[arrayIndex] = myData.sendMillis;
    dists[arrayIndex] = myData.sendDist;
    temps[arrayIndex] = myData.sendTemp;
    hums[arrayIndex] = myData.sendHum;

    // Print data
    printData();

    // Increment index of arrays
    arrayIndex++;
    if (arrayIndex > ARRAY_SIZE) arrayIndex = 0;

    // Update data timer
    dataTimer = millis();
  }
  
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

  dataTimer = millis();
  dataIncoming = true;
}
 
void loop()
{
  if ((dataIncoming) and (millis()-dataTimer > STALE_TIMER))
  {
    dataIncoming = false;

    sdWrite();
    clearArrays();
  }
}

//-------------------------------------------------------------

void clearArrays()
{
  for (int i = 0; i < ARRAY_SIZE; i++)
  {
    months[i] = 0;
    days[i] = 0;
    hours[i] = 0;
    minutes[i] = 0;
    seconds[i] = 0;
    milliseconds[i] = 0;
    dists[i] = 0;
    temps[i] = 0;
    hums[i] = 0;
  }

  arrayIndex = 0;
}

void sdWrite()
{
  Serial.println("\n\nWriting data to SD card\n\n");

  for (int i = 0; i < arrayIndex; i++)
  {
    Serial.print(getTime(months[i], days[i], hours[i], minutes[i], seconds[i]));
    Serial.printf(".%03d, ", milliseconds[i]);
    Serial.printf("%d, %.3f, %.3f\n", dists[i], temps[i], hums[i]);
  }
}

void printData()
{
  int i = arrayIndex;
  
  Serial.print(getTime(months[i], days[i], hours[i], minutes[i], seconds[i]));
  Serial.printf(".%03d, ", milliseconds[i]);
  Serial.printf("%d, %.3f, %.3f\n", dists[i], temps[i], hums[i]);
}

uint16_t getTime(uint8_t nowMonth, uint8_t nowDay, uint8_t nowHour, uint8_t nowMinute, uint8_t nowSecond)
{
  UnixTime stamp(0);
  stamp.setDateTime(YEAR, nowMonth, nowDay, nowHour, nowMinute, nowSecond);
  uint16_t timeStamp = stamp.getUnix();
  return timeStamp;
}
