//Winter 2022 Work
//sensorSoftware_v6
//Works on new prototypes which use this board:
//https://www.amazon.com/dp/B0718T232Z/ref=sspa_dk_detail_0?psc=1&pd_rd_i=B0718T232Zp13NParams&spLa=ZW5jcnlwdGVkUXVhbGlmaWVyPUEzNVdES1U5SDI2MDYyJmVuY3J5cHRlZElkPUEwNzY3NzI0MVVPNjQzOFBCMDFXMyZlbmNyeXB0ZWRBZElkPUEwMjA4NDMxMk8zUVNWVlRSVEExTiZ3aWRnZXROYW1lPXNwX2RldGFpbDImYWN0aW9uPWNsaWNrUmVkaXJlY3QmZG9Ob3RMb2dDbGljaz10cnVl

/* ---------------------
   For SD card:
   MOSI (DI) - 23
   MISO (DO) - 19
   CLK - 18
   CS - 5
   ---------------------
   For GPS RTC:
   RX - TX2 (17)
   TX - RX2 (16)
   EN - 27
   Vin - V+
   GND - GND
   ---------------------
   For Maxbotix
   TX (5) - RX1 (14)
   Start/Stop (4) - 33
   10kOhm - 33 - GND
   10kOhm - Temp (1) - GND
   ---------------------
   For Temp/Humidity:
   Black - GND
   Red - 15
   Yellow - 22
   Green - 21
   ---------------------
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "Adafruit_SHT31.h"
#include "Adafruit_GPS.h"
#include "unixTime.h"



#define READ_TIME 5*60 //Length of time to measure (in seconds)
#define READ_INTERVAL 15*60 //Measurement scheme (in seconds)
#define EFF_HZ 5.64 //MB 7388 (10 meter sensor)
//#define EFF_HZ 6.766 //MB 7388 (5 meter sensor)
#define LIST_SIZE (uint32_t)(EFF_HZ*READ_TIME)
#define secs_to_microsecs(__seconds) (__seconds * 1000000)
#define celsius_to_fahrenheit(__celsius) (__celsius * 9.0 / 5.0 + 32.0)
#define GMT_to_PST(__GMT) ((__GMT + 16) % 24)

#define FORMAT_BUF_SIZE 100

#define SD_CS GPIO_NUM_5 //SD card chip select pin

#define SONAR_RX GPIO_NUM_14 // Sonar sensor receive pin
#define SONAR_TX GPIO_NUM_32 // Sonar sensor transmit pin
#define SONAR_EN GPIO_NUM_33 //Sonar measurements are disabled when this pin is pulled low

#define LED_BUILTIN GPIO_NUM_2

#define GPS_MIN_TIME 100
#define GPS_RX GPIO_NUM_16
#define GPS_TX GPIO_NUM_17
#define GPS_CLOCK_EN GPIO_NUM_27
#define GPS_ECHO false

#define TEMP_SENSOR_ADDRESS 0x44
#define TEMP_EN GPIO_NUM_15

#define LED_PIN 2

uint32_t internal_millis_start;

//For sleep
RTC_DATA_ATTR int wakeCounter = -1;
int SLEEP_TIME;

//Clock variables
unsigned int myMillis;
String lastTime;
UnixTime stamp(3);
Adafruit_GPS GPS(&Serial2);

//For temp measurements
Adafruit_SHT31 tempSensor = Adafruit_SHT31();

struct sensorData {
    int *readList;
    String *timeList;
    float *tempExtList;
    float *humExtList;
    float myLong;
    float myLat;
    float myAlt;
};

//-----------------------------------------------------------------------------------
void setup()
{
    //Setup for SD card
    pinMode(SD_CS, OUTPUT);
    SD.begin(SD_CS);
    updateLog("Waking Up");
    updateLog("SD enabled");

    //9600 bps for Maxbotix
    Serial.begin(9600); //Serial monitor
    Serial1.begin(9600, SERIAL_8N1, SONAR_RX, SONAR_TX); //Maxbotix
    Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX); //Clock
    updateLog("Serial Ports Enabled");

    //Run Setup, check SD file every 1000th wake cycle
    wakeCounter += 1;
    Serial.print("Wake Counter: ");
    Serial.println(wakeCounter);
    if ((wakeCounter % 1000) == 0)
    {
    Serial.println("GPS Begin");
    wakeCounter = 0;

    startGPS();
    updateLog("GPS enabled");
    sdBegin();
    }

    //Turn on relevant pins
    gpio_hold_dis(GPIO_NUM_15);
    gpio_hold_dis(GPIO_NUM_33);
    gpio_hold_dis(GPIO_NUM_27);
    pinMode(GPS_CLOCK_EN, OUTPUT);
    pinMode(TEMP_EN, OUTPUT);
    pinMode(SONAR_EN, OUTPUT);
    digitalWrite(GPS_CLOCK_EN, HIGH); //Hold clock high
    digitalWrite(TEMP_EN, HIGH); //Hold high to supply power to temp sensor
    digitalWrite(SONAR_EN, LOW); //Hold low to prevent measurements
    updateLog("Start/Stop Enabled");

    //Setup for temp/humidity
    tempSensor.begin(TEMP_SENSOR_ADDRESS); //Hex Address for new I2C pins
    updateLog("Temp Sensor Enabled");

    //Setup for LED
    pinMode(LED_PIN, OUTPUT);
    updateLog("LEDs enabled");

    //Check for SD header file
    updateLog("Startup concluded");

    internal_millis_start = millis();

    Serial.println();
    Serial.print("Starting: ");
    Serial.println(displayTime());
}

void loop()
{
    uint32_t meas_start_sec = getTime();
    uint32_t meas_start_milli = GPS.milliseconds;
    uint32_t internal_start = millis();
    int readList[LIST_SIZE];
    String timeList[LIST_SIZE];
    float tempExtList[LIST_SIZE];
    float humExtList[LIST_SIZE];
    sensorData data = {
        readList,
        timeList,
        tempExtList,
        humExtList,
        GPS.longitude,
        GPS.latitude,
        GPS.altitude,
    };

    //Clear LED
    digitalWrite(LED_PIN, LOW);

    //Fill the reading, time, and temp arrays
    Serial.print("Filling Arrays: ");
    Serial.println(displayTime());

    //Get location data

    //Measure
    updateLog("Filling arrays");
    fillArrays(readList, timeList, tempExtList, humExtList); //Tuns on LED if taking good measurements

    //Write to SD card and serial monitor
    updateLog("Done filling arrays");
    updateLog("Writing to SD card");
    sdWrite(readList, timeList, tempExtList, humExtList, myLong, myLat, myAlt);

    //Prepare deep sleep
    digitalWrite(LED_PIN, LOW);
    updateSleep();
    esp_sleep_enable_timer_wakeup(secs_to_microsecs(SLEEP_TIME));

    //Print sleep time info
    Serial.print("Sleep: ");
    Serial.println(displayTime());
    String message = "Sleep for: ";
    message += String(SLEEP_TIME);
    message += " seconds";
    Serial.println(message);
    updateLog(message);

    //Turn everything off
    digitalWrite(GPS_CLOCK_EN, LOW);
    digitalWrite(TEMP_EN, LOW);
    digitalWrite(SONAR_EN, LOW);
    gpio_hold_en(GPS_CLOCK_EN); //Make sure clock is off
    gpio_hold_en(TEMP_EN); //Make sure temp sensor is off
    gpio_hold_en(SONAR_EN); //Make sure Maxbotix is off
    esp_sleep_enable_ext0_wakeup(SONAR_EN, 1);
    gpio_deep_sleep_hold_en();

    //Go to sleep
    Serial.flush();
    esp_deep_sleep_start();
    //Sleeps until woken, runs setup() again
}

//-----------------------------------------------------------------------------------
// Start GPS Clock
void startGPS()
{
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  updateLog("GPS Serial Enabled");

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  updateLog("Minimum Recommended Enabled");

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  updateLog("GPS Frequency Enabled");
}

// Get current unix time
uint32_t getTime()
{
  //Start timer
  uint32_t timer = millis();

  while (millis() - timer <= GPS_MIN_TIME)
  {
    // Read GPS
    char c = GPS.read();


    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived())
    {
      // a tricky thing here is if we print the NMEA sentence, or data
      // we end up not listening and catching other sentences!
      // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
      //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

      if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
        break;  // we can fail to parse a sentence in which case we should just wait for another
    }
  }

  stamp.setDateTime(GPS.year + 2000, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds);
  uint32_t unix = stamp.getUnix();

  unix += 10800; // 3 hrs

  return unix;
}

String displayTime() {
  char buf[FORMAT_BUF_SIZE];
  getTime();
  sprintf(buf, "%d:%d:%d.%03d", GMT_to_PST(GPS.hour), GPS.minute, GPS.seconds, GPS.milliseconds);
  return String(buf);
}

String unixTime() {
  char buf[FORMAT_BUF_SIZE];
  sprintf(buf, "%d.%03d", getTime(), GPS.milliseconds);
  return String(buf);
}

//Get a reading from the sonar
int sonarMeasure()
{
    char inData[5] = {0}; //char array to read data into
    long start = millis(); //timeout if read takes too long
    // Clear cache ready for next reading
    Serial1.flush();

    //Wait for device to be ready
    while (!Serial1.available()){if(millis() - start > 1000) return -1;}

    //Maxbotix reports "Rxxxx", where xxxx is a 4 digit mm distance
    while (Serial1.read() != 'R'){}
    Serial1.readBytes(inData, 4);

    uint result = atoi(inData);

    //Turn on LED if measuring properly
    //Maxbotix reports 300 if object is too close
    //or you'll get 0 if wired improperly
    if ((result > 500) and (result < 9999)) {
        digitalWrite(LED_BUILTIN, HIGH);
    }
    else digitalWrite(LED_BUILTIN, LOW);

    return result;
  }

//Fill time, temp, and measurement arrays
void fillArrays(sensorData *data)
{
  //Turn on Maxbotix
  digitalWrite(SONAR_EN, HIGH);

  //Fill measurement and timestamp lists
  for (int i = 0; i < LIST_SIZE; i++)
  {
    data->readList[i] = sonarMeasure();
    data->timeList[i] = unixTime();
    data->tempExtList[i] =  celsius_to_fahrenheit(tempSensor.readTemperature());
    data->humExtList[i] = tempSensor.readHumidity();

    //Print timestamps and measurement as they are taken
    Serial.printf("%d  %d  %d  %d  %d  %d  %d\n",
        data->timeList[i],
        data->readList[i],
        data->tempExtList[i],
        data->humExtList[i],
        data->myLat,
        data->myLong,
        data->myAlt);
  }

  //Turn off Maxbotix
  digitalWrite(SONAR_EN, LOW);
}

//Write the list to the sd card
void sdWrite(sensorData *data)
{
  long start = millis();

  //Create string for new file name
  String fileName = "/Data/";
  fileName += String(getTime(), HEX);
  fileName += ".txt";

  //Create and open a file
  File dataFile = SD.open(fileName, FILE_WRITE);
  if(!dataFile) return;
  Serial.printf("Writing %s: ", fileName.c_str());

  dataFile.printf("%d, %d, %d", data->myLong, data->myLat, data->myAlt);

  //Iterate over entire list
  for (int i = 0; i < LIST_SIZE; i++)
  {
    //Break out if it's taking too long
    if ((millis() - start) > 10000)
    {
      dataFile.close();
      Serial.println("Taking too long");
      return;
    }

    dataFile.printf("%d, %d, %d, %d\n",
        data->timeList[i],
        data->readList[i],
        data->tempExtList[i],
        data->humExtList[i]);
  }

  //Close file
  dataFile.close();
}

//Check or create header file
//TODO use printf to simplify
void sdBegin()
{
  //Check if file exists and create one if not
  if (!SD.exists("/README.txt"))
  {
    Serial.println("Creating lead file");

    File dataFile = SD.open("/README.txt", FILE_WRITE);

    //Create header with title, timestamp, and column names
    dataFile.println("Cal Poly Tide Sensor");
    dataFile.print("Starting: ");
    dataFile.println(unixTime());
    dataFile.println();
    dataFile.println("UNIX Time, Distance (mm), Internal Temp (F), External Temp (F), Humidity (%), Latitude, Longitude, Altitude");
    dataFile.close();

    SD.mkdir("/Data");
  }
}

//Update Sleep Time
void updateSleep()
{
  //Update the current minute (0-59) and convert to seconds
  int nowTime = 60*GPS.minute + GPS.seconds;


  //Calculate sleep time based on interval
  SLEEP_TIME = READ_INTERVAL - (nowTime % READ_INTERVAL);

  //Offset for half the reading time
  SLEEP_TIME -= (READ_TIME / 2);

  //If something went wrong and the sleep time is too high, sleep for the interval
  if (SLEEP_TIME > READ_INTERVAL)
  {
    SLEEP_TIME = READ_INTERVAL;
  }

  if (SLEEP_TIME < 0)
  {
    //Actually only sleep for 1 second
    SLEEP_TIME = 1;
  }
}

void updateLog(String message)
{
  //Create message
  String myMessage = String(unixTime());
  myMessage += ": ";
  myMessage += message;

  //Open log file and write to it
  File logFile = SD.open("/logFile.txt", FILE_WRITE);
  if(!logFile) return;
  //logFile.seek(logFile.size());
  logFile.println(myMessage);
  logFile.close();
}
