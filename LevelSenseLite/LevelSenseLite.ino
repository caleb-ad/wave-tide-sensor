/*
 *Summer 2021 Internship
 *LevelSensorLite_v1
 */

/* ---------------------
   For SD card:
   MOSI (DI) - 23
   MISO (DO) - 19
   CLK - 18
   SD_CS - 5
   ---------------------
   For RTC:
   SDA - 21
   SCL - 22
   5.01kOhm - 21 - Vcc
   5.01kOhm - 22 - Vcc
   ---------------------
   For Maxbotix
   Tx (5) - Rx2 (16)
   Start/Stop (4) - 33
   10kOhm - 33 - GND
   ---------------------
   For button:
   Blue (Button) - 13
   10kOhm - 13 - GND
   Red (Light) - 25
   Black - GND
   Green - Vcc
   ---------------------
*/
//C/C++ standard library
#include <string.h>
// Arduino/ESP32 header
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <DS3232RTC.h>

/*
 *MEASUREMENT CONSTANTS
 */
#define READ_TIME 5*60 //Length of time to measure (in seconds)
#define READ_INTERVAL 15*60 //Measurement scheme (in seconds)
#define EFF_HZ 5.64 //MB 7388 (10 meter sensor)
//#define EFF_HZ 6.766 //MB 7388 (5 meter sensor)
#define LIST_SIZE (uint32_t)(EFF_HZ*READ_TIME)
#define secs_to_microsecs(__seconds) (__seconds * 1000000)

/*
 *PIN DEFINES
 */
#define BUTTON_PIN GPIO_NUM_13 //Button pin
#define SD_CS GPIO_NUM_5 //SD card chip select pin
#define SONAR_RX2 GPIO_NUM_16 // Sonar sensor receive pin
#define SONAR_TX2 GPIO_NUM_17 // Sonar sensor transmit pin
#define SONAR_ENABLE_PIN GPIO_NUM_33 //Sonar measurements are disabled when this pin is pulled low

//const int LED = 25; //LED pin
#define LED_BUILTIN GPIO_NUM_25 //i don't think this exists on this board

#define FORMAT_BUF_SIZE 100

const u_long startMillis = millis();

//Clock variables
DS3232RTC myClock(false); //For non AVR boards (ESP32)

// represents the current time formatted for display and the unix time
// display: hrs:min:sec.millis
// unix: unix_sec.millis
struct FormattedTimes {
  String unix;
  String display;

  FormattedTimes(time_t current_t)
  {
    char buf[FORMAT_BUF_SIZE];
    unsigned long current_millis = (millis() - startMillis) % 1000;

    sprintf(buf, "%d.%03d", current_t, current_millis);
    this->unix = String(buf);

    sprintf(buf, "%d:%d:%d.%03d", hour(current_t), minute(current_t), second(current_t), current_millis);
    this->display = String(buf);
  }
};

//-----------------------------------------------------------------------------------
//Get a reading from the sonar
int sonarMeasure()
{
  int result;
  char inData[4]; //char array to read data into
  int index = 0;
  boolean stringComplete = false;

  // Clear cache ready for next reading
  Serial2.flush();

  //Try until it gets an actual reading
  while (stringComplete == false)
  {
    //If sensor has data available
    if (Serial2.available())
    {
      //Read serial input
      char rByte = Serial2.read();

      //If the first character is an "R"
      if (rByte == 'R') //Maxbotix reports "Rxxxx", where xxxx is a 4 digit mm distance
      {
        //Read next four characters (range)
        while (index < 4)
        {
          //If there is a number, place it in array
          if (Serial2.available())
          {
            inData[index] = Serial2.read();
            index++;
          }
        }

        //add a padding byte at end for atoi() function
        inData[index] = 0x00;
      }

      //"R" is not the first character
      else
      {
        //Clear the serial channel for next reading
        Serial2.flush();
      }

      //Reset before next reading
      rByte = 0;
      index = 0;
      stringComplete = true;

      //Changes string data into an integer for use
      result = atoi(inData);

      //Turn on LED if measuring properly
      //Maxbotix reports 300 if object is too close
      //or you'll get 0 if wired improperly
      if ((result > 500) and (result < 9999))
      {
        digitalWrite(LED_BUILTIN, HIGH);
      }
      else digitalWrite(LED_BUILTIN, LOW);

    }
  }

  //Reset for next reading
  stringComplete = false;

  return result;
}

//Fill time, temp, and measurement arrays
void fillArrays(String *times, int *distances, double *temps)
{
  //Turn on Maxbotix
  digitalWrite(SONAR_ENABLE_PIN, HIGH);

  //Fill measurement and timestamp lists
  for (int i = 0; i < LIST_SIZE; i++)
  {
    distances[i] = sonarMeasure();
    times[i] = FormattedTimes(now()).unix;
    temps[i] = myClock.temperature() * 9.0 / 20.0 + 32.0;

    //Print timestamps and measurement as they are taken
    Serial.print(times[i]);
    Serial.print(" ");
    Serial.print(distances[i]);
    Serial.print(" ");
    Serial.println(temps[i]);
  }

  //Turn off Maxbotix
  digitalWrite(SONAR_ENABLE_PIN, LOW);
}

//Write the list to the sd card
void sdWrite(String *times, int *distances, double *temps)
{
  //Create string for new file name
  String fileName = "/Data/";
  fileName += String(now(), HEX);
  fileName += ".txt";

  //Create and open a file
  File dataFile = SD.open(fileName, FILE_WRITE);

  Serial.print("Writing ");
  Serial.print(fileName);
  Serial.print(": ");
  Serial.println(FormattedTimes(now()).display);

  //Iterate over entire list
  for (int i = 0; i < LIST_SIZE; i++)
  {
    //Create strings for measurement, time, and temp
    String measurement = String(distances[i]);
    String currentTime = String(times[i]);
    String currentTemp = String(temps[i]);

    //Write time, measurement, and temp on one line in file
    dataFile.print(currentTime);
    dataFile.print(",");
    dataFile.print(measurement);
    dataFile.print(",");
    dataFile.println(currentTemp);

  }

  //Close file
  dataFile.close();
}

//Check or create header file
void sdBegin()
{
  //Check if file exists and create one if not
  if (!SD.exists("/begin.txt"))
  {
    Serial.println("Creating lead file");

    File dataFile = SD.open("/begin.txt", FILE_WRITE);

    SD.mkdir("/Data");

    //Create header with title, timestamp, and column names
    dataFile.println("Cal Poly Tide Sensor");
    dataFile.print("Starting: ");
    dataFile.println(FormattedTimes(now()).display);
    dataFile.println();
    dataFile.println("UNIX Time, Distance (mm), Temp (F)");
    dataFile.close();
  }
}

//returns the time to sleep in seconds
long get_sleep_time()
{
  //Update the current minute (0-59) and convert to seconds
  int nowTime = 60*minute(now()) + second(now());

  //Calculate sleep time based on interval and offset for half the reading time
  return READ_INTERVAL - (nowTime % READ_INTERVAL) - (READ_TIME / 2);
}

//-----------------------------------------------------------------------------------
void setup()
{
  //9600 bps for Maxbotix
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, SONAR_RX2, SONAR_TX2);
  gpio_hold_dis(SONAR_ENABLE_PIN);
  pinMode(SONAR_ENABLE_PIN, OUTPUT);
  digitalWrite(SONAR_ENABLE_PIN, LOW); //Hold low to prevent measurements

  //Setup for button
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT); //Use external 10k pulldown resistor

  //Setup for SD card
  pinMode(SD_CS, OUTPUT);
  SD.begin(SD_CS);

  //Setup for clock
  myClock.begin(); //Initializes I2C bus for non AVR boards
  setSyncProvider(myClock.get); //Set the external RTC as the time keeper

  //Print start time info
  Serial.println();
  Serial.print("Starting: ");
  Serial.println(FormattedTimes(now()).display);

  //Check for SD header file
  sdBegin();

}

void loop()
{
  int distList[LIST_SIZE];
  String timeList[LIST_SIZE];
  double tempList[LIST_SIZE];
  // FormattedTimes FT;

  //Fill the reading, time, and temp arrays
  Serial.print("Filling Arrays: ");
  Serial.println(FormattedTimes(now()).display);
  fillArrays(timeList, distList, tempList); //Turns on LED if taking good measurements

  //Write to SD card and serial monitor
  sdWrite(timeList, distList, tempList);
  digitalWrite(LED_BUILTIN, LOW); //Turns off LED before going to sleep

  //Prepare deep sleep
  long sleep_time = get_sleep_time();
  esp_sleep_enable_timer_wakeup(secs_to_microsecs(sleep_time));
  esp_sleep_enable_ext0_wakeup(BUTTON_PIN, 1);
  gpio_hold_en(SONAR_ENABLE_PIN); //Make sure Maxbotix is off
  gpio_deep_sleep_hold_en();

  //Print sleep time info
  Serial.print("Sleep: ");
  Serial.println(FormattedTimes(now()).display);
  Serial.print("Sleep until: ");
  Serial.print(FormattedTimes(now() + sleep_time).display);

  //Go to sleep
  Serial.flush();
  esp_deep_sleep_start();
  //Sleeps until woken, runs setup() again
}
