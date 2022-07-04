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
#include <driver\timer.h>
#include <soc\rtc.h>
#include <soc\soc.h>
#include <hal\wdt_hal.h>
#include <hal\wdt_types.h>
#include "Adafruit_SHT31.h"
#include "Adafruit_GPS.h"
#include "UnixTime.h"


//! Changed for debugging
#define READ_TIME 5 //Length of time to measure (in seconds)
#define READ_INTERVAL 15 //Measurement scheme (in seconds)
#define MEASUREMENT_HZ 5.64 //MB 7388 (10 meter sensor)

#define UNIX_TIME_ZONE 8

//#define EFF_HZ 6.766 //MB 7388 (5 meter sensor)
#define LIST_SIZE (uint32_t)(MEASUREMENT_HZ*READ_TIME)
#define secs_to_microsecs(__seconds) ((__seconds) * 1000000)
#define celsius_to_fahrenheit(__celsius) ((__celsius) * 9.0 / 5.0 + 32.0)
#define GMT_to_PST(__GMT) (((__GMT) + 16) % 24)
#define FORMAT_BUF_SIZE 100

#define SD_CS GPIO_NUM_5 //SD card chip select pin

#define SONAR_RX GPIO_NUM_14 // Sonar sensor receive pin
#define SONAR_TX GPIO_NUM_32 // Sonar sensor transmit pin
#define SONAR_EN GPIO_NUM_33 //Sonar measurements are disabled when this pin is pulled low
#define SONAR_MEASURE_TIMEOUT 100 //The number of milliseconds to attempt to get a sonar measure before timing out

#define GPS_MIN_TIME 100
#define GPS_RX GPIO_NUM_16
#define GPS_TX GPIO_NUM_17
#define GPS_CLOCK_EN GPIO_NUM_27
#define GPS_ECHO false
#define GPS_POLLING_HZ 100

#define TEMP_SENSOR_ADDRESS 0x44
#define TEMP_EN GPIO_NUM_15

// The RTC slow timer is driven by the RTC slow clock, typically 150kHz
uint64_t clock_start; //the rtc clock cycle count that we begin measuring at
const uint32_t rtc_slow_clk_hz = rtc_clk_slow_freq_get_hz();
const uint32_t rtc_abp_clk_hz = rtc_clk_apb_freq_get();
uint32_t gps_millis_offset = millis();

// Data which should be preserved between sleep/wake cycles
RTC_DATA_ATTR uint32_t wakeCounter = 0;

// The format_buffer is overwritten by displayTime and unixTime
char format_buf[FORMAT_BUF_SIZE];

// GPS ISR and Serial1 callback variables
uint32_t num_gps_reads = 0;
volatile bool measurement_request = false;

//Objects to manage peripherals
Adafruit_GPS GPS(&Serial2);
Adafruit_SHT31 tempSensor = Adafruit_SHT31();

struct sensorData {
    UnixTime time;
    float tempExt;
    float humExt;
    int dist;
};

// every 10ms read from the GPS, when
bool gps_polling_isr(void* arg) {
    num_gps_reads += 1;

    timer_group_clr_intr_status_in_isr(timer_group_t::TIMER_GROUP_0, timer_idx_t::TIMER_0);
    return false;
}

void sonarDataReady(void) {
    measurement_request = true;
}

//-----------------------------------------------------------------------------------
void setup(void) {

    // Clock cycle count when we begin measuring
    clock_start = rtc_time_get();

    //Setup for SD card
    pinMode(SD_CS, OUTPUT);
    SD.begin(SD_CS);
    writeLog("Waking Up");
    writeLog("SD enabled");

    Serial.begin(115200); //Serial monitor

    //9600 bps for Maxbotix
    Serial1.begin(9600, SERIAL_8N1, SONAR_RX, SONAR_TX); //Maxbotix
    Serial1.onReceive(sonarDataReady, true); // register callback
    Serial1.setRxTimeout(10);

    // Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX); //Clock
    writeLog("Serial Ports Enabled");

    //Run Setup, check SD file every 1000th wake cycle
    Serial.print("Wake Counter: ");
    Serial.println(wakeCounter);
    if ((wakeCounter % 1000) == 0) {
        wakeCounter = 0;
        sdBegin();
        startGPS();
        //TODO get measurements to allign with 15 min intervals
        //TODO wait for GPS to get fix?
    }
    wakeCounter += 1;

    // configure timer to manage GPS polling
    timer_config_t timer_polling_config;
    timer_polling_config.alarm_en = timer_alarm_t::TIMER_ALARM_EN;
    timer_polling_config.auto_reload = timer_autoreload_t::TIMER_AUTORELOAD_EN;
    timer_polling_config.counter_dir = timer_count_dir_t::TIMER_COUNT_UP;
    timer_polling_config.divider = 2; //should be in [2, 65536]
    timer_init(timer_group_t::TIMER_GROUP_0, timer_idx_t::TIMER_0, &timer_polling_config);
    timer_set_alarm_value(timer_group_t::TIMER_GROUP_0, timer_idx_t::TIMER_0, rtc_abp_clk_hz / (2 * GPS_POLLING_HZ)); // configure timer to count 10 millis
    timer_isr_callback_add(timer_group_t::TIMER_GROUP_0, timer_idx_t::TIMER_0, gps_polling_isr, nullptr, ESP_INTR_FLAG_LOWMED);
    timer_enable_intr(timer_group_t::TIMER_GROUP_0, timer_idx_t::TIMER_0);
    timer_start(timer_group_t::TIMER_GROUP_0, timer_idx_t::TIMER_0);

    //Turn on relevant pins
    gpio_hold_dis(GPIO_NUM_15);
    gpio_hold_dis(GPIO_NUM_33);
    gpio_hold_dis(GPIO_NUM_27);
    pinMode(GPS_CLOCK_EN, OUTPUT);
    pinMode(TEMP_EN, OUTPUT);
    pinMode(SONAR_EN, OUTPUT);
    digitalWrite(GPS_CLOCK_EN, HIGH); //Hold clock high
    digitalWrite(TEMP_EN, HIGH); //Hold high to supply power to temp sensor
    digitalWrite(SONAR_EN, HIGH); //Hold high to begin measuring with sonar
    writeLog("Start/Stop Enabled");

    //Setup for temp/humidity
    tempSensor.begin(TEMP_SENSOR_ADDRESS); //Hex Address for new I2C pins
    writeLog("Temp Sensor Enabled");

    //Setup for LED
    pinMode(LED_BUILTIN, OUTPUT);
    writeLog("LEDs enabled");

    writeLog("Startup concluded");

    Serial.println();
    Serial.print("Starting: ");
    Serial.println(displayTime(getTime()));
}

void loop(void) {
    //TODO way to prevent reallocation of memory every sleep/wake cycle
    //Everything should be overwritten before it is read, so there is no need to initialize the data
    static sensorData* data = (sensorData*)(operator new[](sizeof(sensorData) * LIST_SIZE, std::nothrow));
    //C++ exceptions are disabled by default, use std::nothrow and assert non-null
    //failed assertions cause a Fatal error
    assert(data != nullptr);
    static uint32_t idx = 0;

    for(int i=0; i < num_gps_reads; i++){
        GPS.read(); //if GPS.read() takes longer than the GPS polling frequency, execution may get stuck in this loop
        if(GPS.newNMEAreceived()) {
            // a tricky thing here is if we print the NMEA sentence, or data
            // we end up not listening and catching other sentences!
            // so be very wary if using OUTPUT_ALLDATA and trying to print out data
            //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
            GPS.parse(GPS.lastNMEA());  // this also sets the newNMEAreceived() flag to false
            gps_millis_offset = millis() - GPS.milliseconds;
        }
    }
    num_gps_reads = 0;
    if(measurement_request && (idx < LIST_SIZE)) {
        measurement_request = false;
        readData(data, idx);
        idx++;
    }
    else if(idx >= LIST_SIZE){
        //Write to SD card and serial monitor
        writeLog("Writing to SD card");
        sdWrite(data);

        //Free allocated data
        //This may not be neccesary because the chip SRAM resets after waking from deep sleep
        delete[] data;

        goto_sleep();
    }
}

//-----------------------------------------------------------------------------------
// Start GPS Clock
void startGPS(void)
{
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  writeLog("GPS Serial Enabled");

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  writeLog("Minimum Recommended Enabled");

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  writeLog("GPS Frequency Enabled");
}

// Return a current time_stamp
UnixTime getTime(void)
{
    UnixTime stamp(UNIX_TIME_ZONE);
    stamp.setDateTime(GPS.year + 2000, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds);
    // unix += 10800; // 3 hrs
    return stamp;
}

//TODO the current way unixTime and displayTime is probably inefficiant because of nested printf
char* displayTime(UnixTime now) {
    snprintf(format_buf, FORMAT_BUF_SIZE, "%02d:%02d:%02d.%03d", now.hour, now.minute, now.second, (millis() - gps_millis_offset)%1000);
    return format_buf;
}

char* unixTime(UnixTime now) {
    snprintf(format_buf, FORMAT_BUF_SIZE, "%d.%03d", now.getUnix(), (millis() - gps_millis_offset)%1000);
    return format_buf;
}

//Get a reading from the sonar
//negative readings imply an error
int32_t sonarMeasure() {
    char inData[5] = {0}; //char array to read data into

    //Maxbotix reports "Rxxxx\L", where xxxx is a 4 digit mm distance, '\L' is carriage return
    //a measurement is 6 bytes
    if(Serial1.available() <= 5) return -1;
    //Measurements begin with 'R'
    while(Serial1.peek() != 'R') {
        Serial1.read();
        if(Serial1.available() <= 5) return -2;
    }
    Serial1.read(); //discard R

    Serial1.readBytes(inData, 4);
    if(Serial1.read() != 13) return -3; //check and discard carriage return

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
//Should be called whenever sonar sensor has data ready
void readData(sensorData *data, uint32_t idx)
{
    data[idx].dist = sonarMeasure();
    data[idx].time = getTime();
    data[idx].tempExt = celsius_to_fahrenheit(tempSensor.readTemperature());
    data[idx].humExt = tempSensor.readHumidity();

    Serial.printf("%s %d  %f  %f  %f  %f  %f\n",
    unixTime(data[idx].time),
    data[idx].dist,
    celsius_to_fahrenheit(data[idx].tempExt),
    data[idx].humExt,
    GPS.latitude,
    GPS.longitude,
    GPS.altitude);
}

//Write the list to the sd card
void sdWrite(sensorData *data)
{
  long start = millis();

  //Create string for new file name
  String fileName = "/Data/";
  if(GPS.fix) snprintf(format_buf, FORMAT_BUF_SIZE, "%x.txt", getTime().getUnix());
  else snprintf(format_buf, FORMAT_BUF_SIZE, "f_%x.txt", millis());

  //Create and open a file
  File dataFile = SD.open(format_buf, FILE_WRITE);
  if(!dataFile) return;
  Serial.printf("Writing %s: ", format_buf);

  dataFile.printf("%f, %f, %f", GPS.longitude, GPS.latitude, GPS.altitude);

  //Iterate over entire list
  for (int i = 0; i < LIST_SIZE; i++)
  {
    //Break out if it's taking too long
    if ((millis() - start) > 10000)
    {
      dataFile.close();
      Serial.println("Taking too long");
      writeLog("data write timed out");
      return;
    }

    dataFile.printf("%s, %d, %f, %f\n",
        unixTime(data[i].time),
        data[i].dist,
        data[i].tempExt,
        data[i].humExt);
  }

  //Close file
  dataFile.close();
}

//Check or create header file
//TODO use printf to simplify
void sdBegin(void)
{
    //Check if file exists and create one if not
    if (!SD.exists("/README.txt"))
    {
        Serial.println("Creating README");
        File dataFile = SD.open("/README.txt", FILE_WRITE);

        //Create header with title, timestamp, and column names
        dataFile.println("");
        dataFile.printf(
            "Cal Poly Tide Sensor\n"
            "Starting: %d\n\n"
            "Data File format:\n"
            "Line   1: Latitude, Longitude, Altitude\n"
            "Line 'n': UNIX Time, Distance (mm), External Temp (F), Humidity (%)\n",
            unixTime(getTime()));
        dataFile.close();

        SD.mkdir("/Data");
    }
}

//Powers down all peripherals, Sleeps until woken after set interval, runs setup() again
void goto_sleep(void) {
    //Print sleep time info
    //TODO it seems that printf with strings and numbers fails, is this because the stack grows too large?
    //* Why does the printf not work with both the str and the number?
    Serial.printf("Going to sleep at %s\n", displayTime(getTime()));
    Serial.printf("Sleeping for %lu secs\n", READ_INTERVAL - (rtc_time_get() - clock_start) / (uint64_t)rtc_slow_clk_hz);

    //TODO log more informative message
    writeLog(String("sleeping"));

    //Turn everything off
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(GPS_CLOCK_EN, LOW);
    digitalWrite(TEMP_EN, LOW);
    digitalWrite(SONAR_EN, LOW);
    gpio_hold_en(GPS_CLOCK_EN); //Make sure clock is off
    gpio_hold_en(TEMP_EN); //Make sure temp sensor is off
    gpio_hold_en(SONAR_EN); //Make sure Maxbotix is off
    gpio_deep_sleep_hold_en();

    //Prepare and go into sleep
    Serial.flush();
    esp_sleep_enable_timer_wakeup(1000000 * READ_INTERVAL - 1000000 * (rtc_time_get() - clock_start) / rtc_slow_clk_hz);
    esp_deep_sleep_start();
}

void writeLog(String message)
{
    //Open log file and write to it
    File logFile = SD.open("/logFile.txt", FILE_WRITE);
    if(!logFile) return;
    //logFile.seek(logFile.size());
    logFile.printf("%s: %s\n", unixTime(getTime()), message);
    logFile.close();
}
