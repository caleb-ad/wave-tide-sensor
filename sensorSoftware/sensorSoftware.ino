#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <driver\timer.h>
#include <soc\rtc.h>
#include "Adafruit_SHT31.h"
#include "Adafruit_GPS.h"
#include "UnixTime.h"

//! Changed for debugging
// values of 'READ_TIME' more than 10 minutes usually require too much memory
#define READ_TIME 2 * 60//Length of time to measure (in seconds)

// measurements will occurr on mulitples of this value after the hour
#define MINUTE_ALLIGN 5//minutes
#define MEASUREMENT_HZ 5.64 //MB 7388 (10 meter sensor)

//#define EFF_HZ 6.766 //MB 7388 (5 meter sensor)
#define LIST_SIZE (uint32_t)((MEASUREMENT_HZ)*(READ_TIME))
#define secs_to_microsecs(__seconds) ((__seconds) * 1000000)
#define celsius_to_fahrenheit(__celsius) ((__celsius) * 9.0 / 5.0 + 32.0)
#define GMT_to_PST(__GMT) (((__GMT) + 17) % 24)
#define FORMAT_BUF_SIZE 100

#define SD_CS GPIO_NUM_5 //SD card chip select pin

#define SONAR_RX GPIO_NUM_14 // Sonar sensor receive pin
#define SONAR_TX GPIO_NUM_32 // Sonar sensor transmit pin
#define SONAR_EN GPIO_NUM_33 //Sonar measurements are disabled when this pin is pulled low

#define GPS_RX GPIO_NUM_16
#define GPS_TX GPIO_NUM_17
#define GPS_CLOCK_EN GPIO_NUM_27
#define GPS_POLLING_HZ 100
#define GPS_DIFF_FROM_GMT 0

#define TEMP_SENSOR_ADDRESS 0x44
#define TEMP_EN GPIO_NUM_15

// The RTC slow timer is driven by the RTC slow clock, typically 150kHz
uint64_t clock_start; //the rtc clock cycle count that we begin measuring at
uint32_t gps_millis_offset = millis();
const uint32_t rtc_slow_clk_hz = rtc_clk_slow_freq_get_hz();
const uint32_t rtc_abp_clk_hz = rtc_clk_apb_freq_get();
const uint64_t half_read_time = 1000000 * READ_TIME / 2;

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

// every 10ms schedule a read from the GPS, when
bool gps_polling_isr(void* arg) {
    num_gps_reads += 1;
    timer_group_clr_intr_status_in_isr(timer_group_t::TIMER_GROUP_0, timer_idx_t::TIMER_0);
    return false;
}

void sonarDataReady(void) {
    measurement_request = true;
}

void setup(void) {
    // Assert that constants and defines are in valid state
    assert(READ_TIME < MINUTE_ALLIGN * 60);

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
    startGPS(GPS); //uses Serial2
    writeLog("Serial Ports Enabled");

    //Run Setup, check SD file every 1000th wake cycle
    Serial.print("Wake Counter: ");
    Serial.println(wakeCounter);
    if ((wakeCounter % 1000) == 0) {
        wakeCounter = 0;
        sdBegin();
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
    static uint32_t idx = 0;

    assert(data != nullptr);

    while(num_gps_reads > 0){
        GPS.read(); //if GPS.read() takes longer than the GPS polling frequency, execution may get stuck in this loop
        num_gps_reads -= 1;
        if(GPS.newNMEAreceived()) {
            // a tricky thing here is if we print the NMEA sentence, or data
            // we end up not listening and catching other sentences!
            // so be very wary if using OUTPUT_ALLDATA and trying to print out data
            //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
            GPS.parse(GPS.lastNMEA());  // this also sets the newNMEAreceived() flag to false
            gps_millis_offset = millis() - GPS.milliseconds;
        }
    }
    if(measurement_request && (idx < LIST_SIZE)) {
        measurement_request = false;
        readData(data[idx]);
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

// Start GPS Clock
void startGPS(Adafruit_GPS &gps)
{
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  gps.begin(9600);
  writeLog("GPS Serial Enabled");

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  writeLog("Minimum Recommended Enabled");

  // Set the update rate
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  writeLog("GPS Frequency Enabled");
}

// Return a current time_stamp
UnixTime getTime(void)
{
    UnixTime stamp(GPS_DIFF_FROM_GMT);
    stamp.setDateTime(2000 + GPS.year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds);
    return stamp;
}

//*Functions which use 'format_buf'*/

//human readable time, in PST
char* displayTime(UnixTime now) {
    snprintf(format_buf, FORMAT_BUF_SIZE, "%02d:%02d:%02d.%03d", GMT_to_PST(now.hour), now.minute, now.second, (millis() - gps_millis_offset)%1000);
    return format_buf;
}

char* unixTime(UnixTime now) {
    snprintf(format_buf, FORMAT_BUF_SIZE, "%d.%03d", now.getUnix(), (millis() - gps_millis_offset)%1000);
    return format_buf;
}
//*End: Functions which use 'format_buf'*/

// return signed latitude with the convention that north of the equator is positve
inline float latitude_signed(Adafruit_GPS &gps) {
    return (gps.lat == 'N') ? gps.latitude : -1.0 * gps.latitude;
}

//return signed longitude with the convention that east of the prime meridian is positive
inline float longitude_signed(Adafruit_GPS &gps) {
    return (gps.lon == 'E') ? gps.longitude : -1.0 * gps.longitude;
}

//Get a reading from the sonar
//negative readings imply an error
int32_t sonarMeasure(void) {
    char inData[5] = {0}; //char array to read data into

    //Maxbotix reports "Rxxxx\L", where xxxx is a 4 digit mm distance, '\L' is carriage return
    //a measurement is 6 bytes
    if(Serial1.available() <= 5) return -1;
    //Measurements begin with 'R'
    while(Serial1.read() != 'R') {
        if(Serial1.available() <= 5) return -2;
    }

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
//TODO GPS negative/postive hemisphere information
void readData(sensorData &data)
{
    data.dist = sonarMeasure();
    data.time = getTime();
    data.tempExt = celsius_to_fahrenheit(tempSensor.readTemperature());
    data.humExt = tempSensor.readHumidity();

    Serial.printf("%s %d  %f  %f  %f  %f  %f\n",
    unixTime(data.time),
    data.dist,
    data.tempExt,
    data.humExt,
    latitude_signed(GPS),
    longitude_signed(GPS),
    GPS.altitude);
}

//Write the list to the sd card
void sdWrite(sensorData *data)
{
  long start = millis();

  //Create string for new file name
  //if the gps has updated its time in the last read cycle use that time to name the file
  //filenames are at most 8 characters + 6("/Data/") + 4(".txt") + null terminator = 19
  if(GPS.fixquality >= 1) snprintf(format_buf, 19 , "/Data/%x.txt", getTime().getUnix());
  else snprintf(format_buf, 19, "/Data/%x_%x.txt", wakeCounter, millis());

  //Create and open a file
  File dataFile = SD.open(format_buf, FILE_WRITE, true);
  if(!dataFile) return;
  Serial.printf("Writing %s: ", format_buf);

  dataFile.printf("%f, %f, %f\n", latitude_signed(GPS), longitude_signed(GPS), GPS.altitude);

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
void sdBegin(void)
{
    //Check if file exists and create one if not
    if (!SD.exists("/README.txt"))
    {
        Serial.println("Creating README");
        File read_me = SD.open("/README.txt", FILE_WRITE);
        if(!read_me) return;

        //Create header with title, timestamp, and column names
        read_me.println("");
        read_me.printf(
            "Cal Poly Tide Sensor\n"
            "https://github.com/caleb-ad/wave-tide-sensor\n\n"
            "Data File format:\n"
            "Line   1: Latitude, Longitude, Altitude\n"
            "Line 'n': UNIX Time, Distance (mm), External Temp (F), Humidity (%)\n",
            unixTime(getTime()));
        read_me.close();

        SD.mkdir("/Data");
    }
}

//Powers down all peripherals, Sleeps until woken after set interval, runs setup() again
void goto_sleep(void) {
    //Print sleep time info
    //* Why does the printf not work with both the str and the number?
    Serial.printf("Going to sleep at %s\n", displayTime(getTime()));
    if(GPS.fixquality > 1) Serial.println("GPS has fix");

    writeLog("sleeping");

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
    //schedule to wake up so that the next measurements are centered at the next shceduled measurement time
    if(GPS.fixquality > 1) {
        uint64_t next_measurement = (MINUTE_ALLIGN - (GPS.minute % MINUTE_ALLIGN)) * (60 * 1000000) - (GPS.seconds * 1000000) - ((millis() - gps_millis_offset) % 1000) * 1000;
        esp_sleep_enable_timer_wakeup(
            next_measurement > half_read_time ?
            next_measurement - half_read_time :
            next_measurement + (MINUTE_ALLIGN * 60 * 1000000) - half_read_time );
    }
    else { // when the GPS does not have a fix sleep for the correct interval, the GPS may have previously had a fix
        //*This could overflow
        esp_sleep_enable_timer_wakeup(1000000 * 60 * MINUTE_ALLIGN  - (1000000 * (rtc_time_get() - clock_start)) / rtc_slow_clk_hz);
    }


    esp_deep_sleep_start();
}

void writeLog(const char* message)
{
    //Open log file and write to it
    File logFile = SD.open("/logFile.txt", FILE_WRITE);
    if(!logFile) return;
    //logFile.seek(logFile.size());
    logFile.printf("%d/%d/20%d, %s: %s\n, wake count: %d", GPS.month, GPS.day, GPS.year, displayTime(getTime()), message, wakeCounter);
    logFile.close();
}
