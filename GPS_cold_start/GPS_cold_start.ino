#include "Adafruit_GPS.h"
#include "UnixTime.h"


#define FORMAT_BUF_SIZE 100
#define GMT_to_PST(__GMT) (((__GMT) + 17) % 24)

#define SD_CS GPIO_NUM_5 //SD card chip select pin
#define SONAR_EN GPIO_NUM_33 //Sonar measurements are disabled when this pin is pulled low

#define GPS_RX GPIO_NUM_16
#define GPS_TX GPIO_NUM_17
#define GPS_CLOCK_EN GPIO_NUM_27
#define GPS_POLLING_HZ 100
#define GPS_DIFF_FROM_GMT 0

#define TEMP_EN GPIO_NUM_15

Adafruit_GPS GPS(&Serial2);
uint32_t gps_millis_offset = millis();

char format_buf[FORMAT_BUF_SIZE];

// Return a current time_stamp
// UnixTime getTime(void)
// {
//     UnixTime stamp(GPS_DIFF_FROM_GMT);
//     stamp.setDateTime(2000 + GPS.year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds);
//     return stamp;
// }

//human readable time, in PST
char* displayTime(UnixTime now) {
    snprintf(format_buf, FORMAT_BUF_SIZE, "%02d:%02d:%02d.%03d", GMT_to_PST(now.hour), now.minute, now.second, (millis() - gps_millis_offset)%1000);
    return format_buf;
}

void setup(void) {
    Serial.begin(115200); //Serial monitor
    startGPS(); //uses Serial2

    //Turn on relevant pins
    gpio_hold_dis(GPIO_NUM_15);
    gpio_hold_dis(GPIO_NUM_33);
    gpio_hold_dis(GPIO_NUM_27);
    pinMode(SD_CS, OUTPUT);
    pinMode(GPS_CLOCK_EN, OUTPUT);
    pinMode(TEMP_EN, OUTPUT);
    pinMode(SONAR_EN, OUTPUT);
    digitalWrite(GPS_CLOCK_EN, HIGH); //Hold clock high
    digitalWrite(TEMP_EN, LOW); //Hold high to supply power to temp sensor
    digitalWrite(SONAR_EN, LOW); //Hold high to begin measuring with sonar
    digitalWrite(SD_CS, LOW);


    //Setup for LED
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
}

void loop(void) {
    // GPS.read(); //if GPS.read() takes longer than the GPS polling frequency, execution may get stuck in this loop
    // if(GPS.newNMEAreceived()) {
    //     // a tricky thing here is if we print the NMEA sentence, or data
    //     // we end up not listening and catching other sentences!
    //     // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //     //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
    //     GPS.parse(GPS.lastNMEA());  // this also sets the newNMEAreceived() flag to false
    //     gps_millis_offset = millis() - GPS.milliseconds;
    // }

    // if(GPS.fixquality > 1) digitalWrite(LED_BUILTIN, HIGH);
    // else digitalWrite(LED_BUILTIN, LOW);

    // Serial.printf("%d/%d/20%d, %s\n", GPS.month, GPS.day, GPS.year, displayTime(getTime()));
    Serial2.readBytesUntil('\n', format_buf, FORMAT_BUF_SIZE);
    Serial.write(format_buf, FORMAT_BUF_SIZE);
}

// Start GPS Clock
void startGPS(void)
{
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
}
