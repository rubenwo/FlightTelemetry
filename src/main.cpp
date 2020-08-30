#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <RF24.h>
#include <Adafruit_BMP085.h>

TinyGPSPlus gps;
HardwareSerial gps_serial(2);
Adafruit_BMP085 bmp180;
float pressure_offset;

/****************** User Config ***************************/
/***      Set this radio as radio number 0 or 1         ***/
bool radioNumber = 0;

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(4, 5); //CE, CSN
/**********************************************************/

byte addresses[][6] = {"1Node", "2Node"};

// Used to control whether this node is sending or receiving
bool role = 0;

struct telemetry_msg
{
  uint64_t msg_num = 0;

  float temperature;

  double baro_altitude;
  double baro_raw;

  double gps_altitude;
  double gps_course_degree;
  double gps_hdop;
  double gps_lon;
  double gps_lat;

  uint32_t gps_sat_amount;
  double gps_kmph;
};

struct ack_msg
{
  uint64_t msg_num = 0;
};

telemetry_msg msg;
ack_msg ack;

void setup()
{
  Serial.begin(460800);

  SPI.begin();
  SPI.setFrequency(2000000);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(0);
  if (radio.begin())
  {
    Serial.println("OK!");
  }
  else
  {
    Serial.println("too bad!");
  }
  delay(5000);
  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.

  radio.setPALevel(RF24_PA_LOW);

  // Open a writing and reading pipe on each radio, with opposite addresses
  if (radioNumber)
  {
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1, addresses[0]);
    radio.startListening();
  }
  else
  {
    gps_serial.begin(9600);
    bmp180.begin();
    pressure_offset = bmp180.readSealevelPressure();

    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1, addresses[1]);
  }

  // Start the radio listening for data
}
void loop()
{
  if (radioNumber)
  {
    if (radio.available())
    {
      radio.read(&msg, sizeof(telemetry_msg));
      Serial.printf("Message Num: %d, Temperature: %f *C, Altitude: %f meters, Pressure: %d hPa, GPS Altitude: %f meters, Course degree: %f, GPS HDOP: %f, GPS Kmph: %f, Latitude: %f, Longitude: %f, Satellites: %d\n", msg.msg_num, msg.temperature, msg.baro_altitude, msg.baro_raw, msg.gps_altitude, msg.gps_course_degree, msg.gps_hdop, msg.gps_kmph, msg.gps_lat, msg.gps_lon, msg.gps_sat_amount);
    }
  }
  else
  {
    while (gps_serial.available() > 0)
    {
      gps.encode(gps_serial.read());
    }
    msg.temperature = bmp180.readTemperature();
    msg.baro_altitude = bmp180.readAltitude(pressure_offset);
    msg.baro_raw = bmp180.readPressure();
    msg.gps_altitude = gps.altitude.meters();
    msg.gps_course_degree = gps.course.deg();
    msg.gps_hdop = gps.hdop.hdop();
    msg.gps_kmph = gps.speed.kmph();
    msg.gps_lat = gps.location.lat();
    msg.gps_lon = gps.location.lng();
    msg.gps_sat_amount = gps.satellites.value();

    radio.write(&msg, sizeof(telemetry_msg));

    msg.msg_num += 1;
  }
}