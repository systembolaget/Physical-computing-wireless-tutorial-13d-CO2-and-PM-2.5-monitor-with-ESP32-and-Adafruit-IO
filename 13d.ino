// Tutorial 13d. CO2 and PM 2.5 monitor with ESP32 and Adafruit IO

// Main parts: Adafruit Metro Mini, Adafruit AirLift FeatherWing ESP32,
// SENSIRION SCD30 and SPS30, free AIO subscription

// Libraries required to interface with the sensors, interface with
// the transceiver via SPI and to manage WLAN and MQTT communication;
// use the latest versions
#include <SPI.h> // Serial Peripheral Interface, a communication protocol
#include <Wire.h> // I2C, a communication protocol
#include <WiFiNINA.h> // Adafruit's WiFiNINA fork, use version 1.4.0
#include "Adafruit_MQTT.h" // Message Queuing Telemetry Transport, a communication protocol
#include "Adafruit_MQTT_Client.h"
#include <sps30.h> // From SENSIRION's GitHub repository
#include "SparkFun_SCD30_Arduino_Library.h" // Sparkfun's SCD30 library
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

// Variables that remain constant
// WiFi
#define SPIWIFI SPI // SPI port
#define SPIWIFI_SS 4 // AirLift ESP32 chip select pin
#define ESP32_RESET 3 // AirLift ESP32 reset pin
#define SPIWIFI_ACK 2 // AirLift ESP32 ready pin
#define ESP32_GPIO0 -1 // AirLift ESP32 pin not used
#define WLAN_SSID "Lagom" // WLAN router SSID
#define WLAN_PASS "63948100905083530879" // WLAN router key
//#define WLAN_SSID "Bra" // Smartphone hotspot SSID
//#define WLAN_PASS "dcb62d5396ad" // Smartphone hotspot key
// AIO
#define AIO_SERVER "io.adafruit.com" // MQTT broker/server host
#define AIO_SERVERPORT 8883 // Secure port, 1883 insecure port
#define AIO_USERNAME "LagomBra" // AIO user name
#define AIO_KEY "aio_OxVh64x79j022uOLdqt7bUWHGkZA" // AIO key

const int intervalSPS30PM25 = 5000; // MQTT broker publish interval
const int intervalSCD30CO2 = 10000; // MQTT broker publish interval

// Variables that can change
unsigned long timeSPS30PM25 = 0; // Timestamp that updates each loop() iteration
unsigned long timeSCD30CO2 = 0; // Timestamp that updates each loop() iteration

float SPS30PM25 = 0; // Mass concentration in micrograms per cubic meter (μg/m3)
float SCD30CO2 = 0; // Gas concentration in parts per million (ppm)

bool stateLED = LOW; // Tracks if the LED is on/off
unsigned long timeLED = 0; // Timestamp that updates each loop() iteration
unsigned long timeIntervalLED = 0; // Tracks the LED's on/off flash interval
bool triggerFlash = false; // Tracks if flashing was triggered

uint8_t status = WL_IDLE_STATUS;

// Instances an object from the WiFiNINA library to connect and
// transfer data with SSL/TLS support
WiFiSSLClient client;

// Instances a client object from the MQTT_Client library with a
// WLAN client, MQTT server, port and credentials
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Instance publishing objects from the MQTT_Client library
Adafruit_MQTT_Publish SPS30PM25feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/PM25");
Adafruit_MQTT_Publish SCD30CO2feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/CO2");

// Instances an object from the SCD30 library
SCD30 SCD30CO2Sensor;

Adafruit_7segment displaySPS30 = Adafruit_7segment(); // Red display
Adafruit_7segment displaySCD30 = Adafruit_7segment(); // White display

void setup()
{
  // Override default pins with the AirLift's breakout board pins
  WiFi.setPins(SPIWIFI_SS, SPIWIFI_ACK, ESP32_RESET, ESP32_GPIO0, &SPIWIFI);

  // Indicate that there is no WLAN connection
  WiFi.setLEDs(0, 255, 0); // Red

  Wire.begin();
  SCD30CO2Sensor.begin(); // Start the CO2 sensor
  SCD30CO2Sensor.setAltitudeCompensation(57); // Metres above sea level
  SCD30CO2Sensor.setAmbientPressure(1015); // Average pressure on location in mBar

  sensirion_i2c_init(); // Start the PM sensor
  sps30_set_fan_auto_cleaning_interval_days(3); // Run internal fan at full power every 3 days
  sps30_start_measurement(); // Start the measuring function

  displaySPS30.begin(0x75); // Start the 7-segment display
  displaySCD30.begin(0x76);
}

void loop()
{
  // A call to this function connects to the WLAN router
  connectToWLAN();

  // A call to this function connects to the MQTT broker
  connectToMQTT();

  if (millis() - timeSPS30PM25 >= intervalSPS30PM25)
    // When it is time to read and publish a sensor value
  {
    timeSPS30PM25 = millis(); // Update the timestamp for the next loop() iteration

    readSPS30(); // And call to function that reads a value from the sensor

    // Indicate that data will be published
    triggerFlash = true;
  }

  if (millis() - timeSCD30CO2 >= intervalSCD30CO2)
  {
    timeSCD30CO2 = millis();

    readSCD30();

    triggerFlash = true;
  }

  // If an event triggered flashing
  if (triggerFlash == true)
  {
    // Then flash the LED with different on and off intervals n times
    flash(125, 75, 3, 255, 255, 255); // White
  }
}

void connectToWLAN()
{
  // Return to loop() if already connected to the WLAN router
  if (WiFi.status() == WL_CONNECTED)
  {
    return;
  }

  // Connect to the Wifi router
  do
  {
    // Indicate that there is no WLAN connection
    WiFi.setLEDs(0, 255, 0); // Red

    // Start the connection
    status = WiFi.begin(WLAN_SSID, WLAN_PASS);

    // Wait until connected
    delay(100);

    // Repeat as long as WiFi status returns "not connected"
  } while (status != WL_CONNECTED);

  // Indicate that the WiFi connection is active
  WiFi.setLEDs(192, 255, 0); // Yellow
}

void connectToMQTT()
{
  // Stores a printable string version of the error code returned by
  // connect()
  int8_t MQTTerrorString;

  // Number of connection attempts before a hard reset is necessary
  uint8_t retries = 3;

  // Return to loop() if already connected to the MQTT broker
  if (mqtt.connected())
  {
    return;
  }

  // In case the error code is not 0 = successful connection, then
  while ((MQTTerrorString = mqtt.connect()) != 0)
  {
    // Send a MQTT disconnect packet and break the connection
    mqtt.disconnect();

    // And wait 3 seconds, and then retry to connect
    delay(3000);

    // Then decrement the retry-counter
    retries--;

    // If no MQTT broker connection can be (re-)established
    if (retries == 0)
    {
      // Then indicate that the connection permanently failed
      WiFi.setLEDs(0, 255, 0); // Red

      // End and wait for user to press the reset button
      while (1)
        ;
    }
  }
  // Indicate that the MQTT connection is active
  WiFi.setLEDs(255, 0, 0); // Green
}

void readSPS30() // Don't call this function more often than every second (company documentation)
{
  struct sps30_measurement m;
  sps30_read_measurement(&m);
  SPS30PM25 = m.mc_2p5; // Read output into a variable. Other PM and NC values also possible

  SPS30PM25feed.publish((float)SPS30PM25);

  displaySPS30.print(SPS30PM25); displaySPS30.writeDisplay(); // Only for display usage
  //Serial.print("PM 2.5: "); Serial.println(SPS30PM25); Serial.println(); // Only for debugging
}

void readSCD30() // Don't call this function more often than every five seconds (company documentation)
{
  SCD30CO2 = SCD30CO2Sensor.getCO2(); // Read output into a variable. Humidity and temperature also possible

  SCD30CO2feed.publish((float)SCD30CO2);

  displaySCD30.print(SCD30CO2); displaySCD30.writeDisplay(); // Only for display usage
  //Serial.print("CO2: "); Serial.println(SCD30CO2); Serial.println(); // Only for debugging
}

void flash(int timeon, int timeoff, byte flashes, byte g, byte r, byte b)
{
  // A variable to count how often the LED flashed (on/off). The static
  // keyword preserves a variable's value between function calls, unlike
  // a local variable declared and destroyed at each new function call
  static byte counter = 0;

  // Check if it is time to flash the LED
  if (millis() - timeLED > timeIntervalLED)
  {
    // Create a new timestamp for the next loop() execution
    timeLED = millis();

    // First check, if the LED was off (= LOW); if it was
    if (stateLED == LOW)
    {
      // Use the on-time set in the function call
      timeIntervalLED = timeon;

      // Then switch the LED on with the specified colours
      WiFi.setLEDs(g, r, b);

      // And remember that it is now on
      stateLED = HIGH;
    }

    // Otherwise, if the LED was on (= HIGH)
    else
    {
      // Use the off-time set in the function call
      timeIntervalLED = timeoff;

      // Then switch the LED off
      WiFi.setLEDs(0, 0, 0);

      // And remember that it is now off
      stateLED = LOW;

      // Finally increment the counter variable at each on/off cycle
      counter++;
    }
  }

  // Check if the number of on/off cycles matches the number of flashes
  // set in the function call and if it does
  if (counter >= flashes)
  {
    // Reset the flash cycle counter to zero
    counter = 0;

    // And stop the switch triggering flashes, if the user continues
    // holding the momentary switch button down. This ensures there
    // is only a "single shot"/"one shot" operation
    triggerFlash = false;

    // Finally set LED back to WLAN and MQTT connected colour
    WiFi.setLEDs(255, 0, 0);
  }
}
