// ESP8266 WebServer - using DHT11 sensor
// (c) D L Bird 2016
//
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Ticker.h>
#include "prometheus.h"

const char *ssid     = "*****"; // Your SSID here
const char *password = "*********"; // Your password here

#define METRICS_JOB         "env_sensor"
#define METRICS_INSTANCE    "Office"

#define MAX_LOOP_TIME_MS     10000

#define SEALEVELPRESSURE_HPA (1013.25)

const String metrics_url = "/metrics/job/" + String(METRICS_JOB) +
                           "/instance/" + String(METRICS_INSTANCE);

#define SERVER_IP           "192.168.252.20"

#define SERVER_PORT         9091

Ticker sleepTicker;

IPAddress ip(192, 168, 252, 20);   // The address 192.168.252.20 is arbitary, if could be any address in the range of your router, but not another device!
IPAddress gateway(192,168,252,1);  // My router has this base address
IPAddress subnet(255,255,254,0); // Define the sub-network

Adafruit_BME280 bme; // Note Adafruit assumes I2C adress = 0x77 my module (eBay) uses 0x76 so the library address has been changed.

unsigned long startTime;

void sleepyTime() {
  const int elapsed = millis() - startTime;
  Serial.printf("Sleeping. Loop took %d ms\n", elapsed);
  // If this sleep happened because of timeout, clear the
  // Wifi state.
  if (elapsed >= MAX_LOOP_TIME_MS) {
    WiFi.disconnect();
  }
  ESP.deepSleep(480000000, WAKE_RF_DEFAULT);
  // It can take a while for the ESP to actually go to sleep.
  // When it wakes up we start again in setup().
  delay(5000);
}

void setup() {
  startTime = millis();
  sleepTicker.once_ms(12000, &sleepyTime);
  Serial.begin(115200);
  Serial.print("Connecting to ");
  Serial.println(ssid);          // Connect to WiFi network
  WiFi.config(ip, gateway, subnet);
  WiFi.persistent(false);  // disables the storage of credentials to flash.
  WiFi.begin(ssid, password);   
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected..");
  // Start the webserver
  // server.begin();
  Serial.println("Webserver started...");
  pinMode(D3, INPUT_PULLUP); //Set input (SDA) pull-up resistor on

  Wire.setClock(2000000);    // Set I2C bus speed 
  Wire.begin(D2,D1); // Define which ESP8266 pins to use for SDA, SCL of the Sensor
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
}


bool isValidHumidity(const float humidity) {
  return (!isnan(humidity) && humidity >= 0 && humidity <= 100);
}

bool isValidTemp(const float temp) {
  return (!isnan(temp) && temp >= -100 && temp <= 212);
}

MapMetric makeMetric(const String& name, const float value) {
  MapMetric m = MapMetric("env_sensor_" + name, "location");
  m.Add(METRICS_INSTANCE, value);
  return m;
}

void loop(void)
{
  PrometheusClient pclient =
      PrometheusClient(SERVER_IP, SERVER_PORT,
                       METRICS_JOB, METRICS_INSTANCE);
                           
  float temp = bme.readTemperature();
  float humidity = bme.readHumidity();
  while (!(isValidTemp(temp) && isValidHumidity(humidity))) {
    delay(100);
    temp = bme.readTemperature();
    humidity = bme.readHumidity();
  }

  const int bmeTime = millis() - startTime;
  Serial.printf("BME read took %d ms\n", bmeTime);
  // waitForWifi();
  const int wifiTime = millis() - (startTime + bmeTime);
  Serial.printf("WiFi init took an additional %d ms\n", wifiTime);
 
  pclient.AddMetric(makeMetric("humidity", humidity));
  pclient.AddMetric(makeMetric("tempC", temp));
  pclient.AddMetric(makeMetric("free_heap", ESP.getFreeHeap()));
  pclient.AddMetric(makeMetric("pressure", bme.readPressure()));
  pclient.AddMetric(makeMetric("altitude", bme.readAltitude(SEALEVELPRESSURE_HPA)));
  pclient.AddMetric(makeMetric("loop_time", float(millis() - startTime)));
  pclient.PrintSerial();
  pclient.Send();
  sleepyTime();
} 
