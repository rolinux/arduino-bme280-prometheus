// ESP8266 WebServer - using BME280 sensor
// https://github.com/rolinux/arduino-bme280-prometheus
//
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Ticker.h>
#include "prometheus.h"
#include "config.h"

// target Prometheus Pushgateway URL
const String metrics_url = "/metrics/job/" + String(METRICS_JOB) +
                           "/instance/" + String(METRICS_INSTANCE);

Adafruit_BME280 bme; // Note Adafruit assumes I2C address = 0x77 but later use set it as 0x76

void setup() {
  // debug on Serial
  Serial.begin(115200);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.config(ip, gateway, subnet); // Connect to WiFi network
  WiFi.persistent(false);  // disables the storage of credentials to flash.
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected..");
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

  pclient.AddMetric(makeMetric("humidity", humidity));
  pclient.AddMetric(makeMetric("temperature", temp));
  pclient.AddMetric(makeMetric("free_heap", ESP.getFreeHeap()));
  pclient.AddMetric(makeMetric("pressure", bme.readPressure()));
  pclient.PrintSerial();
  pclient.Send();
  delay(DELAY_BETWEEN_READINGS);        // Control speed of BME280 sensor reading
}
