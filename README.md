# arduino-bme280-prometheus

Arduino Code using:

* [WEMOS D1 mini](https://wiki.wemos.cc/products:d1:d1_mini) board using WiFi
* [BME280 Temperature, Humidity and Pressure](https://www.adafruit.com/product/2652) sensor
* pushing metrics to [Prometheus Pushgateway](https://github.com/prometheus/pushgateway)

Prometheus Pushgateway (in my case two of them running on a Raspberry Pi 3 and a Raspberry Pi Zero W) are the target for Arduino to avoid running a web server on the Arduino and it is really a caching layer for metrics and later on scraped by Prometheus (alternative can be MQTT)
