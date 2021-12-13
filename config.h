// config file to keep secrets/config separate
const char *ssid     = "*****"; // Your SSID here
const char *password = "*********"; // Your password here

#define METRICS_JOB         "env_sensor"
#define METRICS_INSTANCE    "Office" // where the Arduino is located

#define SERVER_IP_1        "192.168.64.2" // target Prometheus Pushgateway IP - 1st server
#define SERVER_IP_2        "192.168.64.3" // target Prometheus Pushgateway IP - 2nd server
#define SERVER_PORT         9091  // target Prometheus Pushgateway port

#define DELAY_BETWEEN_READINGS         30000  // in miliseconds

IPAddress ip(192, 168, 64, 22);  // Office IP for the Arduino
IPAddress gateway(192, 168, 64, 1); // My router has this base address
IPAddress subnet(255, 255, 224, 0); // Define the sub-network
