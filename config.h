// config file to keep secrets/config separate
const char *ssid     = "*****"; // Your SSID here
const char *password = "*********"; // Your password here

#define METRICS_JOB         "env_sensor"
#define METRICS_INSTANCE    "Office" // where the Arduino is located

#define SERVER_IP           "192.168.7.10" // target Prometheus Pushgateway IP
#define SERVER_PORT         9091  // target Prometheus Pushgateway port

#define DELAY_BETWEEN_READINGS         30000  // in miliseconds

IPAddress ip(192, 168, 252, 20);   // The address 192.168.252.20 is in my case on the IoT VLAN
IPAddress gateway(192,168,252,1);  // My router has this base address
IPAddress subnet(255,255,254,0); // Define the sub-network (/23)
