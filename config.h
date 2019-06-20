// config file to keep secret
const char *ssid     = "*****"; // Your SSID here
const char *password = "*********"; // Your password here

#define METRICS_JOB         "env_sensor"
#define METRICS_INSTANCE    "Office"

#define SERVER_IP           "192.168.7.10"
#define SERVER_PORT         9091

IPAddress ip(192, 168, 252, 20);   // The address 192.168.252.20 is arbitary, if could be any address in the range of your router, but not another device!
IPAddress gateway(192,168,252,1);  // My router has this base address
IPAddress subnet(255,255,254,0); // Define the sub-network
