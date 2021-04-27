// Your Wifi SSID.
#define WIFI_SSID "MikroSun"

// Wifi password.
#define WIFI_PASSWORD "21212121"

// MQTT server details.
#define MQTT_ADDRESS "192.168.101.55"
#define MQTT_USERNAME ""
#define MQTT_PASSWORD ""

// WDT Timeout (in seconds) - comment out if not required
#define WDT_TIMEOUT 6

// Prefix for MQTT topics, you can change this if you have multiple ESP devices, etc.
#define MQTT_TOPIC_PREFIX "dali"

// Device class to report to HomeAssistant. Must be one of the values at
// https://www.home-assistant.io/integrations/light.mqtt/
#define MQTT_DEVICE_CLASS "light"
