#include "config.h"
#include "Dali.h"
#include "Mqtt_light.h"


#include <WiFi.h>
#include <WiFiClient.h>

//https://github.com/knolleary/pubsubclient
#include <PubSubClient.h>

// UNO pin
//const int DALI_TX = 3;
//const int DALI_RX_A = 0;

// ESP pin
const int DALI_TX = 33;    // ADC1      (no use 0,2,15,...)
const int DALI_RX_A = 32;  // ESP only ADC pin

unsigned int last_dev = 99;
unsigned long last_scan = 0;
boolean scan_busy = false;

const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;
const char *mqtt_server = MQTT_ADDRESS;

WiFiClient espClient;
PubSubClient pubSubClient(espClient);

void mqtt_callback(char *top, byte *pay, unsigned int length);


unsigned int wifiDownSince = 0;

void WiFiEvent(WiFiEvent_t event)
{
  //Serial.printf("[WiFi-event] event: %d\r\n", event);

  switch (event)
  {
  case SYSTEM_EVENT_STA_GOT_IP:
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    Serial.println("WiFi lost connection");
    wifiDownSince = millis();
    break;
  }
}

void setup_wifi()
{
  WiFi.disconnect(true);
  delay(1000);
  Serial.printf("Wifi connecting to: %s ... \r\n", ssid);
  WiFi.onEvent(WiFiEvent);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  wifiDownSince = 0;
}

unsigned long nextMqttAttempt = 0;

String topPrefix(const char *top)
{
  String ret = String(MQTT_TOPIC_PREFIX) + top;
  return ret;
}
String lightPrefix(const char *top, uint8_t add_byte)
{
  snprintf(m_topic_buffer, MSG_TOPIC_SIZE, "%s/light%d%s", MQTT_TOPIC_PREFIX, add_byte, top);
  //String ret = String(MQTT_TOPIC_PREFIX) + top;
  return m_topic_buffer;
}
void publishBuffer(const char *top, uint8_t device_add)
{
  pubSubClient.publish(lightPrefix(top, device_add).c_str(), m_msg_buffer, true);
}


void mqtt_pub_one(uint8_t device_add){
  const int delayTime = 10;
  uint8_t level;
  uint8_t status;
	uint8_t q_on;

  //Serial.println("Lamp level:");
  dali.LightCmd(device_add, QUERY_LEVEL);
  level = dali.receive();
  delay(delayTime);
  dali.LightCmd(device_add, QUERY_STATUS);
  status = dali.receive();
  delay(delayTime);
  dali.LightCmd(device_add, QUERY_ON);
  q_on = dali.receive();
  delay(delayTime);
  //Serial.println("Lamp 1 level:");
  //Serial.println(status, BIN);

  snprintf(m_msg_buffer, MSG_SIZE, "%d", level);
  publishBuffer(ML_BRIGHTNESS, device_add);

  snprintf(m_msg_buffer, MSG_SIZE, "%d", status);
  publishBuffer(ML_STATUS_RAW, device_add);

  snprintf(m_msg_buffer, MSG_SIZE, "%d", millis());
  publishBuffer(ML_UPDATE_RAW, device_add);


  if (q_on > 1){
    snprintf(m_msg_buffer, MSG_SIZE, "%s", LIGHT_ON);
  }else{
    snprintf(m_msg_buffer, MSG_SIZE, "%s", LIGHT_OFF);
  }
  publishBuffer(ML_STATE, device_add);

  pubSubClient.subscribe(lightPrefix(ML_BRIGHTNESS_SET, device_add).c_str());
  pubSubClient.subscribe(lightPrefix(ML_STATE_SET, device_add).c_str());

}


void pubAll(){
  
	const uint8_t start_ind_adress = 0;
	uint8_t device_add;

	for (device_add = start_ind_adress; device_add <= 63; device_add++)
	{
		mqtt_pub_one(device_add);
  }
  Serial.println("End pub level:");
}


void reconnect_mqtt()
{
  if (WiFi.status() == WL_CONNECTED && millis() > nextMqttAttempt)
  {
    Serial.print("Attempting MQTT connection...\r\n");
    String mqttPrefix = String(MQTT_TOPIC_PREFIX);
    // Attempt to connect
    if (pubSubClient.connect(topPrefix("-gateway").c_str(), MQTT_USERNAME, MQTT_PASSWORD, topPrefix("/LWT").c_str(), 0, false, "Offline"))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      pubSubClient.publish(topPrefix("/LWT").c_str(), "Online", true);
      pubSubClient.subscribe(topPrefix("/restart").c_str());
      pubSubClient.subscribe(topPrefix("/scan").c_str());
      pubSubClient.subscribe(topPrefix("/init").c_str());
      pubSubClient.subscribe(topPrefix("/sinus").c_str());
      pubSubClient.subscribe(topPrefix("/all/set").c_str());
      //pubSubClient.subscribe(topPrefix("/all/set_position").c_str());
      pubSubClient.loop();

      char discTopic[128];
      char discPayload[300];

      pubAll();
      //sprintf(discTopic, discSwitchTopic, MQTT_TOPIC_PREFIX);
      //sprintf(discPayload, discSwitchPayload, MQTT_TOPIC_PREFIX, MQTT_TOPIC_PREFIX, MQTT_TOPIC_PREFIX);
      //pubSubClient.publish(discTopic, discPayload, true);
      //pubSubClient.publish(topPrefix("/enabled").c_str(), "ON", true);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(pubSubClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      nextMqttAttempt = millis() + 5000;
    }
  }
}

bool otaUpdating = false;

void setup()
{

  Serial.begin(115200);

  // first init DALI
  Serial.println("Configuring DALI...");
  dali.setupTransmit(DALI_TX);
  dali.setupAnalogReceive(DALI_RX_A);
  dali.busTest();
  dali.msgMode = true;

  Serial.print("Levels:");
  Serial.print(dali.minLevel, DEC);
  Serial.print("-");
  Serial.println(dali.maxLevel, DEC);
  Serial.println(dali.analogLevel, DEC);
  if (dali.analogLevel == 0){
    delay(1000);
    Serial.println("Re bustest..");
    dali.busTest();
    Serial.println(dali.analogLevel, DEC);
  }

  Serial.println("Configuring wifi...");
  setup_wifi();
  pubSubClient.setServer(mqtt_server, 1883);
  pubSubClient.setCallback(mqtt_callback);

  help(); //Show help

  //dali.scanShortAdd(); //
  //delay(200);
  //dali.initialisation();


  //dali.transmit(BROADCAST_C, ON_C);
  //delay(4000);
  //sinus();

  //testReceive();

  //delay(10);
  //level1();

}

void help()
{
  Serial.println("Enter 16 bit command or another command from list:");
  Serial.println("help -  command list");
  Serial.println("on -  broadcast on 100%");
  Serial.println("off -  broadcast off 0%");
  Serial.println("scan -  device short address scan");
  Serial.println("initialise -  start process of initialisation");
  Serial.println();
}

void sinus()
{
  uint8_t lf_1_add = 0;
  uint8_t lf_2_add = 1;
  uint8_t lf_3_add = 2;
  uint8_t lf_1;
  uint8_t lf_2;
  uint8_t lf_3;
  int i;
  int j = 0;

  for (i = 0; i < 60; i = i + 1)
  { //360

    lf_1 = (int)abs(254 * sin(i * 3.14 / 180));
    lf_2 = (int)abs(254 * sin(i * 3.14 / 180 + 2 * 3.14 / 3));
    lf_3 = (int)abs(254 * sin(i * 3.14 / 180 + 1 * 3.14 / 3));
    dali.transmit(lf_1_add << 1, lf_1);
    delay(5);
    dali.transmit(lf_2_add << 1, lf_2);
    delay(5);
    dali.transmit(lf_3_add << 1, lf_3);
    delay(5);
    //Serial.println(dali.receive());

    delay(20);
  }
}

void level1()
{
  dali.transmit(0b00000001, QUERY_LEVEL);
  Serial.println("Lamp 1 level:");
  Serial.println(dali.receive());
  delay(10);

  dali.transmit(0b00000010, QUERY_LEVEL);
  Serial.println("Lamp 2 level:");
  Serial.println(dali.receive());
  delay(10);

  dali.transmit(5, QUERY_LEVEL);
  Serial.println("Lamp 3 level:");
  Serial.println(dali.receive());
  delay(10);
}

void testReceive()
{

  Serial.println("Test:");
  //dali.transmit(0, 254);
  //delay(200);
  //dali.transmit(1, 0x90);
  //Serial.println(dali.receive());
  delay(200);
  dali.transmit(5, 0x90);
  Serial.println(dali.receive());
  delay(2000);
  //dali.transmit(5, OFF_C);
  //Serial.println(dali.receive());
}

void loop()
{

  const int delaytime = 500;
  int i;
  int cmd1;
  int cmd2;
  String comMsg;

  if (WiFi.status() != WL_CONNECTED && wifiDownSince > 0 && millis() - wifiDownSince > 20000) {
    setup_wifi();
  }
  if (!pubSubClient.connected()) {
    reconnect_mqtt();
  }
  pubSubClient.loop();

  // Read command from port

  //delay(delaytime);

  //while (Serial.available()) {
  //  comMsg = comMsg + (char)(Serial.read());
  //  Serial.println(comMsg);
  //}; // read data from serial

  if (comMsg == "sinus")
  {
    sinus();
  };

  if (comMsg == "scan")
  {
    Serial.println("s+");
    dali.scanShortAdd();
  }; // scan short addresses

  if (comMsg == "on")
  {
    Serial.println("o+");
    dali.transmit(BROADCAST_C, ON_C);
  }; // broadcast, 100%

  if (comMsg == "off")
  {
    dali.transmit(BROADCAST_C, OFF_C);
  }; // broadcast, 0%

  if (comMsg == "initialise" or comMsg == "ini")
  {
    Serial.println("i+");
    dali.initialisation();
  }; // initialisation

  if (comMsg == "level")
  {
    level1();
  };

  if (comMsg == "help")
  {
    help();
  }; //help

  if (comMsg == "test")
  {
    testReceive();
  }; //graph

  //if (dali.cmdCheck(comMsg, cmd1, cmd2)) {
  //  dali.transmit(cmd1, cmd2);  // command in binary format: (address byte, command byte)
  //}
  //delay(delaytime);

  // scan 2s
  if (millis() - last_scan > 2000 && !scan_busy) {
    scan_busy = true;
    last_scan = millis(); 
    
    uint16_t pub_dev = 0;
    if (mql.need_update > 0){
      //Serial.println("need_update");
      pub_dev = mql.need_update;
      mql.need_update = -1;
    }else{
      last_dev += 1;
      if (last_dev > 63)
        last_dev = 0;
      pub_dev = last_dev;
    }

    mqtt_pub_one(pub_dev);
    scan_busy = false;
  }
  //temp1 = true;

  //  lastTemp = millis();
  //uint8_t tf = temprature_sens_read();
  //float   tc = ( tf - 32 )/1.8;
  //Serial.printf("Temp=%dC hal=%d\n",(int)tc, hall_sens_read());
  //  Serial.printf("Temp=%dC hal=%d\n",(int)0, 1);
  //temp1 = false;

  //}
};

