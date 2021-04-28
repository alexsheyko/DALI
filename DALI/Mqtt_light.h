
// MQTT: topics
//const char* MQTT_LIGHT_STATE_TOPIC = "office/light1/status";
//const char* MQTT_LIGHT_COMMAND_TOPIC = "office/light1/switch";

const PROGMEM char* ML_STATE = "/state";
const PROGMEM char* ML_STATE_SET = "/state/set";
const PROGMEM char* ML_STATUS_RAW = "/status_raw";
const PROGMEM char* ML_UPDATE_RAW = "/last_updated_raw";

const PROGMEM char* MQTT_LIGHT_COMMAND = "/switch";

const PROGMEM char* ML_BRIGHTNESS = "/brightness";  //status
const PROGMEM char* ML_BRIGHTNESS_SET = "/brightness/set";

// payloads by default (on/off)
const char* LIGHT_ON = "ON";
const char* LIGHT_OFF = "OFF";

const uint8_t MSG_SIZE = 20;
char m_msg_buffer[MSG_SIZE]; 
const uint8_t MSG_TOPIC_SIZE = 150;
char m_topic_buffer[MSG_TOPIC_SIZE]; 

void sinus1();

class Mqtt_light
{
public:
	Mqtt_light();	
  int need_update=-1;
  int max_device = 64;
  int dev_answer[64];
	uint16_t delay2;
	uint16_t period;
}; //end of class

Mqtt_light::Mqtt_light() //constructor
{
  need_update = -1;
	//applyWorkAround1Mhz = 0;
}

Mqtt_light mql = Mqtt_light();


void mqtt_callback(char *top, byte *pay, unsigned int length)
{
  pay[length] = '\0';
  String payload = String((char *)pay);
  String topic = String(top);
  Serial.printf("MQTT [%s]%d: %s\r\n", top, length, payload.c_str());

  int i1, i2, i3;
  int pi;

  i1 = topic.indexOf('/');
  i2 = topic.indexOf('/', i1 + 1);
  String address = topic.substring(i1 + 1, i2);
  String command = topic.substring(i2 + 1);
  //Serial.printf("Addr: %s Cmd: %s\r\n", address.c_str(), command.c_str());
  payload.toLowerCase();

  int num = -1;
  String light = address.substring(0,5);
  if (light == "light"){
    num = address.substring(5).toInt();
  }
  //Serial.println(num);
  //Serial.println(light);

  if (address == "scan") {
      if (payload == "on") {
        dali.scanShortAdd();
      }
      delay(1000);
  }
  if (address == "init") {
      if (payload == "on") {
        dali.initialisation();
      }
      delay(1000);
  }
  if (address == "sinus") {
      if (payload == "on") {
        sinus1();
      }
      delay(1000);
  }

  if (num >= 0){
    
    if (command == "state/set") {
      if (payload == "off") {
        dali.LightCmd(num, OFF_C);
      } else if (payload == "on") {
        dali.LightCmd(num, ON_C);
      }
      delay(10);
    }

    if (command == "brightness/set") {
      pi = payload.toInt();
      if (pi > 254)
        pi = 254;
      if (pi < 0)
        pi = 0;
      
      dali.LightLevel(num, pi);
      //Serial.printf("brightness: %d for %d \r\n", pi, num);
      delay(10);
    }

    mql.need_update = num;
  }
  
}


void sinus1()
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
    dali.LightLevel(lf_1_add, lf_1);
    delay(5);
    dali.LightLevel(lf_2_add, lf_2);
    delay(5);
    dali.LightLevel(lf_3_add, lf_3);
    delay(5);
    //Serial.println(dali.receive());

    delay(20);
  }
}