#include <Dali.h>

//const int DALI_TX = 3;
//const int DALI_RX_A = 0;

const int DALI_TX = 2;
const int DALI_RX_A = 4;    // ESP only ADC pin

unsigned long lastTemp = 0;
//boolean temp1 = false;



void setup() {

  Serial.begin(115200);
  //Serial.begin(74880);
  dali.setupTransmit(DALI_TX);
  dali.setupAnalogReceive(DALI_RX_A);
  dali.busTest();
  dali.msgMode = true;
  Serial.println(dali.analogLevel, DEC);
  help(); //Show help
  
  //dali.scanShortAdd(); //
  //delay(200);
  //dali.initialisation();

  /*
  delay(200);
  dali.transmit(BROADCAST_C, OFF_C);
  delay(200);
  level1(); 
  delay(2000);
  */
  dali.transmit(BROADCAST_C, ON_C);
  delay(4000);
  //sinus_esp();
  level1();
  delay(1000);
  dali.transmit(2 << 1, 0x90);
  delay(2000);
  
  //testReceive();
    //dali.transmit(BROADCAST_C, QUERY_LEVEL);
    //Serial.println(dali.receive());
    //delay(10);
  level1();
  
}


void help() {
  Serial.println("Enter 16 bit command or another command from list:");
  Serial.println("help -  command list");
  Serial.println("on -  broadcast on 100%");
  Serial.println("off -  broadcast off 0%");
  Serial.println("scan -  device short address scan");
  Serial.println("initialise -  start process of initialisation");
  Serial.println();
}


void sinus () {
  uint8_t lf_1_add = 0;
  uint8_t lf_2_add = 1;
  uint8_t lf_3_add = 2;
  uint8_t lf_1;
  uint8_t lf_2;
  uint8_t lf_3;
  int i;
  int j = 0;

  while (Serial.available() == 0) {
    for (i = 0; i < 360; i = i + 1) {

      if (Serial.available() != 0) {
        dali.transmit(BROADCAST_C, ON_C);
        break;
      }

      lf_1 = (int) abs(254 * sin(i * 3.14 / 180));
      lf_2 = (int) abs(254 * sin(i * 3.14 / 180 + 2 * 3.14 / 3));
      lf_3 = (int) abs(254 * sin(i * 3.14 / 180 + 1 * 3.14 / 3));
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
}

void sinus_esp () {
  uint8_t lf_1_add = 0;
  uint8_t lf_2_add = 1;
  uint8_t lf_3_add = 2;
  uint8_t lf_1;
  uint8_t lf_2;
  uint8_t lf_3;
  int i;
  int j = 0;

  for (i = 0; i < 60; i = i + 1) {  //360

      lf_1 = (int) abs(254 * sin(i * 3.14 / 180));
      lf_2 = (int) abs(254 * sin(i * 3.14 / 180 + 2 * 3.14 / 3));
      lf_3 = (int) abs(254 * sin(i * 3.14 / 180 + 1 * 3.14 / 3));
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


void level1 () {
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

void testReceive () {

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


void loop() {

  const int delaytime = 500;
  int i;
  int cmd1;
  int cmd2;
  String comMsg;


  // Read command from port

  delay(delaytime);

  //while (Serial.available()) {
  //  comMsg = comMsg + (char)(Serial.read());
  //  Serial.println(comMsg);
  //}; // read data from serial


  if (comMsg == "sinus") {
    sinus();
  };

  if (comMsg == "scan") {
    Serial.println("s+");
    dali.scanShortAdd();
  }; // scan short addresses

  if (comMsg == "on") {
    Serial.println("o+");
    dali.transmit(BROADCAST_C, ON_C);
  }; // broadcast, 100%

  if (comMsg == "off") {
    dali.transmit(BROADCAST_C, OFF_C);
  }; // broadcast, 0%

  if (comMsg == "initialise" or comMsg == "ini") {
    Serial.println("i+");
    dali.initialisation();
  }; // initialisation


  if (comMsg == "level") {
    level1();
  };
  
  if (comMsg == "help") {
    help();
  }; //help

  if (comMsg == "test") {
    testReceive();
  }; //graph



  //if (dali.cmdCheck(comMsg, cmd1, cmd2)) {
  //  dali.transmit(cmd1, cmd2);  // command in binary format: (address byte, command byte)
  //}
  delay(delaytime);

  
    // temp 60s 
  //if (millis() - lastTemp > 6000 && !temp1) {
  //if (millis() - lastTemp > 6000) {
    //temp1 = true;
    
  //  lastTemp = millis();
    //uint8_t tf = temprature_sens_read();
    //float   tc = ( tf - 32 )/1.8; 
    //Serial.printf("Temp=%dC hal=%d\n",(int)tc, hall_sens_read());
  //  Serial.printf("Temp=%dC hal=%d\n",(int)0, 1);
    //temp1 = false;
    
  //}

};
