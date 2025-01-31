

#ifndef dali_h
#define dali_h
//#include <SoftwareSerial.h>

//timer scaling factors for different transmission speeds
#define MAN_300 0
#define MAN_600 1
#define MAN_1200 2
#define MAN_2400 3
#define MAN_4800 4
#define MAN_9600 5
#define MAN_19200 6
#define MAN_38400 7

/*
Timer 2 in the ATMega328 and Timer 1 in a ATtiny85 is used to find the time between
each transition coming from the demodulation circuit.
Their setup is for sampling the input in regular intervals.
For practical reasons we use power of 2 timer prescaller for sampling, 
for best timing we use pulse lenght as integer multiple of sampling speed.
We chose to sample every 8 ticks, and pulse lenght of 48 ticks 
thats 6 samples per pulse, lower sampling rate (3) will not work well for 
innacurate clocks (like internal oscilator) higher sampling rate (12) will
cause too much overhead and will not work at higher transmission speeds.
This gives us 16000000Hz/48/256 = 1302 pulses per second (so it's not really 1200) 
At different transmission speeds or on different microcontroller frequencies, clock prescaller is adjusted 
to be compatible with those values. We allow about 50% clock speed difference both ways
allowing us to transmit even with up to 100% in clock speed difference
*/

// DALI coomands
#define BROADCAST_DP  0b11111110
#define BROADCAST_C   0b11111111
#define ON_DP         0b11111110
#define OFF_DP        0b00000000
#define ON_C          0b00000101
#define OFF_C         0b00000000
#define QUERY_STATUS  0b10010000	// 90	 Reply bits: 0=controlGearFailure; 1=lampFailure; 2=lampOn;
									//					 3=limitError; 4=fadeRunning; 5=resetState; 
									//					 6=shortAddress is MASK; 7=powerCycleSeen
#define QUERY_ON   	  0x93			// 93
#define QUERY_LEVEL   0b10100000	// A0
#define QUERY_MAX_LEVEL   0xA1		// 
#define QUERY_MIN_LEVEL   0xA2		// 
#define QUERY_POW_LEVEL   0xA3		// 
#define RESET         0b00100000	// 20

#define STEP_UP		  0b00000011	// 03
#define STEP_DOWN	  0b00000100	// 04

#define SET_MAX_LEVEL 0b00101010	// 2A
#define SET_MIN_LEVEL 0b00101011	// 2B

//setup timing for transmitter
#define HALF_BIT_INTERVAL 1666

// chipdip
#define DALI_HALF_BIT_TIME 416		 //microseconds
#define DALI_TWO_PACKET_DELAY 10	 //miliseconds
#define DALI_RESPONSE_DELAY_COUNT 15 //максимальное число полубитов


#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
//#include "WProgram.h"
#include "WString.h"
#include <pins_arduino.h>
#include <esp32-hal.h>
#include <HardwareSerial.h>
#endif

class Dali
{
public:
	Dali();										//the constructor
	void setTxPin(uint8_t pin);					//set the arduino digital pin for transmit.
	void setRxAnalogPin(uint8_t pin);			//set the arduino digital pin for receive.
	void workAround1MhzTinyCore(uint8_t a = 1); //apply workaround for defect in tiny Core library for 1Mhz
	void setupTransmit(uint8_t pin);			//set up transmission
	void setupAnalogReceive(uint8_t pin);
	void LightCmd(uint8_t device_add, uint8_t cmd2);	   		//
	void LightLevel(uint8_t device_add, uint8_t cmd2);	    	//
	void transmit(uint8_t cmd1, uint8_t cmd2);	   //transmit 16 bits of data
	//void transmit_old(uint8_t cmd1, uint8_t cmd2); //
	void scanShortAdd();						   //scan for short address
	void busTest();								   // bus test
	void initialisation();						   //initialization of new luminaries
	bool cmdCheck(String &input, int &cmd1, int &cmd2);
	uint8_t receive(); //get response

	int minResponseLevel();
	int maxResponseLevel();

	uint8_t speedFactor;
	uint16_t delay1;
	uint16_t delay2;
	uint16_t period;
	String errorMsg; //error message of last operation
	bool msgMode;	 //0 - get only response from dali bus to COM; 1 - response with text (comments)
	bool getResponse;
	uint8_t RxAnalogPin;

	long daliTimeout = 20000; //us, DALI response timeout
	int analogLevel = 870;	  //analog border level (less - "0"; more - "1")
	int maxLevel = 0;
	int minLevel = 0;
	

private:
	void sendByte(uint8_t b);															 //transmit 8 bits of data
	void sendBit(int b);																 //transmit 1 bit of data
	void sendZero(void);																 //transmit "0"
	void sendOne(void);																	 //transmit "1"
	void splitAdd(long input, uint8_t &highbyte, uint8_t &middlebyte, uint8_t &lowbyte); //split random address

	int readBinaryString(char *s);

	uint8_t TxPin;

	uint8_t applyWorkAround1Mhz;
	uint8_t rxAnalogPin = 0;

}; //end of class Dali

// Cant really do this as a real C++ class, since we need to have
// an ISR
extern "C"
{
}

extern Dali dali;

#endif

/*
void DaliInit()
{
  Serial.println("Initialization...");

  DaliTransmitCMD(RESET, 0x00);
  delay(2*DALI_TWO_PACKET_DELAY);
  DaliTransmitCMD(RESET, 0x00);
  delay(2*DALI_TWO_PACKET_DELAY);
  delay(100);
    
  DaliTransmitCMD(INITIALISE, 0x00); 
  delay(DALI_TWO_PACKET_DELAY);
  DaliTransmitCMD(INITIALISE, 0x00);
  delay(DALI_TWO_PACKET_DELAY);
  DaliTransmitCMD(INITIALISE, 0x00);
  delay(DALI_TWO_PACKET_DELAY);
  delay(100);
  
  DaliTransmitCMD(RANDOMISE, 0x00);
  delay(DALI_TWO_PACKET_DELAY);
  DaliTransmitCMD(RANDOMISE, 0x00);
  delay(DALI_TWO_PACKET_DELAY);
  delay(100);

  while(ShortAddr < 64)
  {
    long SearchAddr = 0xFFFFFF;
    bool Response = 0;
    long LowLimit = 0;
    long HighLimit = 0x1000000;

    Response = SearchAndCompare(SearchAddr);
    delay(DALI_TWO_PACKET_DELAY);
  
    if(Response)
    {
      digitalWrite(LED_PIN, LOW);
      Serial.println("Device detected, address searching...");
      
      if(!SearchAndCompare(SearchAddr - 1))
      {
        delay(DALI_TWO_PACKET_DELAY);
        SearchAndCompare(SearchAddr);
        delay(DALI_TWO_PACKET_DELAY);
        DaliTransmitCMD(PRG_SHORT_ADDR, ((ShortAddr << 1) | 1));
        delay(3*DALI_TWO_PACKET_DELAY);
        DaliTransmitCMD(WITHDRAW, 0x00);
        Serial.print("24-bit address found: 0x");
        Serial.println(SearchAddr, HEX);
        Serial.print("Assigning short address ");
        Serial.println(ShortAddr);
        break;
      }
    }
    else
    {
      Serial.println("No devices detected");
      break;
    }

    while(1)
    {
      SearchAddr = (long)((LowLimit + HighLimit) / 2);

      Response = SearchAndCompare(SearchAddr);
      delay(DALI_TWO_PACKET_DELAY);

      if (Response)
      {
        digitalWrite(LED_PIN, LOW);

        if ((SearchAddr == 0) || (!SearchAndCompare(SearchAddr - 1)))
          break;
        
        HighLimit = SearchAddr;
      }
      else
        LowLimit = SearchAddr;
    }

    delay(DALI_TWO_PACKET_DELAY);
    SearchAndCompare(SearchAddr);
    delay(DALI_TWO_PACKET_DELAY);
    DaliTransmitCMD(PRG_SHORT_ADDR, ((ShortAddr << 1) | 1));
    delay(5*DALI_TWO_PACKET_DELAY);
    DaliTransmitCMD(WITHDRAW, 0x00);
    delay(DALI_TWO_PACKET_DELAY);
    
    Serial.print("24-bit address found: 0x");
    Serial.println(SearchAddr, HEX);
    Serial.print("Assigning short address ");
    Serial.println(ShortAddr);

    ShortAddr++;

   // break; //только для одного модуля
  }

  delay(DALI_TWO_PACKET_DELAY);
  DaliTransmitCMD(TERMINATE, 0x00);
  delay(DALI_TWO_PACKET_DELAY);
  Serial.println("Init complete");
}
//-------------------------------------------------
bool SearchAndCompare(long SearchAddr)
{
  bool Response = 0;
  
  uint8_t HighByte = SearchAddr >> 16;
  uint8_t MiddleByte = SearchAddr >> 8;
  uint8_t LowByte = SearchAddr;

  for(uint8_t i = 0; i < 3; i++)
  {
    DaliTransmitCMD(SEARCHADDRH, HighByte);
    delay(DALI_TWO_PACKET_DELAY);
    DaliTransmitCMD(SEARCHADDRM, MiddleByte);
    delay(DALI_TWO_PACKET_DELAY);
    DaliTransmitCMD(SEARCHADDRL, LowByte);
    delay(DALI_TWO_PACKET_DELAY);
  }
  DaliTransmitCMD(COMPARE, 0x00);
  delayMicroseconds(7 * DALI_HALF_BIT_TIME);
  
  for(uint8_t i = 0; i < DALI_RESPONSE_DELAY_COUNT; i++)
  {
    if (analogRead(DALI_RX_PIN) < DALI_ANALOG_LEVEL)
    {
      Response = 1;
      digitalWrite(LED_PIN, HIGH);
      break;
    }
    
    delayMicroseconds(DALI_HALF_BIT_TIME);
  }

  return Response;
}
*/