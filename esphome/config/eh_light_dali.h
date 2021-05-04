#include "esphome.h"


const int DALI_TX = 33;    // ADC1      (no use 0,2,15,...)
const int DALI_RX_A = 32;  // ESP only ADC pin


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


#define TERMINATE 		0xA1	//0b10100001
#define INITIALISE 		0xA5
#define RANDOMISE		0xA7

#define COMPARE 		0xA9	//0b10101001
#define WITHDRAW 		0xAB

#define SEARCH_ADDRH 	0xB1
#define SEARCH_ADDRM 	0xB3
#define SEARCH_ADDRL 	0xB5

#define PRG_SHORT_ADDR 	0xB7


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

    Dali() //constructor
    {
        applyWorkAround1Mhz = 0;
    }

    void setTxPin(uint8_t pin)
    {
        TxPin = pin; // user sets the digital pin as output
        pinMode(TxPin, OUTPUT);
        digitalWrite(TxPin, HIGH);
    }

    void setRxAnalogPin(uint8_t pin)
    {
        RxAnalogPin = pin; // user sets the digital pin as output
    }

    void workAround1MhzTinyCore(uint8_t a)
    {
        applyWorkAround1Mhz = a;
    }

    void setupAnalogReceive(uint8_t pin)
    {
        setRxAnalogPin(pin); // user sets the analog pin as input
    }

    void setupTransmit(uint8_t pin)
    {
        setTxPin(pin);
        speedFactor = 2;
        //we don't use exact calculation of passed time spent outside of transmitter
        //because of high ovehead associated with it, instead we use this
        //emprirically determined values to compensate for the time loss

    #if F_CPU == 1000000UL
        uint16_t compensationFactor = 88; //must be divisible by 8 for workaround
    #elif F_CPU == 8000000UL
        uint16_t compensationFactor = 12;
    #else //16000000Mhz
        uint16_t compensationFactor = 4;
    #endif

    #if (F_CPU == 80000000UL) || (F_CPU == 160000000) // ESP8266 80MHz or 160 MHz
        delay1 = delay2 = (HALF_BIT_INTERVAL >> speedFactor) - 2;
    #else
        delay1 = (HALF_BIT_INTERVAL >> speedFactor) - compensationFactor;
        delay2 = (HALF_BIT_INTERVAL >> speedFactor) - 2;
        period = delay1 + delay2;

    #if F_CPU == 1000000UL
        delay2 -= 22; //22+2 = 24 is divisible by 8
        if (applyWorkAround1Mhz)
        { //definition of micro delay is broken for 1MHz speed in tiny cores as of now (May 2013)
            //this is a workaround that will allow us to transmit on 1Mhz
            //divide the wait time by 8
            delay1 >>= 3;
            delay2 >>= 3;
        }
    #endif
    #endif
    }


    void LightCmd(uint8_t device_add, uint8_t cmd2){
        uint8_t add_byte;
        add_byte = 1 + (device_add << 1); // convert short address to address byte
        transmit(add_byte, cmd2);
    }


    void LightLevel(uint8_t device_add, uint8_t cmd2){
        uint8_t add_byte;
        add_byte = (device_add << 1); //
        transmit(add_byte, cmd2);
    }


    void transmit(uint8_t cmd1, uint8_t cmd2) // transmit commands to DALI bus (address byte, command byte)
    {
        sendBit(1);
        sendByte(cmd1);
        sendByte(cmd2);
        digitalWrite(TxPin, HIGH);
    }


    void busTest() //DALI bus test
    {
        int maxLevel;
        int minLevel;

        //Luminaries must turn on and turn off. If not, check connection.
        delay(100);
        this->transmit(BROADCAST_C, OFF_C); //Broadcast ON
        delay(500);
        this->transmit(BROADCAST_C, ON_C); //Broadcast OFF
        delay(100);
        //while (!Serial);

        //Receive response from luminaries: max and min level
        this->transmit(BROADCAST_C, QUERY_STATUS);
        maxLevel = this->maxResponseLevel();
        this->transmit(BROADCAST_C, QUERY_STATUS);
        minLevel = this->minResponseLevel();

        this->maxLevel = maxLevel;
        this->minLevel = minLevel;
        this->analogLevel = (int)(maxLevel + minLevel) / 2;
    }

    // define min response level
    int minResponseLevel()
    {
        const uint8_t dalistep = 40; //us
        uint16_t rxmin = 1024;
        uint16_t dalidata;
        long idalistep;

        for (idalistep = 0; idalistep < this->daliTimeout; idalistep = idalistep + dalistep)
        {
            dalidata = analogRead(RxAnalogPin);
            if (dalidata < rxmin)
            {
                rxmin = dalidata;
            };
            delayMicroseconds(dalistep);
        }
        return rxmin;
    }

    // define max response level
    int maxResponseLevel()
    {
        const uint8_t dalistep = 40; //us
        uint16_t rxmax = 0;
        uint16_t dalidata;
        long idalistep;

        for (idalistep = 0; idalistep < this->daliTimeout; idalistep = idalistep + dalistep)
        {
            dalidata = analogRead(RxAnalogPin);
            if (dalidata > rxmax)
            {
                rxmax = dalidata;
            };
            delayMicroseconds(dalistep);
        }
        return rxmax;
    }

    //scan for individual short address
    void scanShortAdd()
    {
        const int delayTime = 10;
        const uint8_t start_ind_adress = 0;
        uint8_t add_byte;
        uint8_t device_short_add;
        uint8_t response;

        this->transmit(BROADCAST_C, OFF_C); // Broadcast Off
        delay(delayTime);

        if (this->msgMode)
        {
            Serial.println("Short addresses:");
        }

        for (device_short_add = start_ind_adress; device_short_add <= 63; device_short_add++)
        {

            this->LightCmd(device_short_add, QUERY_MAX_LEVEL);

            response = this->receive();
            if (this->getResponse)
            {
                this->transmit(add_byte, ON_C); // switch on
                delay(1000);
                this->transmit(add_byte, OFF_C); // switch off
                delay(1000);
            }
            else
            {
                response = 0;
            }

            if (this->msgMode)
            {
                //Serial.print("BIN: ");
                //Serial.print(device_short_add, BIN);
                Serial.print(" ");
                Serial.print("DEC: ");
                Serial.print(device_short_add, DEC);
                Serial.print(" ");
                Serial.print("HEX: ");
                Serial.print(device_short_add, HEX);
                //Serial.print(" A: ");
                //Serial.print(add_byte, DEC);
                Serial.print(" ");
                if (this->getResponse)
                {
                    Serial.print("Get response ");
                    Serial.print(response, HEX);
                }
                else
                {
                    Serial.print("No response");
                }
                Serial.println();
            }
            else
            {
                if (this->getResponse)
                {
                    Serial.println(255, BIN);
                }
                else
                {
                    Serial.println(0, BIN);
                }
            }
        }

        this->transmit(BROADCAST_C, ON_C); // Broadcast On
        Serial.println();
        delay(delayTime);
    }

    bool cmdCheck(String &input, int &cmd1, int &cmd2)
    {
        bool test = true;

        input.replace(" ", ""); // Delete spaces

        if (input.length() != 16)
        {
            test = false; //check if command contain 16bit
        }
        else
        {
            for (int i = 0; i <= input.length() - 1; i++)
            {
                if ((int)input.charAt(i) == 49 or (int) input.charAt(i) == 48)
                {
                }
                else
                {
                    test = false;
                };
            };
        };

        if (test)
        {
            //cmd1 = readBinaryString(input.substring(0, 8).c_str());
            //cmd2 = readBinaryString(input.substring(8, 16).c_str());

            char *S1 = new char[input.substring(0, 8).length() + 1];
            strcpy(S1, input.substring(0, 8).c_str());
            cmd1 = readBinaryString(S1);

            char *S2 = new char[input.substring(8, 16).length() + 1];
            strcpy(S2, input.substring(0, 8).c_str());
            cmd2 = readBinaryString(S2);
        }

        return test;
    }

    void initialisation()
    {
        bool Response = false;
        const int delaytime = 10; //ms

        long low_longadd = 0x000000;
        long high_longadd = 0xFFFFFF;
        long longadd = (long)(low_longadd + high_longadd) / 2;
        uint8_t highbyte;
        uint8_t middlebyte;
        uint8_t lowbyte;
        uint8_t short_add = 0;
        uint8_t cmd2;

        delay(delaytime);
        //dali.transmit(BROADCAST_C, RESET);
        this->transmit(RESET, 0x00);
        delay(2*delaytime);
        //dali.transmit(BROADCAST_C, RESET);
        this->transmit(RESET, 0x00);
        delay(2*delaytime);
        delay(100);
        
        this->transmit(BROADCAST_C, OFF_C);
        delay(delaytime);

        this->transmit(INITIALISE, 0x00); 	//initialise
        delay(delaytime);
        this->transmit(INITIALISE, 0x00); 	//
        delay(delaytime);
        this->transmit(INITIALISE, 0x00); 	//
        delay(delaytime);
        delay(100);

        this->transmit(RANDOMISE, 0x00); 	//randomise
        delay(delaytime);
        this->transmit(RANDOMISE, 0x00); 	//
        delay(delaytime);
        delay(100);

        if (this->msgMode)
        {
            Serial.println("Searching fo long addresses:");
        }

        while (longadd <= 0xFFFFFF - 2 and short_add <= 64)
        {
            while ((high_longadd - low_longadd) > 1)
            {

                this->splitAdd(longadd, highbyte, middlebyte, lowbyte); //divide 24bit adress into three 8bit adresses
                delay(delaytime);
                this->transmit(SEARCH_ADDRH, highbyte); 		//search HB
                delay(delaytime);
                this->transmit(SEARCH_ADDRM, middlebyte); 	//search MB
                delay(delaytime);
                this->transmit(SEARCH_ADDRL, lowbyte); 		//search LB
                delay(delaytime);
                this->transmit(COMPARE, 0x00); 	//compare

                Response = false;
                delayMicroseconds(7 * DALI_HALF_BIT_TIME);
                for(uint8_t i = 0; i < DALI_RESPONSE_DELAY_COUNT; i++)
                {
                    if (analogRead(RxAnalogPin) < this->analogLevel)
                    {
                    Response = true;
                    break;
                    }
                    delayMicroseconds(DALI_HALF_BIT_TIME);
                }

                if (!Response)//(minResponseLevel() > dali.analogLevel)
                {
                    low_longadd = longadd;
                }
                else
                {
                    high_longadd = longadd;
                }

                longadd = (low_longadd + high_longadd) / 2; //center

                if (this->msgMode)
                {
                    //Serial.print("BIN: ");
                    //Serial.print(longadd + 1, BIN);
                    //Serial.print(" ");
                    Serial.print("DEC: ");
                    Serial.print(longadd + 1, DEC);
                    Serial.print(" ");
                    Serial.print("HEX: ");
                    Serial.print(longadd + 1, HEX);
                    Serial.println();
                }
                else
                {
                    Serial.println(longadd + 1);
                }
            } // second while

            if (high_longadd != 0xFFFFFF)
            {
                splitAdd(longadd + 1, highbyte, middlebyte, lowbyte);

                this->transmit(SEARCH_ADDRH, highbyte); 		//search HB
                delay(delaytime);
                this->transmit(SEARCH_ADDRM, middlebyte); 	//search MB
                delay(delaytime);
                this->transmit(SEARCH_ADDRL, lowbyte); 		//search LB
                delay(delaytime);
                this->transmit(PRG_SHORT_ADDR, 1 + (short_add << 1)); //program short adress
                delay(delaytime);
                this->transmit(WITHDRAW, 0x00); 		//withdraw
                delay(delaytime);
                
                this->transmit(1 + (short_add << 1), ON_C);
                delay(1000);
                this->transmit(1 + (short_add << 1), OFF_C);
                delay(delaytime);

                short_add++;

                if (this->msgMode)
                {
                    Serial.println("Assigning a short address");
                }

                high_longadd = 0xFFFFFF;
                longadd = (low_longadd + high_longadd) / 2;
            }
            else
            {
                if (this->msgMode)
                {
                    Serial.println("End");
                }
            }
        } // first while

        this->transmit(TERMINATE, 0x00); 		//terminate
        //dali.transmit(BROADCAST_C, ON_C);	    //broadcast on
    }

    uint8_t receive()
{

	unsigned long startFuncTime = 0;
	bool previousLogicLevel = 1;
	bool currentLogicLevel = 1;
	uint8_t arrLength = 20;
	int timeArray[arrLength];
	int i = 0;
	int k = 0;
	bool logicLevelArray[arrLength];
	int response = 0;

	this->getResponse = false;
	startFuncTime = micros();

	// add check for micros overlap here!!!
	while (micros() - startFuncTime < this->daliTimeout and i < arrLength)
	{
		// geting response
		if (analogRead(RxAnalogPin) > this->analogLevel)
		{
			currentLogicLevel = 1;
		}
		else
		{
			currentLogicLevel = 0;
		}

		if (previousLogicLevel != currentLogicLevel)
		{
			timeArray[i] = micros() - startFuncTime;
			logicLevelArray[i] = currentLogicLevel;
			previousLogicLevel = currentLogicLevel;
			this->getResponse = true;
			i++;
		}
	}

	/*
	Serial.print("I:");
	Serial.print(i, DEC);
	//Serial.println("");
	//Serial.print("I:");
	//Serial.print(startFuncTime, DEC);
	Serial.println("");
	*/

	arrLength = i;

	//decoding to manchester
	for (i = 0; i < arrLength - 1; i++)
	{
		if ((timeArray[i + 1] - timeArray[i]) > 0.75 * this->period)
		{
			for (k = arrLength; k > i; k--)
			{ //asv
				//Serial.print(k, DEC);
				//Serial.print(" ");
				//Serial.println(arrLength, DEC);

				timeArray[k] = timeArray[k - 1];
				logicLevelArray[k] = logicLevelArray[k - 1];
			}
			arrLength++;
			timeArray[i + 1] = (timeArray[i] + timeArray[i + 2]) / 2;
			logicLevelArray[i + 1] = logicLevelArray[i];
		}
	}

	k = 8;

	for (i = 1; i < arrLength; i++)
	{
		if (logicLevelArray[i] == 1)
		{
			if ((int)round((timeArray[i] - timeArray[0]) / (0.5 * this->period)) & 1)
			{
				response = response + (1 << k);
			}
			k--;
		}
	}
	/*
  Serial.println("");
  Serial.print("R:");
  Serial.print(response, BIN);
  Serial.println("");
	
	//remove start bit
	response = (uint8_t)response;

  Serial.print("RI:");
  Serial.print(response, DEC);
  Serial.println("");
  */

	return response;
    }
private:

    uint8_t TxPin;

    uint8_t applyWorkAround1Mhz;
    uint8_t rxAnalogPin = 0;


    void sendByte(uint8_t b)
    {
        for (int i = 7; i >= 0; i--){
            sendBit((b >> i) & 1);
        }
    }

    void sendBit(int b)
    {
        if (b)	{sendOne();	}
        else	{sendZero();}
    }

    void sendZero(void)
    {
        digitalWrite(TxPin, HIGH);
        delayMicroseconds(delay2);		// maybe DALI_HALF_BIT_TIME
        digitalWrite(TxPin, LOW);
        delayMicroseconds(delay1);
    }

    void sendOne(void)
    {
        digitalWrite(TxPin, LOW);
        delayMicroseconds(delay2);
        digitalWrite(TxPin, HIGH);		// maybe DALI_HALF_BIT_TIME
        delayMicroseconds(delay1);
    }

    void splitAdd(long input, uint8_t &highbyte, uint8_t &middlebyte, uint8_t &lowbyte)
    {
        highbyte = input >> 16;
        middlebyte = input >> 8;
        lowbyte = input;
    }

    int readBinaryString(char *s)
    {
        int result = 0;
        while (*s)
        {
            result <<= 1;
            if (*s++ == '1')
                result |= 1;
        }
        return result;
    }

}; //end of class Dali

//Dali d = new Dali();
//extern Dali d = new Dali();

class DaliLightOutput : public Component, public LightOutput {
 public:
  void setup() override {
    Dali dali;

    dali.setupTransmit(DALI_TX);
    dali.setupAnalogReceive(DALI_RX_A);
    dali.busTest();
    dali.msgMode = true;
  }

  LightTraits get_traits() override {
    // return the traits this light supports
    auto traits = LightTraits();
    traits.set_supports_brightness(true);
    traits.set_supports_rgb(false);
    traits.set_supports_rgb_white_value(false);
    traits.set_supports_color_temperature(false);
    return traits;
  }

  void write_state(LightState *state) override {
    //float red, green, blue;
    //state->current_values_as_rgb(&red, &green, &blue);
    Dali dali;

    bool binary;
    state->current_values_as_binary(&binary);
    if (binary){
        dali.transmit(BROADCAST_C, ON_C);
       //this->output_->turn_on();
    }else{
         dali.transmit(BROADCAST_C, OFF_C);
       //this->output_->turn_off();
    }

    float brightnessPercent;
    state->current_values_as_brightness(&brightnessPercent);

    // Convert to 0-100
    int brightness = floor(brightnessPercent * 100);

  }
};