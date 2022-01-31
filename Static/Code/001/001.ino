int wheel1;
int wheel1a;
int wheel2;
int wheel2a;

// Radio
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};

//**************remote control****************
struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
    int16_t menuDown;  
    int16_t Select;    
    int16_t menuUp;  
    int16_t toggleBottom;  
    int16_t toggleTop; 
    int16_t toggle1;
    int16_t toggle2;
    int16_t mode;  
    int16_t RLR;
    int16_t RFB;
    int16_t RT;
    int16_t LLR;
    int16_t LFB;
    int16_t LT;
};

RECEIVE_DATA_STRUCTURE mydata_remote;

int RLR = 0;
int RFB = 0;
int RT = 0;
int LLR = 0;
int LFB = 0;
int LT = 0;

unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long interval = 10;        // time constant for timer

int forwards;
int backwards;
int left;
int right;
int CW;
int CCW;


int A2output;
int A3output;
int A4output;
int A5output;
int A10output;
int A11output;

void setup() {

    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
   
    // initialize serial communication
    Serial.begin(115200);
    
    radio.begin();
    radio.openWritingPipe(addresses[0]); // 00002
    radio.openReadingPipe(1, addresses[1]); // 00001
    radio.setPALevel(RF24_PA_MIN);
    radio.startListening();


    
}   // end of setup

// ********************* MAIN LOOP *******************************

void loop() {  
      
        currentMillis = millis();
        if (currentMillis - previousMillis >= 10) {  // start timed event
          
            previousMillis = currentMillis;


            // check for radio data
            if (radio.available()) {
                    radio.read(&mydata_remote, sizeof(RECEIVE_DATA_STRUCTURE));   
            } 

            else {
              Serial.println("no data");
            }

            // threshold remote data
            // some are reversed based on stick wiring in remote
            RFB = ((thresholdStick(mydata_remote.RFB)))/2;   
            RLR = (thresholdStick(mydata_remote.RLR))/2;
            LT = ((thresholdStick(mydata_remote.LT))*-1)/2; 

            Serial.print(mydata_remote.RFB);
            Serial.print(" , ");
            Serial.println(mydata_remote.RLR); 

            wheel1 = RFB - LT;
            wheel2 = RFB + LT;

            // wheel1

            if (wheel1 > 0) {
              wheel1 = constrain(wheel1,0,255);
              analogWrite(5, wheel1);
              analogWrite(6, 0);
            }
            else if (wheel1 < 0) {
              wheel1a = abs(wheel1);
              wheel1a = constrain(wheel1a,0,255);
              analogWrite(6, wheel1a);
              analogWrite(5, 0);
            }
            else {
              analogWrite(5, 0);
              analogWrite(6, 0);
            }

            // wheel2

            if (wheel2 > 0) {
              wheel2 = constrain(wheel2,0,255);
              analogWrite(10, wheel2);
              analogWrite(9, 0);              
            }
            else if (wheel2 < 0) {
              wheel2a = abs(wheel2);
              wheel2a = constrain(wheel2a,0,255);
              analogWrite(9, wheel2a);
              analogWrite(10, 0);
            }
            else {
              analogWrite(9, 0);
              analogWrite(10, 0);
            }    

      
        }     // end of timed loop         
   
}       // end  of main loop
