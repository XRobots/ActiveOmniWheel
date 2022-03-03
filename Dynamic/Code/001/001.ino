//ODrive
#include <ODriveArduino.h>
//IMU stuff
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

//PID
#include <PID_v1.h>

// Radio
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 10); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};

// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) {
  obj.print(arg);
  return obj;
}
template<>        inline Print& operator <<(Print &obj, float arg) {
  obj.print(arg, 4);
  return obj;
}

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

int IMUdataReady = 0;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

float pitch;
float roll;

//ODrive Objects
ODriveArduino odrive1(Serial1);

//**************remote control****************
struct RECEIVE_DATA_STRUCTURE {
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

float RLR = 0;
float RFB = 0;
float RT = 0;
float LLR = 0;
float LFB = 0;
float LT = 0;

float RLRFiltered = 0;
float RFBFiltered = 0;
float RTFiltered = 0;
float LLRFiltered = 0;
float LFBFiltered = 0;
float LTFiltered = 0;

int remoteState = 0;
int requested_state = 0;

float pot1;
float pot2;
float sw1;

float output1;
float output2;

float output3;
float output4;


// PID

double Pk1 = 0.3; 
double Ik1 = 2;
double Dk1 = 0.011;

double Pk2 = 2.25;  
double Ik2 = 17;
double Dk2 = 0.024;

double Setpoint1, Input1, Output1;    // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup

double Setpoint2, Input2, Output2;    // PID variables
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);    // PID Setup

unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long interval = 10;        // time constant for timer
unsigned long count;

unsigned long remoteMillis;


// ****************** SETUP ******************************

void setup() {

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

  pinMode(2, INPUT_PULLUP);

  // initialize serial communication
  Serial.begin(115200);
  Serial1.begin(115200);

  radio.begin();
  radio.openWritingPipe(addresses[0]); // 00002
  radio.openReadingPipe(1, addresses[1]); // 00001
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();


  // IMU Setup
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize device
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(116);
  mpu.setYGyroOffset(-32);
  mpu.setZGyroOffset(9);
  mpu.setXAccelOffset(-1356);
  mpu.setYAccelOffset(-478);
  mpu.setZAccelOffset(1134);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(33, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  PID1.SetMode(AUTOMATIC);              
  PID1.SetOutputLimits(-20, 20);
  PID1.SetSampleTime(10);

  PID2.SetMode(AUTOMATIC);              
  PID2.SetOutputLimits(-50, 50);
  PID2.SetSampleTime(10);

}   // end of setup

// ********************* MAIN LOOP *******************************

void loop() {

  currentMillis = millis();
  if (currentMillis - previousMillis >= 10) {  // start timed event

    previousMillis = currentMillis;

    // check for IMU inpterrupt, read the data if it's ready
    if (IMUdataReady == 1) {
      readAngles();      
    } 

    // check for radio data
    if (radio.available()) {
      radio.read(&mydata_remote, sizeof(RECEIVE_DATA_STRUCTURE));
      remoteMillis = currentMillis;
    }

    // is the remote disconnected for too long ?
    if (currentMillis - remoteMillis > 500) {
      remoteState = 0;
      Serial.println("no data");
    }
    else {
      remoteState = 1;      
    }

    // threshold remote data
    // some are reversed based on stick wiring in remote
    RFB = thresholdStick(mydata_remote.RFB);
    RLR = thresholdStick(mydata_remote.RLR);
    RT = thresholdStick(mydata_remote.RT);
    LFB = thresholdStick(mydata_remote.LFB);
    LLR = thresholdStick(mydata_remote.LLR);
    LT = thresholdStick(mydata_remote.LT);

    // get IMU values
    roll = (ypr[1] * 180 / M_PI);
    pitch = (ypr[2] * 180 / M_PI);

    // get switches and pots
    pot1 = analogRead(A0);
    pot1 = (pot1-512) / 200;
    pot2 = analogRead(A1);
    pot2 = (pot2-512) / 200;
    sw1 = digitalRead(2);

    if (sw1 == 0) {
      OdriveInit1();
    }

    
    RFB = RFB / 50;  // convert to degrees
    RLR = RLR / 100;  // convert to degrees

    Input1 = pitch;
    Setpoint1 = pot1 + RFB;
    PID1.Compute();

    Input2 = roll*-1;
    Setpoint2 = pot2 + RLR;
    PID2.Compute();

    output1 = Output1;
    output2 = Output2*-1;
    //output1 = 0;

    if (mydata_remote.toggleTop == 0) {
        odrive1.SetVelocity(0, 0);
        odrive1.SetVelocity(1, 0);
        roll = 0;
        pitch = 0;
        Input1 = 0;
        Input2 = 0;
    }

    else {
        odrive1.SetVelocity(0, output2+(output1*-1));  // differential drive
        odrive1.SetVelocity(1, output1);
    }


   }     // end of timed loop

}       // end  of main loop








