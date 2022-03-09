/*
  RC Inspection Tank

  - Flysky FS-I6X RC Transmitter loaded with OpenTX firmware
  - Fli14+ 14CH Mini Receiver 2A receiver

  - TB6612FNG H-Bridge Motor Controller to drive two DC Motors

  - Adafruit PWM 16 Ch servo driver
  VBA controls tilt
  VBB controls pan

  - Head tracker (https://headtracker.gitbook.io/head-tracker/) connected to the FlySky
  TRCH6 tilt
  TRCH7 pan

  Right stick controls Forward and backward direction (CH1) and turning (CH2).
  SWD controls the camera pan and tilt; 
    - Off has it facing forward
    - On allows two modes based on SWA:
       - SWA off control via VBA and VBB
       - SWA on - controls via haedtracker

  Based on ... 
    fsi6x-RC-car-spin.ino at https://dronebotworkshop.com/radio-control-arduino-car/
    Channel functions by Ricardo Paiva - https://gist.github.com/werneckpaiva/
    DroneBot Workshop 2021
*/

// Include iBus Library
#include <IBusBM.h>

// Printf
#include <LibPrintf.h>

// Create iBus Object
IBusBM ibus;

// AdaFruit PWM lib
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!

#define SERVOMIN    150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX    450 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN       600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX      2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ   50 // Analog servos run at ~50 Hz updates

// Define the FS-i6X Channel usage

#define LEFTRIGHT  0
#define FWDBACK    1
#define LEDLIGHT   2
#define VARPOTA    4
#define VARPOTB    5

#define SWA        6
#define SWB        7
#define SWC        8
#define SWD        9

#define headTrackPan  10
#define headTrackTilt 11
#define headTrackRoll 12

String motorDir = "unk";

int turnDirection = 0; // Left - Right
int motorSpeed = 0; // Forward - Reverse

int headLights = 0; // LED lights

int panVal = 0; // camera pan
int tiltVal = 0; // camera tilt

int rcHTpan  = 0; // TR camera pan
int rcHTtilt = 0; // TR camera tilt


bool  CamSwitch = 0;  // camera move - knobs(0) or head track(1)
bool  motorsOn = 0;  
int   driveMode = 0; 
bool  CamFront = 0;  // Camera face - front (0) or move (1)

float panVal_smooth = 0;
float panVal_prev = 0;
float tiltVal_smooth = 0;
float tiltVal_prev = 0;

#define panCenter  400
#define tiltCenter 320

// PWM assignments
#define panPort 0
#define tiltPort 1
#define lightPort 2

// TB6612FNG motor control chip

// Motor A Control Connections
#define pwmA 3
#define in1A 5
#define in2A 2

// Motor B Control Connections
#define pwmB 9
#define in1B 7
#define in2B 6

// TB6612FNG Standby Pin
#define stby 4

// Motor Speed Values - Start at zero
int MotorSpeedA = 0;
int MotorSpeedB = 0;

// Motor Direction Values - 0 = backward, 1 = forward
int MotorDirA = 1;
int MotorDirB = 1;

// Control Motor A
void mControlA(int mspeed, int mdir) {

  // Determine direction
  if (mdir == 0) {
    // Motor backward
    digitalWrite(in1A, LOW);
    digitalWrite(in2A, HIGH);
  } else {
    // Motor forward
    digitalWrite(in1A, HIGH);
    digitalWrite(in2A, LOW);
  }

  // Control motor
  analogWrite(pwmA, mspeed);

}

// Control Motor B
void mControlB(int mspeed, int mdir) {

  // Determine direction
  if (mdir == 0) {
    // Motor backward
    digitalWrite(in1B, LOW);
    digitalWrite(in2B, HIGH);
  } else {
    // Motor forward
    digitalWrite(in1B, HIGH);
    digitalWrite(in2B, LOW);
  }

  // Control motor
  analogWrite(pwmB, mspeed);

}

// Read the number of a given channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = ibus.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

// Read the channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue) {
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}

void setup()
{

  // Start serial monitor for debugging
  Serial.begin(115200);

  // Attach iBus object to serial port
  ibus.begin(Serial1);

  // Set all the motor control pins to outputs

  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(in1A, OUTPUT);
  pinMode(in2A, OUTPUT);
  pinMode(in1B, OUTPUT);
  pinMode(in2B, OUTPUT);
  pinMode(stby, OUTPUT);

  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  // center the camera

  pwm.setPWM(panPort,  100, panCenter );   // pan
  pwm.setPWM(tiltPort, 100, tiltCenter );  // tilt
  delay(100);

  // wiggle it a bit to show power up
  #define shake_range 50
  #define shake_rate   5
  #define shake_delay 10
  
  Serial.println("Shake the camera to say hello.");
  delay(100);
  
  for(int i = panCenter;i<=panCenter + shake_range;i = i + shake_rate){
    pwm.setPWM(panPort, 100, i );
    //Serial.println(i);
    delay(shake_delay + shake_rate);
  }
  for(int i = panCenter + shake_range;i>=panCenter - shake_range;i = i - shake_rate){
    pwm.setPWM(panPort, 100, i );
    //Serial.println(i);
    delay(shake_delay + shake_rate);
  }
    for(int i = panCenter - shake_range;i<=panCenter;i = i + shake_rate){
    pwm.setPWM(panPort, 100, i );
    //Serial.println(i);
    delay(shake_delay + shake_rate);
  }
  pwm.setPWM(lightPort, 100, 0);
  
  // Keep motors on standby for two seconds
  digitalWrite(stby, LOW);
  delay (2000);
  digitalWrite(stby, HIGH);

}

// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
//  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
//  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
//  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void loop() {

  // Get RC channel values
  turnDirection = readChannel(LEFTRIGHT, -255, 255, 0);
  motorSpeed    = readChannel(FWDBACK,   -255, 255, 0);
  headLights    = readChannel(LEDLIGHT,  0, 1000, 0);
  
  
  CamSwitch = readSwitch(SWA,false);
  motorsOn  = readSwitch(SWB,false);
  driveMode = readChannel(SWC,-512,512,0); // Three way...  <0 ; 0 ; >0
  CamFront  = readSwitch(SWD,false);
  
  if(driveMode<0){
    if(CamFront){
       if(CamSwitch){ // head track
          panVal  = readChannel(headTrackPan,  -512, 512, 0) /2 ;
          tiltVal = readChannel(headTrackTilt, -512, 512, 0) /2 ; 
          
       } else { // knobs
          panVal  = readChannel(VARPOTA, -512, 512, 0) / 3;
          tiltVal = readChannel(VARPOTB, -512, 512, 0) / 3;
          
       }
       panVal_smooth  = ((panVal  + panCenter ) *.15) + (panVal_prev  * .85);
       tiltVal_smooth = ((tiltVal + tiltCenter) *.15) + (tiltVal_prev * .85);
    } else { // Face front
       panVal         = panCenter;
       tiltVal        = tiltCenter;
       panVal_smooth  = panVal;
       tiltVal_smooth = tiltVal;
    }
  } else {
       panVal         = panCenter;
       tiltVal        = tiltCenter;
       panVal_smooth  = panVal;
       tiltVal_smooth = tiltVal;
       turnDirection  = -1 * readChannel(headTrackPan,  -512, 512, 0) / 4 ;
  }
  
  //printf("Pan:%5i Tilt:%5i, CamFront: %i \n", int(panVal), int(tiltVal), CamFront);
  //printf("Pan:%5i Tilt:%5i \n", int(panVal_smooth), int(tiltVal_smooth));
    
  panVal_prev = panVal_smooth;
  tiltVal_prev = tiltVal_smooth;

  // Set left/right offset with channel 1 value
  MotorSpeedA = motorSpeed - turnDirection;
  MotorSpeedB = motorSpeed + turnDirection;

  // Set forward/backward direction
  if (MotorSpeedA < 0) {
    //Forward
    MotorDirA = 0;
    MotorSpeedA = abs(MotorSpeedA);
  } else {
    MotorDirA = 1;
  }
  
  if (MotorSpeedB < 0) {
    MotorDirB = 0; 
    MotorSpeedB = abs(MotorSpeedB);
  } else {
    MotorDirB = 1;
  }

  switch(MotorDirA*10+MotorDirB) {
    case 0:
       motorDir="Rev";
       break;
    case 1:
       motorDir="RT";
       break;
    case 10:
       motorDir="LT";
       break;
    case 11:
       motorDir="Fwd";
       break;
  }

  // ignore near zero values
  if ((MotorSpeedA + MotorSpeedB <= 4) || (motorsOn == 0)) {
     motorDir = "Stp";
     MotorSpeedA = 0;
     MotorSpeedB = 0;
  }

  // Ensure that speeds are between 0 and 255
  MotorSpeedA = constrain(MotorSpeedA, 0, 255);
  MotorSpeedB = constrain(MotorSpeedB, 0, 255);
  
  //Drive Motors
  mControlA(MotorSpeedA, MotorDirA);
  mControlB(MotorSpeedB, MotorDirB);

  // Camera Control
  pwm.setPWM(panPort,   100, int(panVal_smooth)  ); //pan
  pwm.setPWM(tiltPort,  100, int(tiltVal_smooth) ); //tilt

  pwm.setPWM(lightPort,   5, int(headLights)     );

  // Print values to serial monitor for debugging
  printf("1:%3i 2:%3i 5:%3.0f 6:%3.0f A:%o B:%o MA:%3i MB:%3i motorDir:" \
  ,turnDirection, motorSpeed, panVal_smooth, tiltVal_smooth, CamSwitch, CamFront, MotorSpeedA, MotorSpeedB);
  //printf("%s\n", motorDir);
   Serial.println(motorDir);
 
  // Slight delay
  // delay(50);

}
