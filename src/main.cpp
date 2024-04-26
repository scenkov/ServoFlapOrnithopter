// Exporting the code for ESP32 board and SBUS receiver. Thanks to K.Kakuta for the work
// http://kakutaclinic.life.coocan.jp/SFOsys2S.html
// 221207 Version2 Rudder ElevatorUP FlapAmp-UP delaytime control PPMRX 
// Ch1-4 writeMicroseconds(1000-2000uS) use dt and elapsed time by K.Kakuta

#include <Arduino.h>
#include "ESP32Servo.h"
#include "SBUS.h"

//#define DOPRINTS
//#define GLIDING_CONTROL

#define RX_pin 16
#define TX_pin 17
// a SBUS object, which is on hardware
SBUS x8r(Serial1);

// channel, fail safe, and lost frames data
uint16_t channel[16];
bool failSafe;
bool lostFrame;

uint8_t FrameErrorRate = 0;
//int16_t channel[18] = {0};

int interruptPin = 2;
int channelAmount = 6;

//Servo servo_left, servo_right; // create servo object to control a servo
ESP32PWM pwm;
Servo servo_left;
Servo servo_right;
//Servo servo3;
//Servo servo4;
//Servo servo5;
// Published values for SG90 servos; adjust if needed
int minUs = 1000;
int maxUs = 2000;

int servo_left_pin = 12;
int servo_right_pin = 13;
volatile int elevator = 0;
volatile int flapamp = 0;
//volatile int delaytime = 100;// Servo speed low-increase Servo speed high-decrease this.

                          // unit: micro second
volatile float delaytime = 100;// Servo speed low-increase Servo speed high-decrease this.
                          // unit: micro second
float elapsed_time = 0;
float dt;
unsigned long current_time, prev_time;
volatile int ch3value = 1000;//Ch3
volatile int ch1value = 1500;//Ch1
volatile int ch2value = 1500;//Ch2
volatile int ch4value = 1500;//Ch4: Set Scale of 4ch of TX to 70%
volatile int ch5value = 1500;//Ch5
static int servo_comm1 = 0;// Left or Right Servo high point and low point
static int servo_comm2 = 0; // Left or Right Servo high point and low point
volatile int rudder = 0;
float glide_deg = 0; // Gliding angle 0=0 degree 500=90degree
static float servo_zero1 = 0;//flap angle adjust
static float servo_zero2 = 0; //flap angle adjust


void setup() {
  // Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	Serial.begin(115200);
	servo_left.setPeriodHertz(200);      // Standard 200hz servo
	servo_right.setPeriodHertz(200);      // Standard 200hz servo
//	servo3.setPeriodHertz(200);      // Standard 200hz servo
//	servo4.setPeriodHertz(200);      // Standard 200hz servo

  Serial.begin(115200);
//  Serial.println(__FILE__);
  x8r.begin(RX_pin, TX_pin, true, 100000); // optional parameters for ESP32: RX pin, TX pin, inverse mode, baudrate

  pinMode(servo_left_pin, OUTPUT);
  pinMode(servo_right_pin, OUTPUT);

  servo_left.setPeriodHertz(50);      // Standard 50hz servo
	servo_right.setPeriodHertz(50);      // Standard 50hz servo
  
  servo_left.attach(servo_left_pin, minUs, maxUs);
	servo_right.attach(servo_right_pin, minUs, maxUs);

  servo_left.writeMicroseconds(1500); // servo position in variable 'pos'
  servo_right.writeMicroseconds(1500); // servo position in variable 'pos'

  delay(2000);//Avoid abnormal positions at startup-wait 2 second until RX starts220306
}

void loop() {

  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0;

  elapsed_time = elapsed_time + dt; // total time spent in the main loop since beginning one upstroke/downstroke

  // look for a good SBUS packet from the receiver
  if(x8r.read(&channel[0], &failSafe, &lostFrame)){
    
    ch1value = channel[0];//Ch1
    ch2value = channel[1];//Ch2
    ch3value = channel[2];//Ch3
    ch4value = channel[3];//Ch4
    ch5value = channel[4];//Ch5

#ifdef GLIDING_CONTROL
    glide_deg = channel[5] - 1500;//Ch6 center-up
    if (glide_deg < 0) glide_deg = 0;
    if (glide_deg > 500) glide_deg = 500;
#endif
/*    
#ifdef DOPRINTS 
    for (int8_t i = 0; i < 5; i++) {
      Serial.print(channel[i]);
      Serial.print("\t");
    }
 //   Serial.println("");
#endif
*/
  }
  rudder=(int)(ch1value-1500);//Ch1  Flap angle incline-- AileronStick
  elevator=(int)(ch2value-1500);//Ch2 Flap Angle bilateral UP&Down
  flapamp=(int)(ch4value-1500);//Ch4 Right and left Flap angle difference from3to2
  delaytime=(int)((ch5value-950)/5);//Ch5 Flapping frequency 
  // you can change UP or Down direction by your transmitter Reverse setting of each Channel

#ifdef DOPRINTS
      //Serial.print("rudder");Serial.print(rudder);
      //Serial.print(",\t");
      //Serial.print("elevator");Serial.print(elevator);
      //Serial.print(",\t");
      //Serial.print("flapamp");Serial.print(flapamp);
      //Serial.print(",\t");
      Serial.print(" delaytime: ");Serial.print(delaytime);
      Serial.println(",\t");
#endif
  if (ch3value > 1080) {
    if (elapsed_time < delaytime/1000) {

      servo_comm1 = (int)( (ch3value -1000)/2 + 1500 + rudder - elevator + servo_zero1 + flapamp);
      servo_comm2 = (int)(1000 + (2000 - ((ch3value - 1000)/2 + 1500)) + rudder + elevator - servo_zero2 + flapamp);  
      
      servo_left.writeMicroseconds(servo_comm1); // servo position in variable 'pos'
      servo_right.writeMicroseconds(servo_comm2); // servo position in variable 'pos'
    }
    //Wait 1second=1000mseconds

    if ((elapsed_time > delaytime/1000) && ( elapsed_time < (delaytime + delaytime)/1000)) {
      servo_comm1 = (int)( (ch3value -1000)/2 + 1500 + rudder + elevator + servo_zero1 - flapamp);
      servo_comm2 = (int)(1000 + (2000 - ((ch3value -1000)/2 + 1500)) + rudder - elevator - servo_zero2 - flapamp);  

      servo_left.writeMicroseconds(servo_comm2); // servo position in variable 'pos'
      servo_right.writeMicroseconds(servo_comm1); // servo position in variable 'pos'

      //Serial.print("servo_comm1");Serial.print(servo_comm1);
      //Serial.print(",\t");
      //Serial.print("servo_comm2");Serial.print(servo_comm2);
      //Serial.println(",\t");
      
    //Wait 1second=1000mseconds
    }
  }
  else{
    servo_comm1=(int)(1500+rudder-elevator+glide_deg);
    servo_comm2=(int)(1500+rudder+elevator-glide_deg);  

    servo_left.writeMicroseconds(servo_comm1); // servo position in variable 'pos'
    servo_right.writeMicroseconds(servo_comm2); // servo position in variable 'pos' 

      //Serial.print("servo_comm1");Serial.print(servo_comm1);
      //Serial.print(",\t");
      //Serial.print("servo_comm2");Serial.print(servo_comm2);
      //Serial.println(",\t");

  }
  if (elapsed_time > (delaytime + delaytime)/1000) {// one full flap is finisheddt
      elapsed_time = 0; // start next flapping cycle
  }
}

