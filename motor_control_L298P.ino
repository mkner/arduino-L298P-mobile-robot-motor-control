/*
 *
 * 
 * Differential Drive Motor control 
 * 
 * v.0.26
 * 
 * mobile robot differential drive functions for basic motor/wheel control
 * for MotorShield L298P on Arduino UNO
 * 
 * low level actuator functions for basic motor control 
 * higher low-level commands using  rate, time, distance units for motion control 
 * supports odometry using typical IR wheel encoder sensors & encoder wheels for TT-Motors 
 * 
 * versatile functions with intuitive semantic interface 
 * 
 * (c) 2022-2023 Mike Knerr
 * 
 */



//SET ROBOT MODEL TO FWD OR RWD

// USING FWD
#define FRONT_WHEEL_DRIVE
//#define REAR_WHEEL_DRIVE // not implemented yet

///////// L298P Motor Control Board pin assignments //////////

#ifdef FRONT_WHEEL_DRIVE


// using pin assignments for L298P motor control board (shield)
// EN & IN defines are motor board pin-out dependent

// Motor A - RIGHT

#define ENA 10 //motor rotational speed in [0,255] ie.[0,$FF], 0 is stop
#define INA 12 // direction 

#define RIGHT_FORWARD_SIGNAL LOW // LOW on A  for front wheel drive RIGHT side motor
#define RIGHT_REVERSE_SIGNAL HIGH // HIGH on A  for front wheel drive RIGHT side motor

// MOTOR B - LEFT

// pin assignments L298P

#define ENB 11 //motor rotational speed
#define INB 13 //direction

#define LEFT_FORWARD_SIGNAL HIGH // HIGH on A  for front wheel drive LEFT side motor
#define LEFT_REVERSE_SIGNAL LOW // LOW on A  for front wheel drive LEFT side motor

#endif

#ifdef REAR_WHEEL_DRIVE
// not implemented yet
/*
// REAR WHEEL DRIVE 
// 
// Motor A - LEFT
#define ENA  //speed
#define INA //direction

// MOTOR B - RIGHT
#define ENB //speed
#define INB  //direction
*/

#endif

// standard constants 

const float pi = 3.141592; // precision is only 6-7 digits on arduino //#define pi 3.1415962535 

//ROBOT PHYSICAL PARAMETERS 

// physical parameters for wheels, chassis...

const float ROBOT_TRACK_WIDTH = 0.13; // changed 11/12/22 to 0.13 from 0.125; // 090; // 0.085; // meters (85 mm,, 8.5 cm)  ,<<<<<<<====== CHECK THIS!
const float ROBOT_CIRC = 2*ROBOT_TRACK_WIDTH; // may want to use this as constant across equations
const float ROBOT_WHEEL_RADIUS = 0.0325; // meters, 32.5 mm = 3.25 cm was diameter = 0.065; //meters (65 mm, 6.5 cm))

const float WHEEL_CIRC = 2*pi*ROBOT_WHEEL_RADIUS; //; //wheel circumference


// MOTOR FUNCTION 

// cheap motors sometimes need to get a pulse to start moving w/ L298P?
// at very low speeds or from a stop when not enough stalling current is available
// so have to prime the pump
// these are the global default parameters that can be
// set to use and fine tune the auto prime feature in set_motor_speed(...)

#define AUTO_PRIME true // turns on auto priming for motors at low speeds
#define DEFAULT_MOTOR_PRIME_DURATION 35 // was 50 //100 // in ms
#define DEFAULT_MOTOR_PRIME_RATE 35 //30 //was 30 // % in scale [1,100]
#define MIN_AUTO_PRIME_SPEED 15 //10  //15 //20 //25  // in [0,100] prime motors if set under this speed


// calibrate min starting range //////

#define MINSPEED 45 // this is the min motor speed setting where the motors will turn in [1,100], 6v & Stemedu (yellow) motors

// calibration parameters for angular velocity of wheels to map into [1,100]
// that defines the [min,max] range of motors physical rotation using the motor control board

// min/max rotational rates have be determined from field test of motors
// using rough aproximation by sight not odometry
// was before actually motor test with this function
  
const float MOTOR_MIN_ANGULAR_VELOCITY = 4.7124; //4; //; //4.7124; // from visual measurement  3.0; //4.0; 
  
// 1 m/s w/ 20cm circumference wheel is 
// radians per cm = (2*pi)/20 = 0.3141592653589793
// the min rate (lower bound) motor can turn at is 45, this is motor speed 1 ie. 1 in [1,100] (via map or direct)
// recheck visual from video, at speed 1, turns 15 cm in 1 second
// this is 4.71238898038469 radians per second at speed setting 1
//
// 1 m/s linear w/ 3.25 cm radius wheels is angular velocity of 30.769230769230766... radians per second
  
const float MOTOR_MAX_ANGULAR_VELOCITY = 18.85; // 20; //30; //25;// //18.85; //visual measurement ~ 3 revolutions @ 100% //31.42; /// ~ 1 m/s in radians per second

// ODOMETRY

// arduino uno pinouts used for odometry sensors
#define INTERRUPT_ENCODER_SENSOR_A 0 // MOTOR/WHEEL A  w/ sensor on pin #2
#define INTERRUPT_ENCODER_SENSOR_B 1 // MOTOR/WHEEL B w/ sensor on pin #3

const float ENC_WHEEL_TICKS = 20;

  
/*
 * make sure compiler fixes these variables 
 * in specific memory locations
 * since used by interrupt functions
*/

volatile unsigned int tickCounterA = 0;
volatile unsigned int tickCounterB = 0;

 
// SE&L 

// robots state (pose)
// volatile since are updated in an ISR & too important to move around
 
volatile float robot_cx = 0; //current x position // anchor in memory current state too important 
volatile float robot_cy = 0; //current y position
volatile float robot_phi = 0; //current heading  



// functions 

//constrain selections to enum type

enum SIDE { RIGHT, LEFT, BOTH, NEITHER }; 

// STOP is stop immediately
// STOP_WAIT, stops and will pause for duration set in ms
// spin right is clockwise, spin left is counter-clockwise

enum DIRECTION { FORWARD, REVERSE, SPIN_RIGHT, SPIN_LEFT, STOP, STOP_WAIT, NONE}; //none is placeholder

enum MOTOR_TEST_TYPE {RIGHT_MOTOR, LEFT_MOTOR, BOTH_MOTORS, BOTH_MOTORS_SPIN};

#define DEFAULT_STOP_DURATION 1500 // 1.5 sec when using wheel(STOP_WAIT,...) or wheels(...) w/out args



// DEBUG 

#define BUZZER 4

#define BEEPS_ON  false // true //turn off if dont want runtime progress beeps 
#define DEBUG true

// INIT 

void setup() {
  
// int y=f2(2);

  // Motor A - RIGHT w/ FWD
  pinMode(ENA, OUTPUT);
  pinMode(INA, OUTPUT);

  // Motor B - LEFT W/ FWD
  pinMode(ENB, OUTPUT);
  pinMode(INB, OUTPUT);

  pinMode(BUZZER, OUTPUT);
   
  // send stop signal(s)
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  if (DEBUG) {
    Serial.begin(9600);
    Serial.print("\n");
    Serial.println("Hello...\n");
    //Serial.println("Running time in milliseconds:");
    //Serial.println("\t");
  }

  //init state
  robot_cx = 0; //current x position
  robot_cy = 0; //current y position
  robot_phi = 0; //current heading  
  
  
  // attach interrupts to ISRs for odometry
  attachInterrupt(INTERRUPT_ENCODER_SENSOR_A,encCounterA,FALLING);
  attachInterrupt(INTERRUPT_ENCODER_SENSOR_B,encCounterB,FALLING);

}


// DEBUG 

void beep(int dmsec) {
  // single beep of duration dmsec
  digitalWrite(BUZZER,HIGH);delay(dmsec);digitalWrite(BUZZER,LOW);
}


void beeps(int cnt = 1, float wait = 1, int dur=250, int pause = 250 ) {

  // one or more beeps
  // default is 1 beep, wait 1 sec before exit, 
  // 250 ms duration of each beep, 250 ms pause between beeps 
  // call as beeps(); (is same as beeps(1);
  
  if (BEEPS_ON) { 
    for (int i=1; i<=cnt; i++) { // lenght of beep and pause betwen are equal
    beep(dur);delay(pause); // in milliseconds
    }
  delay(wait*1000); // wait before exiting in seconds
  }
}

// ODOMETRY 

static int motorAdir=1;
static int motorBdir=1;

// ISR FOR sensor on Motor A - RIGHT
void encCounterA() {
    tickCounterA++;
    //robotUpdatePose(); //FUTURE was ok w/ interrupt
}

// ISR FOR sensor on MOTOR B - LEFT
void encCounterB() {
    tickCounterB++;
    //robotUpdatePose(); //FUTURE was ok w/ interrupt
}


int getEncCountA() {

  // since only single encoder sensor per wheel 
  // motion functions set the direction and
  // is retreived here
 
  noInterrupts();
  int count = tickCounterA * getMotorAdir();
  interrupts();
 
  return count; 
  }


int getEncCountB() {

  noInterrupts();
  int count = tickCounterB * getMotorBdir();
  interrupts();
  
  return count;
}

void setMotorAdir(DIRECTION dir) {
  
// set current direction of encoder on motor A
// needed since only one encoder sensor per wheel 
// so direction is unknown from encoder sensor signals

 if (dir == FORWARD) {
  motorAdir = 1;
  }
  else if (dir == REVERSE) {
  motorAdir = -1;
  }
  // otherwise don't do anything
 
}


void setMotorBdir(DIRECTION dir) {
  
// set current direction of encoder on motor A
// needed since only one encoder sensor per wheel 
// so direction is unknown from encoder sensor signals

 if (dir == FORWARD) {
  motorBdir = 1;
  }
  else if (dir == REVERSE) {
  motorBdir = -1;
  }
  // otherwise don't do anything
 
}

int getMotorAdir() {
 return motorAdir;
}

int getMotorBdir() {
 return motorBdir;
}




// LOCALIZATION -- SE&L //

float robotGetPoseX() {
  return robot_cx;
  }
  
float robotGetPoseY(){
  return robot_cy;
  }
  
float robotGetPosePhi(){
  return robot_phi;
  }


void robotPoseInit() {
  robot_cx = 0.0;
  robot_cy = 0.0;
  robot_phi = 0.0;
}


void robotSetPose(float x,float y,float phi) {
  // set pose manually update current state
  robot_cx = x;
  robot_cy = y;
  robot_phi = phi;
}

void robotUpdatePose() {
  // UpdatePose
  // called from interrupts to updates state
  getEncCountA(); // update pose called from interrupt, was ok 

}



int get_motor_speed(int spd) {
  
// maps [0,100] -> [0,FF] for arduino uno board 

  if (spd == 0) {
    return 0;
  }
  else {
   spd = constrain(spd,1,100); // map does not bound input range
   return map(spd,1,100,MINSPEED,255);
  }
   
}


void set_motor_speed(int pin, int rate) {

// relatively low-level function 
// use this to automatically manually
// prime motors for slow speeds if using the AUTO_PRIME
// feature, AnalogWrite is builtin this function
// pass the pin # as an arg & rate


 int new_speed = get_motor_speed(rate);
  
  if (AUTO_PRIME) { // if set then pulse out to get motor moving

   if (new_speed > 0 && new_speed < get_motor_speed(MIN_AUTO_PRIME_SPEED) ) { //let STOP fall through
       analogWrite(pin, get_motor_speed(DEFAULT_MOTOR_PRIME_RATE) ); //kick it
       delay(DEFAULT_MOTOR_PRIME_DURATION); 
      }
   } 

   analogWrite(pin,new_speed); // continue with usual speed seting
}

// MOTOR CONTROL //


void wheel(SIDE side, DIRECTION dir, int rate=0,  int dur=0, bool nonstop=false) {

// turn by pivoting 
// rotate only one wheel at a time
// this will pivot robot on other stationary wheel to turn

// right & left depend on chassis config with motors 


switch(side) {
  
  case RIGHT:

  if (dir == FORWARD) {

     setMotorAdir(FORWARD);
     digitalWrite(INA, RIGHT_FORWARD_SIGNAL); 
     
  } else if (dir == REVERSE) {
    
     setMotorAdir(REVERSE);
     digitalWrite(INA, RIGHT_REVERSE_SIGNAL); 
     
  } else if (dir == STOP_WAIT) { 
     if (rate != 0  && dur == 0) { 
      dur = rate;
      rate = 0;
      } else if (rate == 0 && dur == 0) {
        dur = DEFAULT_STOP_DURATION;
      }
   } else if (dir == STOP) { //stop now
     dur = 0;
     rate = 0;
   }

  // run motor A at rate
  set_motor_speed(ENA, rate); 

  if (dur != 0) { // is a STOP_WAIT
    delay(dur); //wait
  }
  
  set_motor_speed(ENA, 0); //then stop
  
  break; //right

  
   case LEFT:
   
   if (dir == FORWARD) {

     setMotorBdir(FORWARD);
     digitalWrite(INB, LEFT_FORWARD_SIGNAL); 
     
   } else if (dir == REVERSE) {
    
     setMotorBdir(REVERSE);
     digitalWrite(INB, LEFT_REVERSE_SIGNAL); 
     
   } else if (dir == STOP_WAIT) { 
     if (rate != 0  && dur == 0) { 
      dur = rate;
      rate = 0;
      } else if (rate == 0 && dur == 0) {
        dur = DEFAULT_STOP_DURATION;
      }
   } else if (dir == STOP) { // stop now
     dur = 0;
     rate = 0;
   }
   
   set_motor_speed(ENB, rate); 
  
  if (dur != 0) { // is a STOP_WAIT
    delay(dur); //so wait
   }
   
   set_motor_speed(ENB, 0); //then stop
  
  break; //left

  
   }//end switch

}//end wheel 


void prime_motor(SIDE side, DIRECTION dir, int rate=DEFAULT_MOTOR_PRIME_RATE,\
     int duration=DEFAULT_MOTOR_PRIME_DURATION) {
      
// this is the manual version to  kick start sluggish cheapo motors, use as required

  wheel(side,dir,rate,duration);
}


void wheels(SIDE side1, DIRECTION dir1, int rate1=0, int dur1=0, SIDE side2=NEITHER, DIRECTION dir2=NONE, int rate2=0, int dur2=0) {

//void wheels(SIDE side1, DIRECTION dir1, int rate1=0,  SIDE side2=NEITHER, DIRECTION dir2=NONE, int rate2=0, int dur=0) {

  // in v0.19
  // there is no continuous mode with this function
  // 
  
  bool nonstop;
  
  //
  // ignore redundant or invalid cases
  //
  
  //if 2 sides need both different 
  if (side1 == side2) {
    return;
  }
  
  // ignore BOTH sides, used twice, do nothing
  if (side1 == BOTH && side2 == BOTH) {
    return;
  }
  
  // BOTH has to be used only once as 1st arg
  if (side2 == BOTH ) {
    return;
  }

  // shortcut for full stop
  if (side1 == BOTH && dir1 == STOP) {
    wheel(side1,STOP);
    wheel(side2,STOP);
    return;
  }

  // 
  //shortform syntax for BOTH motors at same rate
  //in tandom movement of wheels & SPIN in place
  //
  
  if (side1 == BOTH ) {
 
    if (dir1 == FORWARD) {

      setMotorAdir(FORWARD);
      digitalWrite(INA, RIGHT_FORWARD_SIGNAL);
      
      setMotorBdir(FORWARD);
      digitalWrite(INB, LEFT_FORWARD_SIGNAL);
     
    } else if (dir1 == REVERSE) {
      
      setMotorAdir(REVERSE);
      digitalWrite(INA, RIGHT_REVERSE_SIGNAL);

      setMotorBdir(REVERSE);
      digitalWrite(INB, LEFT_REVERSE_SIGNAL);

      //clockwise, right reverse, left forward
      
    } else if (dir1 == SPIN_RIGHT) {
      
      setMotorAdir(REVERSE);
      digitalWrite(INA, RIGHT_REVERSE_SIGNAL);
      
      setMotorBdir(FORWARD);
      digitalWrite(INB, LEFT_FORWARD_SIGNAL);
      
     //counter-clockwise, right forward, left reverse 
     
    } else if (dir1 == SPIN_LEFT) {
      
      setMotorAdir(FORWARD);
      digitalWrite(INA, RIGHT_FORWARD_SIGNAL);
      
      setMotorBdir(REVERSE);
      digitalWrite(INB, LEFT_REVERSE_SIGNAL);
      
    }  else if (dir1 == STOP_WAIT) { 
      
     if (rate1 != 0  && dur1 == 0) { 
      dur1 = rate1;
      rate1 = 0;
      } else if (rate1 == 0 && dur1 == 0) {
        dur1 = DEFAULT_STOP_DURATION;
      }
   } else if (dir1 == STOP) { // stop now
     dur1 = 0;
     rate1 = 0;
   }

  set_motor_speed(ENA,rate1); 
  set_motor_speed(ENB,rate1); 

  if (dur1 != 0) { 
    delay(dur1); //wait or keep running
   }

  //stop both wheels
  //for now
  set_motor_speed(ENA, 0); //then off signal
  set_motor_speed(ENB, 0); 
  
  // are these working? yes
  //wheel(side1,STOP);
  //wheel(side2,STOP);

  return;
  
  } //end tandom commands ("BOTH" commands)
  
  //================================================================

  
  // if made it this far, control both wheels seperately at the same time
  // this can rotate both motors at different rates 
  // or at the same rate, and in the same of different directions 
  // at the same time and have different durations for each wheel in one command

  
  // RIGHT wheel
  if (side1 == RIGHT) {
    
    if (dir1 == FORWARD) {
     digitalWrite(INA, RIGHT_FORWARD_SIGNAL); 
     
    } else if (dir1 == REVERSE) {
     digitalWrite(INA, RIGHT_REVERSE_SIGNAL); 
     
    } else {
    return; // not valid direction , just bail out
    }
    
    set_motor_speed(ENA,rate1); 
  }

   if (side2 == RIGHT) {
    
    if (dir2 == FORWARD) {
     digitalWrite(INA, RIGHT_FORWARD_SIGNAL); 
     
    } else if (dir2 == REVERSE) {
     digitalWrite(INA, RIGHT_REVERSE_SIGNAL); 
     
    } else {
    return; // not valid direction , just bail out
    }
    
    set_motor_speed(ENA,rate2); 
  }
  
  // LEFT wheel
  if (side1 == LEFT) {
  
  if (dir1 == FORWARD) {
     digitalWrite(INB, LEFT_FORWARD_SIGNAL); 
     
  } else if (dir1 == REVERSE) {
     digitalWrite(INB, LEFT_REVERSE_SIGNAL); 
     
  } else {
    return; // not valid direction , just bail out
  }
  set_motor_speed(ENB,rate1);
  }
  
  if (side2 == LEFT) {
    
    if (dir2 == FORWARD) {
     digitalWrite(INB, LEFT_FORWARD_SIGNAL); 
     
    } else if (dir2 == REVERSE) {
     digitalWrite(INB, LEFT_REVERSE_SIGNAL); 
     
    } else {
    return; // not valid direction , just bail out
    }
    set_motor_speed(ENB,rate2);
  } 

 
 // timers for the wheels with different durations

  const unsigned long max_time = 300000; //60*1000*5; //5 minutes
  
  //durations are in milliseconds

  bool side1_stopped = false;
  bool side2_stopped = false;

  unsigned long now = millis();
  unsigned long dur1_endtime = now + dur1;
  unsigned long dur2_endtime = now + dur2;
  
 // both same duration 
 // may want to skip this short version
 // if delay() function not to accurate? for long time periods
 
 if ( dur1 == dur2) {
   delay(dur1);
   wheel(side1,STOP);
   wheel(side2,STOP);
   return;
  
 } else {

  // countdown by milliseconds

  for (int i=0; i<= max_time; i++) {
    
    delay(10); // 1 millisecond
    
    if (millis() >= dur1_endtime) {
      wheel(side1,STOP);
      side1_stopped = true;
    }
   
    if (millis() >= dur2_endtime) {
      wheel(side2,STOP);
      side2_stopped = true;
    }
    
    //both wheels done running their durations so bye
    if (side1_stopped && side2_stopped) {
      return;
    }
  }
 } //end else
 
  // otherwise something goofed?
  wheel(RIGHT,STOP);
  wheel(LEFT, STOP);


 }  //end wheels(...) 
 //


 

void unicycle(DIRECTION dir, int rate=0, int dur=0) {

// commands are constrained to the motion of a one wheel unicycle 
// implemented on a two wheel differential drive circular chassis robot

  switch(dir) {

    case FORWARD:
      wheels(BOTH,FORWARD,rate,dur);
    break;

    case REVERSE:
      wheels(BOTH,REVERSE,rate,dur);
    break;

    case SPIN_RIGHT:
      wheels(BOTH,SPIN_RIGHT,rate,dur);
    break;

    case SPIN_LEFT:
      wheels(BOTH,SPIN_LEFT,rate,dur);
    break;

    case STOP:
      wheels(BOTH,STOP);  //,rate); //use syntax unicycle(STOP,1000); stop 1 sec or unicycle(STOP); 
    break;
    
    //anything else do nothing
  } //end switch
}


void test_motors(MOTOR_TEST_TYPE motor_type ) {

// test motor(s) routine
// to put motors through the paces
// test types are {RIGHT_MOTOR, LEFT_MOTOR, BOTH_MOTORS, BOTH_MOTORS_SPIN}

  SIDE side;
  DIRECTION dir;
  
   // set side here once only
 
  switch(motor_type) {

   case RIGHT_MOTOR:
     side = RIGHT;
   break;

   case LEFT_MOTOR:
    side = LEFT;
   break;
   
   case BOTH_MOTORS:
    side = BOTH;
   break;
   
   case BOTH_MOTORS_SPIN:
     side = BOTH;
   break;

  } //end switch
     
 //////////////////////////////////////////////////////////
 // test min,avg,max speed levels where motor will rotate 
 // pause betwen rate changes
 /////////////////////////////////////////////////////////
  
   //FORWARD/////////////////////////////

   if (motor_type == BOTH_MOTORS_SPIN){
      dir = SPIN_RIGHT;
   }
     else {
        dir = FORWARD; 
   } 
   
   // minimum speed 
   if (side == BOTH) {
     wheels(side,dir,1,3000);  
     wheels(side,STOP_WAIT,1000);
   } else { 
     wheel(side,dir,1,3000);  
     wheel(side,STOP_WAIT,1000);
   }
   
   // 50%
    if (side == BOTH) {
     wheels(side,dir,50,3000);  
     wheels(side,STOP_WAIT,1000);
   } else { 
     wheel(side,dir,50,3000);  
     wheel(side,STOP_WAIT,1000);
   }
   
   // 100%
    if (side == BOTH) {
     wheels(side,dir,100,3000);  
     wheels(side,STOP_WAIT,1000);
   } else { 
     wheel(side,dir,100,3000);  
     wheel(side,STOP_WAIT,1000);
   }

   //wait
   delay(1000);  
   
    //REVERSE///////////////////////////

    if (motor_type == BOTH_MOTORS_SPIN){
      dir = SPIN_LEFT;
    }
     else {
      dir = REVERSE; 
    }
   // minimum speed 
   if (side == BOTH) {
     wheels(side,dir,1,3000);  
     wheels(side,STOP_WAIT,1000);
   } else { 
     wheel(side,dir,1,3000);  
     wheel(side,STOP_WAIT,1000);
   }
   
   // 50%
    if (side == BOTH) {
     wheels(side,dir,50,3000);  
     wheels(side,STOP_WAIT,1000);
   } else { 
     wheel(side,dir,50,3000);  
     wheel(side,STOP_WAIT,1000);
   }
   
   // 100%
    if (side == BOTH) {
     wheels(side,dir,100,3000);  
     wheels(side,STOP_WAIT,1000);
   } else { 
     wheel(side,dir,100,3000);  
     wheel(side,STOP_WAIT,1000);
   }

   //pause
   delay(1000);

   //ok 9/29

   /////////////////////////////////////////////////
   /// forward & rev right away @ min,avg,max speeds 
   /////////////////////////////////////////////////
    
    DIRECTION fdir, rdir;

    if (motor_type == BOTH_MOTORS_SPIN){
      fdir = SPIN_RIGHT;
      rdir = SPIN_LEFT;
    }
     else {
      fdir = FORWARD; 
      rdir = REVERSE;
    }
    
   // minimum speed
   if (side == BOTH){
     wheels(side,fdir,1,3000);
     delay(250); 
     wheels(side,rdir,1,3000);
   } 
   else {
     wheel(side,fdir,1,3000);
     delay(250); 
     wheel(side,rdir,1,3000);
   }
   
   delay(750);
  
   // 50%
   if (side == BOTH){
     wheels(side,fdir,50,3000);
     delay(250); 
     wheels(side,rdir,50,3000);
   } 
   else {
     wheel(side,fdir,50,3000);
     delay(250); 
     wheel(side,rdir,50,3000);
   }
   
   wheels(BOTH,STOP_WAIT,750);

   if (side == BOTH){
     wheels(side,fdir,100,3000); 
     wheels(BOTH,STOP_WAIT,250); //need stop & wait to run after @ higher speeds
     wheels(side,rdir,100,3000); 
   }
   else {
     wheel(side,fdir,100,3000);
     wheel(side,STOP_WAIT,250);
     wheel(side,rdir,100,3000);
   }

   wheels(BOTH,STOP_WAIT,2000);

   ///////////////////////////////////////////////////////////////////
   // increase speed forward, decrease reverse (accelerate/decelerate)
   // works also with both 2 wheel SPIN & NON-SPIN
   ///////////////////////////////////////////////////////////////////

const int p = 40; //space pulses apart by p milliseconds

  //FORWARD
  
   if (motor_type == BOTH_MOTORS_SPIN){
       dir = SPIN_RIGHT;
    }
      else {
        dir = FORWARD; 
    }

   for(int spd = 1; spd <= 100; spd++)
      {
        if (side == BOTH){
          wheels(side, dir, spd, 50); 
         } else {
          wheel(side, dir, spd, 50); 
         }
      }

    //pause
    wheels(BOTH, STOP_WAIT,1500);
   
  // REVERSE

   if (motor_type == BOTH_MOTORS_SPIN){
       dir = SPIN_LEFT;
    }
      else {
        dir = REVERSE; 
    }

   for(int spd = 100; spd >= 1; spd--)
      {
       if (side == BOTH){
        wheels(side,dir,spd, 50); 
      } else {
        wheel(side,dir,spd, 50); 
      }
   }
   wheels(side,STOP_WAIT,2000); 

   
  return; //END OF TEST
  
} //end motor test function


///////// math & utility //////////

float deg2rad(float degree) {
  
  // use values calc from python
  // from math import pi
  // (2*pi)/360
  // Out[6]: 0.017453292519943295
  // яндекс 0,01745329251
  // also builtin constant is DEG_TO_RAD 0.01745329 Константа перевода град в рад 
  
  float radian = degree * 0.017453292519943295;  /////////0.017452006980803);
  return radian;
}

float rad2deg(float radian) {
  
  // use values calc from python
  // from math import pi
  // (2*pi)/360
  // Out[6]: 0.017453292519943295
  // яндекс 0,01745329251

  float degree= radian/0.017453292519943295  ;  /////////0.017452006980803);
  return degree;
}


float boundto2pi(float angle) {
  
  // dont bound w/ eg arctan2(sin(theta),cos(theta)) since one full turn around or less is ok
  // since robot can rotate in place
  // with no change in postion allow angles in [-2pi,2pi]
  // unwind angle when it goes over/under 2pi/-2pi (360/-360 deg)
  // to wrap it back into range
   
  
    while (angle > 2.0*pi) // counter-clockwise
        angle -= 2.0*pi;
        
    while (angle < -2.0*pi) // clockwise if turn past -2pi
        angle += 2.0*pi;
        
    return angle;
}


float boundWheelAngVel(float omega) {

// bound motor/wheel angular velocity to limits 
// determined by testing #defined above

   if (omega < MOTOR_MIN_ANGULAR_VELOCITY){
      omega = MOTOR_MIN_ANGULAR_VELOCITY;
     }
         
     if (omega > MOTOR_MAX_ANGULAR_VELOCITY){
       omega = MOTOR_MAX_ANGULAR_VELOCITY;
     }
     
   return omega;
   
}

/* python
def amap(x, in_min, in_max, out_min, out_max):
    # python equiv of arduino map
  return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min
*/

/*
 * ARDUINO C
  #long map(long x, long in_min, long in_max, long out_min, long out_max) {
  #return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  #}
*/


float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  
    // float map for eg angular velocity eg w_wheels
    // float equiv of arduino map (that uses long numeric types)
    
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


////////////////////// motion control start here //////////////////////////

float omega2MotorSpeed(float omega) {

//  get  motor rotation rate in in range [1,100] for low level motor control
//  from wheel/motor angular velocity value 
 
   omega = boundWheelAngVel(omega);  // constrain input domain
   
   float mspeed = fmap(omega, MOTOR_MIN_ANGULAR_VELOCITY, MOTOR_MAX_ANGULAR_VELOCITY, 1,100);

   return(mspeed);
}



void go( DIRECTION dir, float sped, float dur) {

// inputs: sped = speed in m/s, speed is arduino reserved word
// DIRECTION makes it velocity since it will have direction
// dur(ation) in seconds , want to keep in usual SI/metrics  units for motion control
// not motor/actuation control level that would be in // was milliseconds

  float w_wheels = sped/ROBOT_WHEEL_RADIUS; //angular (rotational) velocity of wheel(s) (omega) in radians per second
 
  //just go forward or reverse at speed meters/second

  //  speed to set motor rotation rate in range of [1,100] //note 10/16  works
   
   float motor_speed = omega2MotorSpeed(w_wheels); 

    Serial.println();
    Serial.print("velocity: "); Serial.println(sped);
    Serial.print("motor speed setting: "); Serial.println(motor_speed);
    Serial.print("Seconds: ");Serial.println(dur);
     
  if (dir == FORWARD) {

    wheels(BOTH, FORWARD, motor_speed, dur*1000);
    
    
  } else if (dir == REVERSE) {

    wheels(BOTH, REVERSE ,motor_speed, dur*1000);
     
  }
  
// otherwise do nothing
  
}



//void goto(x,y,v) {}
// goto(x,y,velocity)
// turn to heading to (x,y) the go to the point
// velocity is a vector, + forward - backward, goto is just linear traversal (ie straight ahead or backwards)
// turn (spin) first in place, then just go(...)


void turnTo(int heading, float rate) {
  // ok fine, which reference frame?
  // from current heading turn to new heading
  // by turning in place 

  
}

// MOTION CONTORL
// robot motion commands 
// use underlying low-level motor motion (control) commands

// *here

float turn(int angle,  float velocity) {
  
// turn by rotating in place using a set angle and linear speed of wheels
// specify: angle in degrees & linear velocity of wheels in m/s , uses  wheels( spin...) 
// angle: degrees to turn by rotating in place to a new heading (yaw) 
// (+) is normal radian direction (counter-clockwise, (-) is clockwise
// velocity: linear speed of wheels to turn at (meters/sec), is used as > 0 only
// returns: time that was calculated to use in other odometry etc calcs

    /*
    if (velocity <= 0.0 || angle == 0) { 
        return; //just bail for now
       }
    */

   /*
    if ( angle == 0 ) { 
        return; //just bail for now
       }
   */

    // leave 0 angle as ok in this function might be used by a controller
    // and could  have transient 0 direction change values 

    // gets lower bounded anyway, but make things easier
    // may change in future if a controller (eg PID) needs to send (-) velocity to reverse spin
    
    velocity=abs(velocity);

    float theta = boundto2pi(deg2rad(angle)); // bound in [-2pi,2pi] & convert to radians, 

    // w = v/r  get wheel angular velocity w (omega) , v is linear velocity (surface) of wheel 
    
    float w_wheels = velocity/ROBOT_WHEEL_RADIUS; //r_wheel; //angular (rotational) velocity of wheel(s) (omega) in radians per second
 
    float c_robot = pi*ROBOT_TRACK_WIDTH; // the turning circumference of the robot, trackwidth as diameter (not physical chassis)

    /// DEBUG
    Serial.print("\n\n");
    //Serial.print("pi:\t"); Serial.println(pi); 
    Serial.print("theta:\t"); Serial.println(theta); //ok
    Serial.print("velocity:\t"); Serial.println(velocity); 
    Serial.println();
    Serial.print("c_robot:\t"); Serial.println(c_robot); 
    Serial.print("r_wheel:\t"); Serial.println(ROBOT_WHEEL_RADIUS); 
    Serial.println();
    Serial.print("w_wheels:\t"); Serial.println(w_wheels); 


    w_wheels = boundWheelAngVel(w_wheels); //radian bound on wheel omega to motor constraints

    Serial.print("w_wheels (bounded):\t"); Serial.println(w_wheels);  
     
    float t = (theta*c_robot)/(2*pi*ROBOT_WHEEL_RADIUS*w_wheels); // how long to turn wheels to reach angle position
   
    float motor_speed = omega2MotorSpeed(w_wheels); // from angular velocity, uses fmap

   //////
   // DEBUG
   if (angle <= 180) {
    motor_speed = motor_speed+ motor_speed*0.16;
   }
    
    Serial.print("motor_speed:\t"); Serial.println(motor_speed); 
    Serial.println();
    Serial.print("t:\t"); Serial.println(t); // print time after omega (w_wheels angular velocity) & motor speed

    float t_ms = t*1000; // to milliseconds

    Serial.print("t_ms:\t"); Serial.println(t_ms);

   // in v0.25 was backwards (+) theta needs to be spin right (clockwise), left (counterclockwise)
   // theta sets direction velocity is always positive
   // fix in v0.26


    if (theta >= 0) {  
      
      //counter-clockwise
      wheels(BOTH,SPIN_LEFT, motor_speed, t_ms);
      
    } else if (theta < 0)  {
      
     // clockwise
     wheels(BOTH,SPIN_RIGHT, motor_speed, t_ms);
    }
    

  return t; // in seconds not ms
  
}


/*** MAIN LOOP ***/

void loop() { 

/* see examples */

}
