 
 /*
 * 
 * motor test functions
 * for differential drive robots
 *  
 * (c) 2022-2023 Mike Knerr
 * 
 */
 
/*
 * can run the following test patterns:
 * 
 * test_motors(RIGHT_MOTOR)
 * test_motors(LEFT_MOTOR)
 * test_motors(BOTH_MOTORS)
 * test_motors(BOTH_MOTORS_SPIN)
 */

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

