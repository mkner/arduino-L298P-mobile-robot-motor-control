Functions for basic motor/wheel control of differential drive mobile robots\
using MotorShield L298P on Arduino UNO. Implemented in C/C++ 
   
 * low level controller/actuator functions for basic motor control
   
 * higher low-level commands using  rate, time, distance units for motion control
   
 * supports odometry using interrupt driven IR wheel encoder sensors & encoder wheels for TT-Motors
   
 * has an auto-prime function that can kick-start TT-Motors into rotation \
   when stalled and not enough current to escape resting inertia at low velocities
   
 * versatile functions with intuitive semantic interface

 * wheel odometry using Runge-Kutte approximation\
   with serial terminal output of runtime results from implementation of\
   ICR - Instananeous Center of Rotation - equations


       

  
Example from unicycle.c

Unicycle movements with a differential drive robot

Use the wheels function to create a unicycle function that limits the\
motion of the mobile robot to that of a unicycle
   
``` cpp

   unicycle(FORWARD, 25, 4000) // go forward at 25% for 4 seconds
   
   unicycle(SPIN_LEFT, 5, 2000) // spin in pos dir with rotational velocity (phi-dot) at 5% for 2 seconds
   
   delay(4000)
   
   unicycle(FORWARD, 25) // move forward @ 25% continuously
   
   unicycle(STOP) //until stopped
   
   delay(2000) 
   
   //note: wheels(STOP,2000) is same as above

```

