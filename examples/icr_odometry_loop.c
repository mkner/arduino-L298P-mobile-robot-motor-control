 
/*
 * implementation of ICR equations
 * with odometry using Runge-Kutte aproximation
 * 
 * output is to serial terminal
 * 
 * v0.01.02
 * 
 * (c) 2022-2023 Mike Knerr
 * 
 */
 


/*
 * usual arduino runtime loop
 */

#define ROBOT_TRACK_WIDTH 0.125 // m not cm

void loop() { 

    // init 1st for future port to c++ 

    float velocity = 0.10;// for motor actuation [1,100] --> [0,255] //10; //10;  // 10 for x in [1,100]  0.5;
    float seconds  = 1;
    float angle=0;
    unsigned int motor_speed = 10;
 
    const float mseconds = seconds*1000; // millseconds from seconds 

    const float TW = ROBOT_TRACK_WIDTH;

    float D_right = 0; // distance right wheel during time interval
    float D_left = 0; // distance left wheel during time interval
  
    float D_avg = 0; // robot distance traveled over time interval 
  
    float velocity_avg = 0; //velocity is vector has direction(+/-) 
    float speed_avg = 0; // speed m/s (not a vector)

  
    float v_right =0.0;  // velocity right wheel  velocities , not speed
    float v_left = 0.0;  // velocity left wheel
    float v_avg = 0.0; // see above
    float v_robot = 0.0; //this is from the calculated right & left wheel velocities
    
    float cx=0.0; // current (x,y)
    float cy=0.0;
    float cPhi=0.0; // current phi heading
    float dPhi=0.0; // delta phi change in heading over time interval
    float phi_dot=0.0; // phi angular velocity
  
    float x_prev = 0.0; 
    float y_prev = 0.0;
    float phi_prev = 0.0; // last heading relative to robots reference that starts at 
    
    int cR1 = 0; //encoder counts
    int cL1 = 0;
  
    int cR2 = 0; //encoder counts
    int cL2 = 0;

 
    int delta_ticks_R = 0; // # encoder ticks over time interval
    int delta_ticks_L = 0;
   
  
    // IRC 
    float icr_radius  = 0; 
    float icr_omega = 0; //(v_right-v_left)/TW
    float icr_circ = 0; //2*pi*icr_radius
    
    float icr_wheel_radius = 0; //icr_radius + (TW/2)
    float icr_wheel_circ = 0; //2*pi*icr_wheel_radius

     //Serial.println("\nStarting here...\n\n");

     int time_step = 1000; // in ms
     
     delay(1000);
  
     seconds = 2;
     velocity = 0.25; // when input is in m/s
     motor_speed = 20;//9;//9; //  for motor actuation [1,100] --> [0,255] 
     angle=90;
     
     cR1 = getEncCountA();
     cL1 = getEncCountB();

     seconds = 2;
     motor_speed = 100;
     
     // straight line ahead both wheels
     wheels(BOTH,FORWARD,motor_speed,seconds*1000);
     delay(1000);
     
    
     cR2 = getEncCountA();
     cL2 = getEncCountB();
  
     Serial.println();
     //Serial.print(cR1); Serial.print(","); Serial.println(cR1);
 
     // delta ticks from encoder sensor readings over time interval
 
     delta_ticks_R = cR2-cR1;
     delta_ticks_L = cL2-cL1;

     // get distances, calc velocities,  distances traversed are signed (have direction)
  
     D_right = WHEEL_CIRC*(delta_ticks_R/ENC_WHEEL_TICKS); // distance 
     D_left = WHEEL_CIRC*(delta_ticks_L/ENC_WHEEL_TICKS); // distance 

     /*****************************************************************/
     // OVERRIDE odometry to test calculations against simulator
     //D_left=D_right; //straight line along x axis (forward)
     //D_right=D_left+0.01;// slow turn to left (pos angle )
     /// can simulate perfect spins, motors dont work exaclyt same rate
     //D_left = -D_right; //counter clockwise
     //D_right =- D_left; //clockwise
     // Serial.print("D_right: "); Serial.println(D_right);
     //Serial.print("D_left: "); Serial.println(D_left);
     /*****************************************************************/

     D_avg = (D_right+D_left)/2;  //keep signed

     velocity_avg = ((D_right+D_left)/2)/seconds; //velocity is vector has direction(+/-) 
     speed_avg = abs(velocity_avg); // speed m/s (not a vector)

     // wheel velocities
     // velocities are signed (vectors), not speeds
    
     v_right = D_right/seconds;
     v_left = D_left/seconds;
     v_robot = (v_right+v_left)/2;
 
     dPhi = (D_right - D_left)/TW; //ROBOT_TRACK_WIDTH; // delta phi (heading)
     phi_dot = dPhi/seconds; // omega=phi_dot , angular velocity of phi  over interval

     /****** ICR calculations *******/
 
     if (v_right == v_left) {
       icr_radius  = INFINITY; //arduino builtin constant infinity;
     }
     else {
      icr_radius  = (TW/2)*((v_right+v_left)/(v_right-v_left));
     }
   
     icr_omega = (v_right-v_left)/TW;
     icr_circ = 2*pi*icr_radius;

     icr_wheel_radius = icr_radius + (TW/2);
     icr_wheel_circ = 2*pi*icr_wheel_radius;

     // right wheel path - outer radius
     float icr_right_wheel_radius = icr_radius + (TW/2);
     float icr_right_wheel_circ = 2*pi*icr_right_wheel_radius;
    
     // left wheel path - inner radius
     float icr_left_wheel_radius = icr_radius - (TW/2);
     float icr_left_wheel_circ = 2*pi*icr_left_wheel_radius;


     // get robot previous position 

     x_prev = robotGetPoseX();
     y_prev = robotGetPoseY();
     phi_prev = robotGetPosePhi(); // last heading relative to robots reference that starts at 

     // euler method
     // assume robot initial pose position is (0,0,0) and 
     // robot is facing in positive x direction
     // update to current position at now after moving
     // based on state at beginning of the interval (x_k,y_k,phi_k)
     // x_k+1 and y_k+1 are approximate, phi_k+1 is exact

     //cx = x_prev + D_avg*cos(phi_prev);
     //cy = y_prev + D_avg*sin(phi_prev);

     /*
     R U N G E - K U T T A
     uses average orientation over the integration interval so
     the xk+1 and yk+1 values are better than with just euler when only the initial
     orientation of the interval is used as an offset, results are still approximate
     but  𝜃_k+1 is exact.RK works well with small time step intervals
     */
     
     // this is runga-kutta

     cx = x_prev + D_avg * cos(phi_prev + dPhi/2); // runge-kutta was in siegwart et al chapter 5
     cy = y_prev + D_avg * sin(phi_prev + dPhi/2); 

     cPhi = phi_prev + dPhi;
     cPhi = boundto2pi(cPhi); // works, keep angle in [-2pi,2pi] (360 deg) to wrap around if eg robot rotating in place 
                           //cPhi = tan(sin(cPhi)/cos(cPhi));// not this one, is in [0,pi] this robot can spin in place  

     // update robot current state
     robotSetPose(cx,cy,cPhi); 

     //get coordinates of ICR 
     float icr_pos = (cx-icr_radius*sin(cPhi),cy+icr_radius*cos(cPhi));
   

     /*
      * THERE ARE PRINTED INSIDE go(...) & turn(...) now
     Serial.print("velocity: "); Serial.println(velocity);
     Serial.print("motor speed setting: "); Serial.println(motor_speed);
     Serial.print("Seconds: ");Serial.println(seconds);
     */
     
     Serial.println();
     Serial.print("delta_ticks R: "); Serial.println(delta_ticks_R);
     Serial.print("delta_ticks L: "); Serial.println(delta_ticks_L);
     Serial.println();
     // surface distances traveled by wheels
     Serial.print("D_right\t");Serial.println(D_right);
     Serial.print("D_left\t");Serial.println(D_left);
     Serial.print("D_avg\t"); Serial.println(D_avg);
     Serial.println();
      // wheel velocities & avg for robot
     Serial.print("v_right\t");Serial.println(v_right);
     Serial.print("v_left\t");Serial.println(v_left);
     Serial.print("v_robot\t");Serial.println(v_robot); // this is from the calculated right & left wheel velocities
     //change in heading
     //Serial.print("velocity: ");Serial.println(velocity_avg);
     Serial.println();
     Serial.print("delta_phi: ");Serial.println(int(rad2deg(dPhi))); //Serial.print("\t");Serial.println(dPhi);
  
     Serial.print("omega_phi (phi_dot): ");Serial.println(int(rad2deg(phi_dot))); //Serial.print("\t");Serial.println(dPhi/seconds);
     Serial.println();
      
      // ICR circular path specific 
      Serial.print("icr pos\t"); Serial.print(cx);Serial.print("\t"); Serial.println(cy); //Serial.print("\t"); 
      Serial.print("icr radius\t"); Serial.println(icr_radius);
      Serial.print("icr omega\t"); Serial.println(int(rad2deg(icr_omega)));
      Serial.print("icr circ\t"); Serial.println(icr_circ);
      
      Serial.print("icr wheel radius\t"); Serial.println(icr_wheel_radius);
      Serial.print("icr wheel circ\t"); Serial.println(icr_wheel_circ);
      Serial.println();
      
      Serial.print("icr right wheel circ\t"); Serial.println(icr_right_wheel_circ);
      Serial.print("icr left wheel circ\t"); Serial.println(icr_left_wheel_circ);
      Serial.println();
        
      // pose (x,y,phi)
      Serial.println();
      Serial.print("(x,y,phi) "); Serial.print("( ");Serial.print(cx);Serial.print(" , ");
      Serial.print(cy);Serial.print(" , ");
      Serial.print(int(rad2deg(cPhi))); Serial.print(" )");
      
      Serial.print("\n-------------------------------------\n");
      
      delay(2000);
}
