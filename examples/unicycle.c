/*
 * example using wheels function to create 
 * basic unicycle movement
 *  
 * (c) 2022-2023 Mike Knerr
 * 
 */
 

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
