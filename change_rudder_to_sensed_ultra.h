#include <LookUpT>;



const float angle = 22.5;
const float tol = 0.9;

float controllerOutput;
float headingerror;

float previousError = 0.0;
float controllerOutput = 0.0;



int change_rudder_to_sensed_ultra(int which_ultra_is_on, int heading_deg)
{
   
  int required;  
  switch (which_ultra_is_on)
  {
    case 1: required = angle;    break;
    case 2: required = 2*angle;  break;
    case 3: required = 3*angle;  break;
    case 4: required = 4*angle;  break;
    case 5: required = 5*angle;  break;
    case 6: required = 6*angle;  break;
    case 7: required = 7*angle;  break;
    case 8: required = 8*angle;  break;
  }
        // Calculate error signal
        // TODO: Deal with the 360 = 0 problem
    	headingerror = heading_deg - required;

        if (abs(headingerror) < tol){
        	return;
        }
        
        // Compute controller output
        controllerOutput = heading_pid.pid_func(headingerror, previousError, LookUpT.Ki, LookUpT.Kp, LookUpT.Kd, LookUpT.Tbase, LookUpT.MAX, LookUpT.MIN);
        
        
        // new fpid
        fpid fc = new fpid();
        // Calculate new heading      
		//double ruddersign = fc.rudder_fpid(headingerror);
        float ruddersign=1;
        heading_deg = heading_pid.shipTurn(heading_deg, ruddersign, controllerOutput);
       
        // Calculate error
        previousError = headingerror;
        headingerror = heading_deg - required;
        
        //System.out.println("Heading: " + heading_deg + " Error: "+ headingerror + " Controller: "+ controllerOutput);
        //System.out.println("params[0]: " + params[0] + " params[1]: "+ params[1] + " params[2]: "+ params[2]);

        //printf("%d,%f,%f\n", i, heading, error);

       
        fc.applyHardControl(carX, carY, xPlannedPath);
}
