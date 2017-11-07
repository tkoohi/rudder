#include <LookUpT>;



class heading_pid {
	
	float epsilon = 0.01;

	// Assign global variables
	float derivative = 0.0;
	float integral   = 0.0;

	// PID FUNCTION
	float pid_func(float error, float previousError, float Ki, float Kp, float Kd, float dt, float MAX, float MIN)
	{
	   
	   float output;

	   //In case of error too small then stop intergration
	   if(abs(error) > epsilon)
	   {
	      integral = integral + error*dt;
	   }
	   derivative = (error - previousError)/dt;
	   output = Kp*error + Ki*integral + Kd*derivative;

	   //Saturation Filter
	   if(output > MAX)
	   {
	      output = MAX;
	   }
	   else if(output < MIN)
	   {
	      output = MIN;
	   }   

	    return output;
	}

	// SHIP TURN - CALCULATES HOW THE SHIP'S HEADING ALTERS
	float shipTurn(float currentHeading, float ruddersign, float controllerOut)  {

	   float newHeading;
	   float turnRate;
	
	   new LookUpT();

	   // Calculate rudder position from controller output
	   rudder = -LookUpT.maxRudder * controllerOut;
	   //rudder = -ruddersign  * controllerOut;
	   
	   // Take account of rudder end stops
	   if(rudder > LookUpT.maxRudder) {
	      rudder = LookUpT.maxRudder;
	   }
	   else if(rudder < -LookUpT.maxRudder) {
	      rudder = -LookUpT.maxRudder;
	   }
	   
	   // Calculate turn rate from amount of rudder applied
	   // it is assumed that above a certain amount of rudder, turn rate will be constant
	   // TODO: Rate of change of rate of turn -> here it is assumed turn is yaw and rudders are vertical
	   turnRate = 0.2 * rudder;
	   
	   // Take account of max turn rate
	   if(turnRate > LookUpT.maxTurnRate) {
	      turnRate = LookUpT.maxTurnRate;
	   }
	   else if(turnRate < -LookUpT.maxTurnRate) {
	      turnRate = -LookUpT.maxTurnRate;
	   }

	   // Calculate new heading
	   newHeading = currentHeading + (turnRate * LookUpT.Tbase);
	   //System.out.println("ShipTurn-Rudder: " + rudder + "TurnRate: " + turnRate);
	   
	   return newHeading;
	}
	
	float rudder = 0.0;

    int keyCode = 0;

}

