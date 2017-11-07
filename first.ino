#include <AP_Compass.h> // Compass Library


AP_Compass_HMC5843 compass;

int heading_deg = 0;
double pi=3.141592653589793238;
double halfpi=1.570796326794896619;
int Tbase = 1;
	
int MAX = 10;              
int MIN = -10;
float Kp = 0.1;
float Kd = 0.01;
float Ki = 0.005;

	
int maxTurnRate = 2;
int maxRudder = 8;

const float angle = 22.5;
const float tol = 0.9;

float controllerOutput;
float headingerror;

float previousError = 0.0;
float controllerOutput = 0.0;


void setup ()
{
    compass.set_offsets(0,0,0); // set offsets to account for surrounding interference
    compass.set_declination(ToRad(0.0)); // set local difference between magnetic north and true north
}

void loop()
{
     float min[3], max[3], offset[3];

    compass.accumulate();
    compass.read();  
    //float heading;
    
        if (!compass.healthy()) {
 
            return;
        }
        
	Matrix3f dcm_matrix;
	// use roll = 0, pitch = 0 for this example
	dcm_matrix.from_euler(0, 0, 0);
        heading = compass.calculate_heading(dcm_matrix);
        heading_deg = ToDeg(heading);
        compass.learn_offsets();

       const Vector3f &mag = compass.get_field();
        if( mag.x < min[0] )
            min[0] = mag.x;
        if( mag.y < min[1] )
            min[1] = mag.y;
        if( mag.z < min[2] )
            min[2] = mag.z;

        // capture max
        if( mag.x > max[0] )
            max[0] = mag.x;
        if( mag.y > max[1] )
            max[1] = mag.y;
        if( mag.z > max[2] )
            max[2] = mag.z;

        // calculate offsets
        offset[0] = -(max[0]+min[0])/2;
        offset[1] = -(max[1]+min[1])/2;
        offset[2] = -(max[2]+min[2])/2;
        
        
        float ultra_meter=random(10);
        int which_ultra_is_on=random(8);// 22.5 degree
        
        change_rudder_to_sensed_ultra(which_ultra_is_on, heading_deg);
}


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
  // Deal with the 360 = 0 problem
  headingerror = heading_deg - required;

  if (abs(headingerror) < tol){
   return;
   }
        
        // Compute controller output
        controllerOutput = pid_func(headingerror, previousError, Ki, Kp, Kd, Tbase, MAX, MIN);
        
        
        // Calculate new heading      
	//float ruddersign = rudder_fpid(headingerror);
        float ruddersign=1;
        heading_deg = shipTurn(heading_deg, ruddersign, controllerOutput);
       
        // Calculate error
        previousError = headingerror;
        headingerror = heading_deg - required;
        
        //System.out.println("Heading: " + heading_deg + " Error: "+ headingerror + " Controller: "+ controllerOutput);
        //System.out.println("params[0]: " + params[0] + " params[1]: "+ params[1] + " params[2]: "+ params[2]);

        //printf("%d,%f,%f\n", i, heading, error);

}

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
	
	   // Calculate rudder position from controller output
	   rudder = -maxRudder * controllerOut;
	   //rudder = -ruddersign  * controllerOut;
	   
	   // Take account of rudder end stops
	   if(rudder > maxRudder) {
	      rudder = maxRudder;
	   }
	   else if(rudder < -maxRudder) {
	      rudder = -maxRudder;
	   }
	   
	   // Calculate turn rate from amount of rudder applied
	   // it is assumed that above a certain amount of rudder, turn rate will be constant
	   // TODO: Rate of change of rate of turn -> here it is assumed turn is yaw and rudders are vertical
	   turnRate = 0.2 * rudder;
	   
	   // Take account of max turn rate
	   if(turnRate > maxTurnRate) {
	      turnRate = maxTurnRate;
	   }
	   else if(turnRate < -maxTurnRate) {
	      turnRate = -maxTurnRate;
	   }

	   // Calculate new heading
	   newHeading = currentHeading + (turnRate * Tbase);
	   //System.out.println("ShipTurn-Rudder: " + rudder + "TurnRate: " + turnRate);
	   
	   return newHeading;
	}
	
    float rudder = 0.0;
    int keyCode = 0;


