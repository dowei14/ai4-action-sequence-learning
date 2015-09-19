#include "aslcontroller.h"

/**
 * Action-Sequence-Learning Controller for 
 * FourWheeldRPos_Gripper(Nimm4 with added sensors and gripper)
 * The controller gets a number of input sensor values each timestep
 * and has to generate a number of output motor values.
 *
 * Dominik Steven Weickgenannt (dowei14@student.sdu.dk 2015/2016)
 */
 
 

ASLController::ASLController(const std::string& name, const std::string& revision, 
		lpzrobots::FourWheeledRPosGripper* vehicleIn, std::vector<lpzrobots::Primitive*> grippablesIn)
	: AbstractController(name, revision){

  // DSW
  counter = 0;
  speed = 1.0;
  vehicle = vehicleIn;
	grippables = grippablesIn;

	// things for plotting
  parameter.resize(2);
  addInspectableValue("parameter1", &parameter.at(0),"parameter1");
  addInspectableValue("parameter2", &parameter.at(1),"parameter2");

}


/*************************************************************************************************
*** performs one step (includes learning).
*** Calculates motor commands from sensor inputs.
***   @param sensors sensors inputs scaled to [-1,1]
***   @param sensornumber length of the sensor array
***   @param motors motors outputs. MUST have enough space for motor values!
***   @param motornumber length of the provided motor array
*************************************************************************************************/
void ASLController::step(const sensor* sensors, int sensornumber,
      motor* motors, int motornumber){
      assert(number_sensors == sensornumber);
      assert(number_motors == motornumber);


      /*****************************************************************************************/
      // motors 0-4
      // motor 0 = left front motor
      // motor 1 = right front motor
      // motor 2 = left hind motor
      // motor 3 = right hind motor

      // sensors 0-3: wheel velocity of the corresponding wheel
      // sensor 0 = wheel velocity left front
      // sensor 1 = wheel velocity right front
      // sensor 2 = wheel velocity left hind
      // sensor 3 = wheel velocity right hind

      // sensors 4-7: IR Sensors
      // sensor 4 = front right IR
      // sensor 5 = front middle right IR
      // sensor 6 = front middle left IR
      // sensor 7 = front left IR

      // sensors 8-31: distance to obstacles local coordinates (x,y,z)
      // sensor  8 = x direction to the first object (goal detection sensor)
      // sensor  9 = y direction to the first object (goal detection sensor)
      // sensor 10 = z direction to the first object (goal detection sensor)
      
      //  8-10 : Landmark 1
      // 11-13 : Landmark 2
      // 14-16 : Landmark 3
      // 17-19 : Landmark 4
      // 20-22 : Goal      
      // 23-25 : Box 1
      // 27-28 : Box 2
      // 31-31 : Box 3
      
      /*****************************************************************************************/


      parameter.at(0) = sensors[4];
      parameter.at(1) = sensors[5];



      // DSW forward for a bit then backforward for grasping test
      counter++;
      if (counter < 420) {
	      vehicle->addGrippables(grippables);
				speed = 1.0;
      } else if (counter < 700) {       
				speed = 0.0;
      } /*else if (counter < 750) {
      	vehicle->removeAllGrippables();
      } else if (counter < 1000){
				speed = -1.0;
      } else {
      	speed = 1.0;
      }
*/
      for (int i = 0; i < number_motors; i++){
        motors[i]=speed;
      }

};


void ASLController::stepNoLearning(const sensor* , int number_sensors,motor* , int number_motors){
	
};

//calculate distances
void ASLController::calculateDistanceToGoals(const sensor* x_)
{
/*	double distance_scale = 0.001;//1/100;

	for (int i=0; i<number_relative_sensors; i++){
		distance = (sqrt(pow(x_[13+(i*3) ],2)+pow(x_[12+(i*3)],2)));
		distances[i] = distance*distance_scale;//85;//max_dis;
	}
*/
}

