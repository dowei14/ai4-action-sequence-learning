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
  vehicle = vehicleIn;
	grippables = grippablesIn;

  // DSW temp things
  counter = 0;
  speed = 1.0;

	// things for plotting
  parameter.resize(16);
  addInspectableValue("parameter1", &parameter.at(0),"parameter1");
  addInspectableValue("parameter2", &parameter.at(1),"parameter2");
  addInspectableValue("parameter3", &parameter.at(2),"parameter3");
  addInspectableValue("parameter4", &parameter.at(3),"parameter4");
  addInspectableValue("parameter5", &parameter.at(4),"parameter5");
  addInspectableValue("parameter6", &parameter.at(5),"parameter6");
  addInspectableValue("parameter7", &parameter.at(6),"parameter7");
  addInspectableValue("parameter8", &parameter.at(7),"parameter8");
  addInspectableValue("parameter9", &parameter.at(8),"parameter9");
  addInspectableValue("parameter10", &parameter.at(9),"parameter10");
  addInspectableValue("parameter11", &parameter.at(10),"parameter11");
  addInspectableValue("parameter12", &parameter.at(11),"parameter12");
  addInspectableValue("parameter13", &parameter.at(12),"parameter13");
  addInspectableValue("parameter14", &parameter.at(13),"parameter14");
  addInspectableValue("parameter15", &parameter.at(14),"parameter15");
  addInspectableValue("parameter16", &parameter.at(15),"parameter16");


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
      
      //  8-10 : Landmark 1		(0)
      // 11-13 : Landmark 2		(1)
      // 14-16 : Landmark 3		(2)
      // 17-19 : Landmark 4		(3)
      // 20-22 : Goal      		(4)
      // 23-25 : Box 1				(5)
      // 27-28 : Box 2				(6)
      // 31-31 : Box 3				(7)
      
      /*****************************************************************************************/

			// calculate relative distances and angles from sensor value, normalized to 0..1 for distance and -1..1 for angle
			calculateDistanceToGoals(sensors);
			calculateAnglePositionFromSensors(sensors);

//			for (int i=0;i<8;i++) parameter.at(2*i) = distances[i];
//			for (int i=0;i<8;i++) parameter.at((2*i)+1) = angles[i];

      // DSW forward for a bit then backforward for grasping test
      counter++;
      if (counter < 420) {
	      vehicle->addGrippables(grippables);
				speed = 1.0;
      } else if (counter < 700) {       
				speed = 0.0;
      } else if (counter < 1200) {
      	vehicle->removeAllGrippables();
      } else if (counter < 1300){
				speed = -1.0;
      } else {
      	speed = 1.0;
      }

      for (int i = 0; i < number_motors; i++){
        motors[i]=speed;
      }	    

};


void ASLController::stepNoLearning(const sensor* , int number_sensors,motor* , int number_motors){
	
};

//calculate distances
void ASLController::calculateDistanceToGoals(const sensor* x_)
{
	double distance_scale = 0.0005; // should normalize it to 0..1
  double distance;
	for (int i=0; i<number_relative_sensors; i++){
		distance = (sqrt(pow(x_[9+(i*3) ],2)+pow(x_[8+(i*3)],2)));
		distances[i] = distance * distance_scale;//85;//max_dis;
	}
}

//calculate relative angles
void ASLController::calculateAnglePositionFromSensors(const sensor* x_)
{
	int i=0;
	for (int counter=0; counter<number_relative_sensors; ++counter)
	{
		double alpha_value = 0;
		if (sign(x_[8 + i])>0)
			alpha_value = atan(x_[9 + i]/x_[8 + i]) * 180 / M_PI; // angle in degrees
		else {
			double alpha_value_tmp = -1*atan (x_[9 + i]/x_[8 + i]) * 180 / M_PI; // angle in degrees
			if (alpha_value_tmp<=0)
				alpha_value = (-90 + (-90-alpha_value_tmp));
			else alpha_value = ( 90 + ( 90-alpha_value_tmp));
		}
		angles[counter] = alpha_value/180.*M_PI / M_PI;
		i+=3;
	}
}

