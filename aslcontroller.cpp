#include "aslcontroller.h"

/**
 * Action-Sequence-Learning Controller for 
 * FourWheeldRPos_Gripper(Nimm4 with added sensors and gripper)
 * The controller gets a number of input sensor values each timestep
 * and has to generate a number of output motor values.
 *
 * Dominik Steven Weickgenannt (dowei14@student.sdu.dk 2015/2016)
 */
 
 
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

      // sensors 4-11: IR Sensors
      // sensor 4 = front right IR
      // sensor 5 = front left IR
      // sensor 6 = middle hind left IR
      // sensor 7 = middle front left IR
      // sensor 8 = hind left IR
      // sensor 9 = hind right IR
      // sensor 10 = middle hind right IR
      // sensor 11 = middle front right IR


      // sensors 12-35: distance to obstacles local coordinates (x,y,z)
      // sensor 12 = x direction to the first object (goal detection sensor)
      // sensor 13 = y direction to the first object (goal detection sensor)
      // sensor 14 = z direction to the first object (goal detection sensor)
      
      // 12-14 : Landmark 1
      // 15-17 : Landmark 2
      // 18-20 : Landmark 3
      // 21-23 : Landmark 4
      // 24-26 : Goal      
      // 27-29 : Box 1
      // 30-32 : Box 2
      // 33-35 : Box 3
      
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
      } else if (counter < 750) {
      	vehicle->removeAllGrippables();
      } else if (counter < 1000){
				speed = -1.0;
      } else {
      	speed = 1.0;
      }
      for (int i = 0; i < number_motors; i++){
        motors[i]=speed;
      }

    };
