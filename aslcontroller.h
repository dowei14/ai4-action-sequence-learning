#ifndef __ASLCONTROLLER_H
#define __ASLCONTROLLER_H


#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>

// DSW
#include <fourwheeledrpos_gripper.h>

/**
 * Action-Sequence-Learning Controller for 
 * FourWheeldRPos_Gripper(Nimm4 with added sensors and gripper)
 * The controller gets a number of input sensor values each timestep
 * and has to generate a number of output motor values.
 *
 * Dominik Steven Weickgenannt (dowei14@student.sdu.dk 2015/2016)
 */
class ASLController : public AbstractController {
  public:

    //Define global parameters-begin//
    std::vector<double> parameter;
    double distance2;
    double distance3;
    double mc[4];

    // DSW
    int counter;
    double speed;
    lpzrobots::FourWheeledRPosGripper* vehicle;
    std::vector<lpzrobots::Primitive*> grippables;
    
    //Define global parameters-end//

    /// contructor (hint: use $ID$ for revision)
    ASLController(const std::string& name, const std::string& revision, 
    		lpzrobots::FourWheeledRPosGripper* vehicleIn, std::vector<lpzrobots::Primitive*> grippablesIn)
    : AbstractController(name, revision){

      //For students, Initialization -begin//
      parameter.resize(2);
      //For students, Initialization -end//


      // DSW
      counter = 0;
      speed = 1.0;
      vehicle = vehicleIn;
			grippables = grippablesIn;

      //plot values on GUI, ./start -g 1
      addInspectableValue("parameter1", &parameter.at(0),"parameter1");
      addInspectableValue("parameter2", &parameter.at(1),"parameter2");


    }

    /** initialization of the controller with the given sensor/ motornumber
      Must be called before use. The random generator is optional.
     */
    virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0){
      number_sensors = sensornumber;
      number_motors = motornumber;
    };

    /** @return Number of sensors the controller
      was initialised with or 0 if not initialised */
    virtual int getSensorNumber() const {
      return number_sensors;
    };

    /** @return Number of motors the controller
      was initialised with or 0 if not initialised */
    virtual int getMotorNumber() const {
      return number_motors;
    };

/*************************************************************************************************
*** performs one step (includes learning).
*** Calculates motor commands from sensor inputs.
***   @param sensors sensors inputs scaled to [-1,1]
***   @param sensornumber length of the sensor array
***   @param motors motors outputs. MUST have enough space for motor values!
***   @param motornumber length of the provided motor array
*************************************************************************************************/
    virtual void step(const sensor* sensors, int sensornumber,
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


      // sensors 12-23: distance two objects in local coordinates (x,y,z)
      // sensor 12 = x direction to the red object (goal detection sensor)
      // sensor 13 = y direction to the red object (goal detection sensor)
      // sensor 14 = z direction to the red object (goal detection sensor)

      // sensor 15 = x direction to the green object (goal detection sensor)
      // sensor 16 = y direction to the green object (goal detection sensor)
      // sensor 17 = z direction to the green object (goal detection sensor)

      // sensor 18 = x direction to the blue object (goal detection sensor)
      // sensor 19 = y direction to the blue object (goal detection sensor)
      // sensor 20 = z direction to the blue object (goal detection sensor)

      // sensor 21 = x direction to the yellow object (goal detection sensor)
      // sensor 22 = y direction to the yellow object (goal detection sensor)
      // sensor 23 = z direction to the yellow object (goal detection sensor)
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

    /** performs one step without learning.
      @see step
     */
    virtual void stepNoLearning(const sensor* , int number_sensors,
        motor* , int number_motors){

    };



    /********* STORABLE INTERFACE ******/
    /// @see Storable
    virtual bool store(FILE* f) const {
      Configurable::print(f,"");
      return true;
    }

    /// @see Storable
    virtual bool restore(FILE* f) {
      Configurable::parse(f);
      return true;
    }


    virtual void setMC(double left, double right){
      mc[0]=left;
      mc[1]=right;
      mc[2]=left;
      mc[3]=right;
    }

  protected:

    int number_sensors;
    int number_motors;

};

#endif
