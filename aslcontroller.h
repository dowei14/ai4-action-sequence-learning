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

		// perform one step with 
virtual void step(const sensor* sensors, int sensornumber, motor* motors, int motornumber);

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
