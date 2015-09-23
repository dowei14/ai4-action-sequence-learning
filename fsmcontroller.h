#ifndef __FSMCONTROLLER_H
#define __FSMCONTROLLER_H


#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>

// DSW
#include <fourwheeledrpos_gripper.h>
#include <cmath> //pow sqrt


/*********************************************************************
***  Parameters
*********************************************************************/
	#define number_relative_sensors 8


/**
 * Action-Sequence-Learning Controller for 
 * FourWheeldRPos_Gripper(Nimm4 with added sensors and gripper)
 * The controller gets a number of input sensor values each timestep
 * and has to generate a number of output motor values.
 *
 * Dominik Steven Weickgenannt (dowei14@student.sdu.dk 2015/2016)
 */
class FSMController : public AbstractController {
  public:

    //Define global parameters-begin//
    std::vector<double> parameter;

    // DSW
	lpzrobots::FourWheeledRPosGripper* vehicle;
	std::vector<lpzrobots::Primitive*> grippables;
    double distances [number_relative_sensors];
	double angles [number_relative_sensors];
	double irSmooth[6];
	double smoothingFactor;
	// DSW temp stuff for testing
	int counter;
    double speed;
    double left,right;
    
    // DSW FSM Control
    bool done;
    int currentBox;
    int state;
    bool boxGripped;
    int testBoxCounter;
    int dropBoxCounter;
    int crossGapCounter;
    
    
    //Define global parameters-end//

    /// contructor (hint: use $ID$ for revision)
    FSMController(const std::string& name, const std::string& revision, 
    		lpzrobots::FourWheeledRPosGripper* vehicleIn, std::vector<lpzrobots::Primitive*> grippablesIn);

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

	// perform one step with learning
	virtual void step(const sensor* sensors, int sensornumber, motor* motors, int motornumber);

	// perform one step without learning
	virtual void stepNoLearning(const sensor* , int number_sensors,motor* , int number_motors);


	// DSW
	virtual void calculateDistanceToGoals(const sensor* x_);
	virtual void calculateAnglePositionFromSensors(const sensor* x_);

	// FSM - Behaviours
	virtual void setTarget();
	virtual bool goToRandomBox(double boxDistance, double boxAngle, motor* motors);
	virtual bool testBox(double boxDistance, motor* motors, int& testBoxCounter, bool& isGripped);
	virtual bool moveToEdge(double irLeft, double irRight, motor* motors);
	virtual bool orientAtEdge(double irLeftLong, double irRightLong, double irLeftShort, double irRightShort, motor* motors);
	virtual bool dropBox(lpzrobots::FourWheeledRPosGripper* vehicle, int& dropBoxCounter, bool& isGripped);
	virtual bool crossGap(motor* motors, int& crossGapCounter);
		
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

  protected:

    int number_sensors;
    int number_motors;

};

#endif
