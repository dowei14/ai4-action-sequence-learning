#include "aslcontroller.h"

/**
 * Action-Sequence-Learning Controller for 
 * FourWheeldRPos_Gripper(Nimm4 with added sensors and gripper)
 * The controller gets a number of input sensor values each timestep
 * and has to generate a number of output motor values.
 *
 * Dominik Steven Weickgenannt (dowei14@student.sdu.dk 2015/2016)
 */
 

ASLController::ASLController(const std::string& name, const std::string& revision)
	: AbstractController(name, revision){

	// DSW
	for (int i=0;i<number_ir_sensors;i++) irSmooth[i]=0;


		
	testBoxCounter = 0;
	dropBoxCounter = 0;
	crossGapCounter = 0;
	atEdge = false;
	haveTarget = false;
	prevhaveTarget = false;
	boxGripped = false;
	dropStuff = false;
	done = false;
	reset = false;
	counter = 0;
	// parameters
	boxTouching = 0.00115;
	irFloorDistance = 0.5;
	irFrontClearDistance = 0.8;
	smoothingFactor = 1.0;
	state = 0;
	runNumber = 0;
	currentBox = 5;
	prevMotorLeft = 0;
	prevMotorRight = 0;

	// things for plotting
	parameter.resize(8);
	addInspectableValue("parameter1", &parameter.at(0),"parameter1");
	addInspectableValue("parameter2", &parameter.at(1),"parameter2");
	addInspectableValue("parameter3", &parameter.at(2),"parameter3");
	addInspectableValue("parameter4", &parameter.at(3),"parameter4");
	addInspectableValue("parameter5", &parameter.at(4),"parameter5");
	addInspectableValue("parameter6", &parameter.at(5),"parameter6");
	addInspectableValue("parameter7", &parameter.at(6),"parameter7");
	addInspectableValue("parameter8", &parameter.at(7),"parameter8");	


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

	// sensors 4-9: IR Sensors
	// sensor 4 = front middle right IR
	// sensor 5 = front middle left IR
	// sensor 6 = front right long range IR
	// sensor 7 = front left long range IR
	// sensor 8 = front right short range IR
	// sensor 9 = front left short range IR

	// sensors 10-33: distance to obstacles local coordinates (x,y,z)
	// sensor 10 = x direction to the first object (goal detection sensor)
	// sensor 11 = y direction to the first object (goal detection sensor)
	// sensor 12 = z direction to the first object (goal detection sensor)
      
	// 10-12 : Landmark 1		(0)
	// 13-15 : Landmark 2		(1)
	// 16-18 : Landmark 3		(2)
	// 19-21 : Landmark 4		(3)
	// 22-24 : Goal      		(4)
	// 25-27 : Box 1			(5)
	// 28-30 : Box 2			(6)
	// 31-33 : Box 3			(7)
      
	/*****************************************************************************************/
	// add grippables
	vehicle->addGrippables(grippables);
	
	// calculate relative distances and angles from sensor value, normalized to 0..1 for distance and -1..1 for angle
	calculateDistanceToGoals(sensors);
	calculateAnglePositionFromSensors(sensors);
			
	// smooth ir sensors
	for (int i=0;i<number_ir_sensors;i++) irSmooth[i] += (sensors[4+i]-irSmooth[i])/smoothingFactor;




/********************************************************************************************
*** store previous values
********************************************************************************************/

	prevMotorLeft = motors[0];
	prevMotorRight = motors[1];
	prevState = state;
	prevhaveTarget = haveTarget;
	
/********************************************************************************************
*** FSM
********************************************************************************************/



	distanceCurrentBox = distances[currentBox];
	angleCurrentBox = angles[currentBox];
	irLeftLong = irSmooth[3];
	irRightLong = irSmooth[2];
	irLeftShort = irSmooth[5];
	irRightShort = irSmooth[4];
	irFrontLeft = irSmooth[1];
	irFrontRight = irSmooth[0];
	
	parameter.at(0) = irLeftLong;
	parameter.at(1) = irRightLong;
	parameter.at(2) = irLeftShort;
	parameter.at(3) = irRightShort;	
	parameter.at(4) = irFrontLeft;
	parameter.at(5) = irFrontRight;	
	parameter.at(6) = distanceCurrentBox;	
	parameter.at(7) = angleCurrentBox;			


	if (reset){		
		haveTarget = false;
		prevhaveTarget = false;
		boxGripped = false;
		dropStuff = false;
		testBoxCounter = 0;
		dropBoxCounter = 0;
		crossGapCounter = 0;
		state = 0;
		counter =0;
	} else if (counter > 1) {
		// determine new state
		if (state==0) {
			if (prevhaveTarget)	state++;
		} else if (state==1){
			if (distanceCurrentBox <= boxTouching ){
				state++;
			}
		} else if (state==2){
			if (testBoxCounter < 100)
				testBoxCounter++;
			else {
				testBoxCounter = 0;
				if (distanceCurrentBox > boxTouching) {
					boxGripped = false; // not being used
					haveTarget = false;
					prevhaveTarget = false;
					state = 0;
				} else {
					boxGripped = true; // not being used
					state++;
//					state=7;
				}
			}
		} else if (state ==3){
			if (irLeftLong < irFloorDistance || irRightLong < irFloorDistance) state++;
		} else if (state ==4){
			if ((irLeftLong < irFloorDistance) && (irRightLong < irFloorDistance) && ((irLeftShort > irFloorDistance) || (irRightShort > irFloorDistance))) {
				state++;
				motors[0]=0.0; motors[2] = 0.0;
				motors[1]=0.0; motors[3] = 0.0;
			}
		} else if (state ==5){
			if (irFrontLeft < irFrontClearDistance){		
				boxGripped = false;
				state++;
			}
		} else if (state ==6){
			if (irFrontLeft > irFrontClearDistance){
				state++;
			}
		} else {		
			reset = true;
			runNumber++;
		}
			
		
		// execute action
		if (state==0) {
			getTargetAction = true;
			setTarget(haveTarget);
		} else if (state==1){
			getTargetAction = false;
			goToRandomBox(distanceCurrentBox,angleCurrentBox,motors);
		} else if (state==2){
			testBox(distanceCurrentBox,motors,testBoxCounter, boxGripped);
		} else if (state==3){
			moveToEdge(irLeftLong,irRightLong,motors);
		} else if (state==4){
			orientAtEdge(irLeftLong,irRightLong,irLeftShort,irRightShort,motors, atEdge);
		} else if (state==5){
			dropBox(vehicle, dropBoxCounter, dropStuff, boxGripped);
		} else if (state==6){
			crossGap(motors, crossGapCounter);
		}
	}
	for (int i=0;i<4;i++){
	if (motors[i]>1.0) motors[i]=1.0;
	if (motors[i]<-1.0) motors[i]=-1.0;
	
	}
		
	motorLeft = motors[0]; motorRight = motors[1];
	if (!reset && runNumber>0 && counter>1) {
		store();
		//storeBySkillCSMTL();
	}
	counter++;

		
};


void ASLController::stepNoLearning(const sensor* , int number_sensors,motor* , int number_motors){
	
};

/*****************************************************************************************
*** Q-Learning Functions
*****************************************************************************************/
void ASLController::setTarget(bool& haveTarget){
	std::random_device rd; // obtain a random number from hardware
	std::mt19937 eng(rd()); // seed the generator
	std::uniform_int_distribution<> distr(5, 7); // define the range
	currentBox = distr(eng);
	haveTarget = true;
}

bool ASLController::goToRandomBox(double boxDistance, double boxAngle, motor* motors)
{
	double left,right;
	bool done = false;
	left = 0.5 + boxAngle;
	right = 0.5 - boxAngle;
	motors[0]=left; motors[2] = left;
	motors[1]=right; motors[3] = right;
	return done;
}

bool ASLController::testBox(double boxDistance, motor* motors, int& testBoxCounter, bool& isGripped){
	double speed;
	bool done = false;
	speed = -0.5;
	motors[0]=speed; motors[2] = speed;
	motors[1]=speed; motors[3] = speed;
	return done;
}

bool ASLController::moveToEdge(double irLeft, double irRight, motor* motors){
	bool done = false;
	double left,right;
	
	left = 0.3; right = 0.3;

	motors[0]=left; motors[2] = left;
	motors[1]=right; motors[3] = right;
	return done;
}

bool ASLController::orientAtEdge(double irLeftLong, double irRightLong, double irLeftShort, double irRightShort, motor* motors, bool& atEdge){
	bool done = false;
	double left = 0.0;
	double right = 0.0;
	double threshold = 0.5;
	double speed = 0.05;
	if( (irLeftLong < threshold) && (irRightLong < threshold) && ((irLeftShort < threshold) || (irRightShort < threshold)) ){
		left = -speed*2; right = -speed*2; // overshoot
	} else if ( (irLeftLong > threshold) && (irRightLong > threshold) && (irLeftShort > threshold) && (irRightShort > threshold) ){
		left = speed;		right = speed; // undershoot
	} else if ( (irLeftLong > threshold) && (irRightLong < threshold) && (irLeftShort > threshold) && (irRightShort < threshold) ){
		left = speed;		right = -speed; // turn right
	} else if ( (irLeftLong < threshold) && (irRightLong > threshold) && (irLeftShort < threshold) && (irRightShort > threshold)){
		left = -speed;	right = speed; // turn left
	} else if ( (irLeftLong < threshold) && (irRightLong > threshold) && (irLeftShort > threshold) && (irRightShort > threshold)){
		left = -speed/2;	right = speed; // turn left with slower back movement
	} else if ( (irLeftLong > threshold) && (irRightLong < threshold) && (irLeftShort > threshold) && (irRightShort > threshold)){
		left = speed;	right = -speed/2; // turn right with slower back movement
	} 

	motors[0]=left; motors[2] = left;
	motors[1]=right; motors[3] = right;
	return done;
}

bool ASLController::dropBox(lpzrobots::FourWheeledRPosGripper* vehicle, int& dropBoxCounter, bool& dropStuff, bool& isGripped){
	bool done = false;	

	// the counter is for the robot to stop moving before dropping, otherwise it will fuck up from time to time
	// TODO: replace this with a "sensor based option"
	dropBoxCounter++;
	if (dropBoxCounter > 50) {
		dropStuff = true;
		vehicle->removeGrippables(grippables);
	}
	return done;
}

bool ASLController::crossGap(motor* motors, int& crossGapCounter){
	bool done = false;
	double speed = 0.5;
	motors[0]=speed; motors[2] = speed;
	motors[1]=speed; motors[3] = speed;
	return done;
}


/*****************************************************************************************
*** Pass grippables and vehicle after reset
*****************************************************************************************/
void ASLController::setGrippablesAndVehicle(lpzrobots::FourWheeledRPosGripper* vehicleIn, std::vector<lpzrobots::Primitive*> grippablesIn){
	vehicle = vehicleIn;
	grippables = grippablesIn;
}

/*****************************************************************************************
*** Sensor Data => relative angle/distance
*****************************************************************************************/
//calculate distances
void ASLController::calculateDistanceToGoals(const sensor* x_)
{
	double distance_scale = 0.0005; // should normalize it to 0..1
	double distance;
	for (int i=0; i<number_relative_sensors; i++){
		distance = (sqrt(pow(x_[11+(i*3) ],2)+pow(x_[10+(i*3)],2)));
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
		if (sign(x_[10 + i])>0)
			alpha_value = atan(x_[11 + i]/x_[10 + i]) * 180 / M_PI; // angle in degrees
		else {
			double alpha_value_tmp = -1*atan (x_[11 + i]/x_[10 + i]) * 180 / M_PI; // angle in degrees
			if (alpha_value_tmp<=0)
				alpha_value = (-90 + (-90-alpha_value_tmp));
			else alpha_value = ( 90 + ( 90-alpha_value_tmp));
		}
		angles[counter] = alpha_value/180.*M_PI / M_PI;
		i+=3;
	}
}

void ASLController::store(){
//	std::string inRNNname = "../data/inRNN" + std::to_string(runNumber) + ".txt";
	std::string inRNNname = "../data/inRNN_18.txt";
	inRNN.open (inRNNname.c_str(), ios::app);
	inRNN.precision(5);
	inRNN<<fixed;
	
	std::string inRNNname2 = "../data/inRNN_11.txt";
	inRNN2.open (inRNNname2.c_str(), ios::app);
	inRNN2.precision(5);
	inRNN2<<fixed;
	
//	std::string outRNNname = "../data/outRNN" + std::to_string(runNumber) + ".txt";
	std::string outRNNname = "../data/outRNN.txt";	
	outRNN.open (outRNNname.c_str(), ios::app);
	outRNN.precision(5);
	outRNN<<fixed;
/*	
	std::string inCSMTLname = "../data/inCSMTL" + std::to_string(runNumber) + ".txt";
	inCSMTL.open (inCSMTLname.c_str(), ios::app);
	inCSMTL.precision(5);
	inCSMTL<<fixed;
	
	std::string outCSMTLname = "../data/outCSMTL" + std::to_string(runNumber) + ".txt";
	outCSMTL.open (outCSMTLname.c_str(), ios::app);
	outCSMTL.precision(5);
	outCSMTL<<fixed;
*/
	int repetitions = 1;
	if (prevState != state) repetitions= 100;
	for (int a=0;a<repetitions;a++){
		for (int i=0;i<7;i++){
	//	for (int i=0;i<3;i++){
			if (i == prevState)	inRNN<<"1";
			else inRNN<<"0";
			inRNN<<" ";
		
			if (i == state)	outRNN<<"1";
			else outRNN<<"0";
			if (i<6) outRNN<<" ";
			if (i==6) outRNN<<"\n";
	//		if (i<2) outRNN<<" ";
	//		if (i==2) outRNN<<"\n";

		
	//		if (i == state)	inCSMTL<<"1";
	//		else inCSMTL<<"0";
	//		if (i<6) inCSMTL<<" ";
		}
	

		inRNN<<prevMotorLeft<<" "<<prevMotorRight;	
		inRNN<<" ";
		inRNN<<distanceCurrentBox<<" "<<angleCurrentBox;
		inRNN<<" ";
		inRNN<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFrontLeft<<" "<<irFrontRight;
		inRNN<<" ";
		if (prevhaveTarget) inRNN<<"1";
		else inRNN<<"0";
		inRNN<<"\n";
	
		inRNN2<<prevMotorLeft<<" "<<prevMotorRight;	
		inRNN2<<" ";
		inRNN2<<distanceCurrentBox<<" "<<angleCurrentBox;
		inRNN2<<" ";
		inRNN2<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFrontLeft<<" "<<irFrontRight;
		inRNN2<<" ";
		if (prevhaveTarget) inRNN2<<"1";
		else inRNN2<<"0";
		inRNN2<<"\n";
	}
/*	
	inCSMTL<<" ";
	inCSMTL<<prevMotorLeft<<" "<<prevMotorRight;	
	inCSMTL<<" ";
	inCSMTL<<distanceCurrentBox<<" "<<angleCurrentBox;
	inCSMTL<<" ";
	inCSMTL<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFrontLeft<<" "<<irFrontRight;
	inCSMTL<<"\n";
	if (prevhaveTarget) inCSMTL<<"1";
	else inCSMTL<<"0";
	
	outCSMTL<<motorLeft<<" "<<motorRight;
	if (getTargetAction) outCSMTL<<" "<<"1";
	else outCSMTL<<" "<<"0";
	outCSMTL<<"\n";
*/	
	
  	inRNN.close();
  	inRNN2.close();	
	outRNN.close();
  	inCSMTL.close();  	
	outCSMTL.close();
}

void ASLController::storeBySkillCSMTL(){
	
	std::string inCSMTLname = "../data/stateIn" + std::to_string(state) + ".txt";
	inCSMTL.open (inCSMTLname.c_str(), ios::app);
	inCSMTL.precision(5);
	inCSMTL<<fixed;
	
	std::string outCSMTLname = "../data/stateOut" + std::to_string(state) + ".txt";
	outCSMTL.open (outCSMTLname.c_str(), ios::app);
	outCSMTL.precision(5);
	outCSMTL<<fixed;
	
	for (int i=0;i<7;i++){

		if (i == state)	inCSMTL<<"1";
		else inCSMTL<<"0";
		if (i<6) inCSMTL<<" ";
	}


	inCSMTL<<" ";
	inCSMTL<<prevMotorLeft<<" "<<prevMotorRight;	
	inCSMTL<<" ";
	inCSMTL<<distanceCurrentBox<<" "<<angleCurrentBox;
	inCSMTL<<" ";
	inCSMTL<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFrontLeft<<" "<<irFrontRight;
	inCSMTL<<"\n";
	if (prevhaveTarget) inCSMTL<<"1";
	else inCSMTL<<"0";
	
	outCSMTL<<motorLeft<<" "<<motorRight;
	if (getTargetAction) outCSMTL<<" "<<"1";
	else outCSMTL<<" "<<"0";
	outCSMTL<<"\n";

  	inCSMTL.close();  	
	outCSMTL.close();
}
