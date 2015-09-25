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
	smoothingFactor = 1.0;
	reset = false;

	// DSW temp things
	counter = 0;
	speed = 1.0;

	// DSW Q-Learning
	state = 0;
	for(int i_s = 0; i_s <6; ++i_s) {
		for(int i_a = 0; i_a <6; ++i_a) {
			Q[i_a][i_s] = (double)(rand()%1000)/9000.0;
		}
	}
	learnRate = 0.7;
	exploreRate = 0.1;
	discountFactor = 0.9;
	reward = 0;
	j_RL = 0;
	r_RL = 0;
	a_RL = 0;
	a1_RL = 0;

	// things for plotting
	parameter.resize(4);
	addInspectableValue("parameter1", &parameter.at(0),"parameter1");
	addInspectableValue("parameter2", &parameter.at(1),"parameter2");
	addInspectableValue("parameter3", &parameter.at(2),"parameter3");
	addInspectableValue("parameter4", &parameter.at(3),"parameter4");
//  addInspectableValue("parameter3", &parameter.at(2),"parameter3");
//  addInspectableValue("parameter4", &parameter.at(3),"parameter4");
//  addInspectableValue("parameter5", &parameter.at(4),"parameter5");
//  addInspectableValue("parameter6", &parameter.at(5),"parameter6");
//  addInspectableValue("parameter7", &parameter.at(6),"parameter7");
//  addInspectableValue("parameter8", &parameter.at(7),"parameter8");
//  addInspectableValue("parameter9", &parameter.at(8),"parameter9");
//  addInspectableValue("parameter10", &parameter.at(9),"parameter10");
//  addInspectableValue("parameter11", &parameter.at(10),"parameter11");
//  addInspectableValue("parameter12", &parameter.at(11),"parameter12");
//  addInspectableValue("parameter13", &parameter.at(12),"parameter13");
//  addInspectableValue("parameter14", &parameter.at(13),"parameter14");
//  addInspectableValue("parameter15", &parameter.at(14),"parameter15");
//  addInspectableValue("parameter16", &parameter.at(15),"parameter16");


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

	// calculate relative distances and angles from sensor value, normalized to 0..1 for distance and -1..1 for angle
	calculateDistanceToGoals(sensors);
	calculateAnglePositionFromSensors(sensors);
			
	// smooth ir sensors
	for (int i=0;i<number_ir_sensors;i++) irSmooth[i] += (sensors[4+i]-irSmooth[i])/smoothingFactor;


//			for (int i=0;i<8;i++) parameter.at(2*i) = distances[i];
//			for (int i=0;i<8;i++) parameter.at((2*i)+1) = angles[i];


/***  // DSW forward for a bit then backforward for grasping test
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
*/

/********************************************************************************************
*** FSM
********************************************************************************************/

		parameter.at(0) = irSmooth[3];
		parameter.at(1) = irSmooth[2];
		parameter.at(2) = irSmooth[5];
		parameter.at(3) = irSmooth[4];	
		
		counter++;			
		if (state==0 && reset== false) {
			vehicle->addGrippables(grippables);
			done = false;
			boxGripped = false;
			testBoxCounter = 0;
			dropBoxCounter = 0;
			crossGapCounter = 0;
			setTarget();
			state++;
//			smoothingFactor = 1;
			atEdge = false;
		} else if (state==1){
			if(!done) done = goToRandomBox(distances[currentBox],angles[currentBox],motors);
			else {
				state++;
				done = false;
			}
		} else if (state==2){
			if (!done) done = testBox(distances[currentBox],motors,testBoxCounter, boxGripped);
			else {
				done = false;
				if (boxGripped) {
					state++;
					counter = 0;
				}else state=0;
			}
		} else if (state==3){
			if (!done) done = moveToEdge(irSmooth[3],irSmooth[2],motors);
			else {
				done = false;
				state++;
			}
		} else if (state==4){
			if (!done) done = orientAtEdge(irSmooth[3],irSmooth[2],irSmooth[5],irSmooth[4],motors, atEdge);
			else {				
				done = false;
				state++;
				dropBoxCounter = counter;
			}
		} else if (state==5){
			if (!done) done = dropBox(vehicle, dropBoxCounter, boxGripped);
			else {
				done = false;
				state++;
			}
		} else if (state==6){
			if (!done) done = crossGap(motors, crossGapCounter);
			else {
				done = false;
				state = 0;
				reset = true;
			}
		}

		

/********************************************************************************************
*** Q-Learning
********************************************************************************************/		

//	std::cout<<std::endl<<getState(sensors, boxGripped, atEdge, vehicle->getPosition())<<std::endl;
		parameter.at(0) = vehicle->getPosition().x;
		parameter.at(1) = vehicle->getPosition().y;
/*
      for (int i = 0; i < number_motors; i++){
        motors[i]=0.2;
      }	   


	state = getState(sensors, boxGripped, atEdge, vehicle->getPosition());
	if (state==6) {
		reward = 1;
		reset=true;
	}

	j_RL = state;
	r_RL = reward;
	a1_RL = getMaxAction(j_RL);
	Q[a_RL][i_RL] = Q[a_RL][i_RL] + LEARN_RATE * (r_RL + discount_factor*Q[a1_RL][j_RL] - Q[a_RL][i_RL]);
	exploration_activation = (double)(rand()%1000)/1000.0; 
	if(exploration_activation < EXPLORE_RATE)
		a_RL = getMaxAction(j_RL); //getMaxAction(j);
	else {
		a_RL = (int) (6.0*rand()/(RAND_MAX+1.0)); //between 0-2, 3 actions
	}


	=> loop for action execution.
*/




		
};


void ASLController::stepNoLearning(const sensor* , int number_sensors,motor* , int number_motors){
	
};

/*****************************************************************************************
*** Q-Learning Functions
*****************************************************************************************/
void ASLController::setTarget(){
	std::random_device rd; // obtain a random number from hardware
	std::mt19937 eng(rd()); // seed the generator
	std::uniform_int_distribution<> distr(5, 7); // define the range
	currentBox = distr(eng);
}

bool ASLController::goToRandomBox(double boxDistance, double boxAngle, motor* motors)
{
	double left,right;
	bool finished;
	if (boxDistance > 0.0008 ){
		left = 0.5 + boxAngle;
		right = 0.5 - boxAngle;
		finished = false;
	} else{
		left = 0.0;
		right = 0.0;
		finished =true;
	}	
	motors[0]=left; motors[2] = left;
	motors[1]=right; motors[3] = right;
	return finished;
}

bool ASLController::testBox(double boxDistance, motor* motors, int& testBoxCounter, bool& isGripped){
	double speed;
	bool done;
	if (testBoxCounter < 100){
		speed = -0.5;
		testBoxCounter++;
		done = false;
	} else {
		if (boxDistance > 0.0008) isGripped = false;
		else isGripped = true;
		done = true;
		speed = 0.0;
	}
	motors[0]=speed; motors[2] = speed;
	motors[1]=speed; motors[3] = speed;
	return done;
}

bool ASLController::moveToEdge(double irLeft, double irRight, motor* motors){
	bool done = false;
	double left,right;
	
	if (irLeft > 0.6 && irRight > 0.6){
		left = 0.3; right = 0.3;
	} else {
		left = 0.0;	right = 0.0;
		done = true;
	}
	
	motors[0]=left; motors[2] = left;
	motors[1]=right; motors[3] = right;
	return done;
}

bool ASLController::orientAtEdge(double irLeftLong, double irRightLong, double irLeftShort, double irRightShort, motor* motors, bool& atEdge){
	bool done = false;
	double left,right;
	double threshold = 0.5;
	double speed = 0.05;
	
	if( (irLeftLong < threshold) && (irRightLong < threshold) && (irLeftShort < threshold) && (irRightShort < threshold) ){
		left = -speed*2; right = -speed*2; // overshoot
	} else if ( (irLeftLong > threshold) && (irRightLong > threshold) && (irLeftShort > threshold) && (irRightShort > threshold) ){
		left = speed;		right = speed; // undershoot
	} else if ( (irLeftLong > threshold) && (irRightLong < threshold) && (irLeftShort > threshold) && (irRightShort < threshold) ){
		left = speed;		right = -speed; // turn right
	} else if ( (irLeftLong < threshold) && (irRightLong > threshold) && (irLeftShort < threshold) && (irRightShort > threshold)){
		left = -speed;	right = speed; // turn left
	} else if ( (irLeftLong < threshold) && (irRightLong > threshold) && (irLeftShort > threshold) && (irRightShort > threshold)){
		left = -speed/2;	right = speed; // turn left with slower back movement
	}else if ( (irLeftLong > threshold) && (irRightLong < threshold) && (irLeftShort > threshold) && (irRightShort > threshold)){
		left = speed;	right = -speed/2; // turn right with slower back movement
	} else {//if ( (irLeftLong < threshold) && (irRightLong > threshold) && (irLeftShort < threshold) && (irRightShort > threshold) ){
		left = 0.0;		right = 0.0;
		done = true;
		atEdge = true;
	}
	
	motors[0]=left; motors[2] = left;
	motors[1]=right; motors[3] = right;
	return done;
}

bool ASLController::dropBox(lpzrobots::FourWheeledRPosGripper* vehicle, int& dropBoxCounter, bool& isGripped){
	bool done = false;
	vehicle->removeGrippables(grippables);
	dropBoxCounter++;
	if (dropBoxCounter > 3500) {
		done = true;
		isGripped = false;
	}
	return done;
}

bool ASLController::crossGap(motor* motors, int& crossGapCounter){
	bool done = false;
	speed = 0.5;
	crossGapCounter++;
	if (crossGapCounter > 500) {
		done = true;
		speed = 0.0;
	}
	motors[0]=speed; motors[2] = speed;
	motors[1]=speed; motors[3] = speed;
	return done;
}

int ASLController::getState(const sensor* sensors, bool& isGripped, bool& atEdge, Position pos){
	double x,y;
	x=abs(pos.x); y=abs(pos.y);
	int state =-1;

	if ((x<=9) && (y<=9)){
		if(sensors[5]<0.8 && sensors[6]<0.8){
			state=0;
		} else {
			if (isGripped) state=2;
			else state=1;
		}
	}

	if (((x>9) && (x<9.5)) | ((y > 9) && (y < 9.5))) {
		if (isGripped) {
			if (atEdge)	state=4;
			else state=3;
		}
	}

	if ((x>11) | (y > 11)) state =5;

	return state;
}

int ASLController::getMaxAction(int state){
	// find the largest Q-value for a given state (j), and return action
	float max = -1000;
	int action = 0;
	for(int a=0;a<6;++a) { // Find MAX Q of all actions in this state!
		if(Q[a][state] > max) {
		max = Q[a][state];
		action = a;
		}
	}
	return action;
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
