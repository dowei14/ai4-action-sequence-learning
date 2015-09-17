#include <stdio.h>

//RANDOM
#include <stdlib.h>  /* RANDOM_MAX */

// include ode library
#include <ode-dbl/ode.h>

// include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/noisegenerator.h>

// include simulation environment stuff
#include <ode_robots/simulation.h>

// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>

// used wiring
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>

// used robot...
#include <ode_robots/nimm2.h>
#include <ode_robots/nimm4.h>
#include <fourwheeledrpos_gripper.h>


#include <selforg/trackrobots.h>
// used arena
#include <ode_robots/playground.h>
// used passive spheres
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>

// controller
#include "aslcontroller.h"

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;


ASLController* qcontroller;


// obstacles and spheres
int number_spheres = 0; // set number of spheres for relative sensors
std::vector<AbstractObstacle*> obst;


// DSW
std::vector<Primitive*> grippables;


class ThisSim : public Simulation {
public:

	// starting function (executed once at the beginning of the simulation loop)
	void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
	{

		// set initial camera position
		setCameraHomePos(Pos(0, 5, 5),  Pos(0, 0, 0));

		// initialization simulation parameters

		//1) - set noise to 0.1
		global.odeConfig.noise= 0.02;//0.05;

		//2) - set controlinterval -> default = 1
		global.odeConfig.setParam("controlinterval", 1);/*update frequency of the simulation ~> amos = 20*/
		//3) - set simulation setp size
		global.odeConfig.setParam("simstepsize", 0.01); /*stepsize of the physical simulation (in seconds)*/
		//Update frequency of simulation = 1*0.01 = 0.01 s ; 100 Hz

		//4) - set gravity if not set then it uses -9.81 =earth gravity
		//global.odeConfig.setParam("gravity", -9.81);

/**************************************************************************************************
***			Set up Environment
**************************************************************************************************/
		//5) - set Playground as boundary:
    // inner Platform
		double length_pg = 0.0; //45, 32, 22
		double width_pg = 10.0;//0.0;  //0.2
		double height_pg = 1.0;//0.0; //0.5
		
		Playground* playground = new Playground(odeHandle, osgHandle.changeColor(Color(0.6,0.0,0.6)),
				osg::Vec3(length_pg /*length*/, width_pg /*width*/, height_pg/*height*/), /*factorxy = 1*/1, /*createGround=true*/true /*false*/);

		playground->setPosition(osg::Vec3(0,0,0.0));
		global.obstacles.push_back(playground);

		// DSW: outer_pg = outter platform
  	double length_outer_pg = 22.0; //45, 32, 22
		double width_outer_pg = 5.0;//0.0;  //0.2
		double height_outer_pg = 1.0;//0.0; //0.5

		Playground* outer_playground = new Playground(odeHandle, osgHandle.changeColor(Color(0.6,0.0,0.6)),
					osg::Vec3(length_outer_pg /*length*/, width_outer_pg /*width*/, height_outer_pg/*height*/), /*factorxy = 1*/1, /*createGround=true*/true /*false*/);
		outer_playground->setPosition(osg::Vec3(0,0,0.0));
		global.obstacles.push_back(outer_playground);

  	// DSW: outer_pg2 = walls
  	double length_outer_pg2 = 32.0; //45, 32, 22
		double width_outer_pg2 = 1.0;//0.0;  //0.2
		double height_outer_pg2 = 2.0;//0.0; //0.5

		Playground* outer_playground2 = new Playground(odeHandle, osgHandle.changeColor(Color(0.6,0.0,0.6)),
					osg::Vec3(length_outer_pg2 /*length*/, width_outer_pg2 /*width*/, height_outer_pg2/*height*/), /*factorxy = 1*/1, /*createGround=true*/true /*false*/);
		outer_playground2->setPosition(osg::Vec3(0,0,0.0));
  	global.obstacles.push_back(outer_playground2);

/**************************************************************************************************
***			Set up goal boxes
**************************************************************************************************/

		PassiveBox* b1;
		double length = 0.9;
		double width = 0.9;
		double height = 0.9;
  	b1 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(length, width, height));
  	b1->setColor(Color(1,0,0));
  	b1->setPose(osg::Matrix::rotate(0, 0,0, 1) * osg::Matrix::translate(-5,0,1.5));
		grippables.push_back(b1->getMainPrimitive());
		global.obstacles.push_back(b1);


/**************************************************************************************************
***			Set up robot and controller
**************************************************************************************************/

		//0)

		//qcontroller->setReset(1);

		//1) Activate IR sensors
	  	FourWheeledConfGripper fconf = FourWheeledRPosGripper::getDefaultConf();

		///2) relative sensors
		for (int i=0; i < number_spheres; i++){
			fconf.rpos_sensor_references.push_back(obst.at(i)->getMainPrimitive());
		}
		FourWheeledRPosGripper* vehicle = new FourWheeledRPosGripper(odeHandle, osgHandle, fconf);

		/****Initial position of Nimm4******/
    // DSW
    Pos pos(0.0/*, +x = to left, -x = to right*/,0.0/*y*/,1.0/*z*/);
    //setting position and orientation
    vehicle->place(osg::Matrix::rotate(0, 0, 0, 1) *osg::Matrix::translate(pos));
		
		qcontroller = new ASLController("1","1", vehicle, grippables);
		global.configs.push_back(qcontroller);

		// create pointer to one2onewiring
		AbstractWiring*  wiring = new One2OneWiring(new ColorUniformNoise(0.1));

		// create pointer to agent
		plotoptions.push_back(PlotOption(NoPlot /*select "File" to save signals, "NoPlot" to not save*/));
		OdeAgent* agent = new OdeAgent(plotoptions);

		agent->init(qcontroller, vehicle, wiring);///////////// Initial controller!!!
		global.agents.push_back(agent);

/**************************************************************************************************
***			Testing stuff
**************************************************************************************************/
		

	}



};


int main (int argc, char **argv)
{
	ThisSim sim;

	return sim.run(argc, argv) ? 0 : 1;



}
