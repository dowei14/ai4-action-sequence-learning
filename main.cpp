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

// relative_sensors
std::vector<AbstractObstacle*> relative_sensor_obst;

// DSW
Primitives grippables;
Primitives boxPrimitives;


class ThisSim : public Simulation {
public:

	// generate 4 landmark spheres and 1 goal
	void generate_spheres(GlobalData& global)
	{
		PassiveSphere* s1;
		s1 = new PassiveSphere(odeHandle, osgHandle, 0.5, 0.0);
		s1->setPosition(osg::Vec3(16,16,2));
		s1->setColor(Color(0,0,1));
		relative_sensor_obst.push_back(s1);
		
		PassiveSphere* s2;
		s2 = new PassiveSphere(odeHandle, osgHandle, 0.5, 0.0);
		s2->setPosition(osg::Vec3(-16,-16,2));
		s2->setColor(Color(0,0,1));
		relative_sensor_obst.push_back(s2);
		
		PassiveSphere* s3;
		s3 = new PassiveSphere(odeHandle, osgHandle, 0.5, 0.0);
		s3->setPosition(osg::Vec3(-16,16,2));
		s3->setColor(Color(0,0,1));
		relative_sensor_obst.push_back(s3);
		
		PassiveSphere* s4;
		s4 = new PassiveSphere(odeHandle, osgHandle, 0.5, 0.0);
		s4->setPosition(osg::Vec3(16,-16,2));
		s4->setColor(Color(0,0,1));
		relative_sensor_obst.push_back(s4);
		
		PassiveSphere* s5;
		s5 = new PassiveSphere(odeHandle, osgHandle, 0.5, 0.0);
		s5->setPosition(osg::Vec3(-13,0,2));
		s5->setColor(Color(0,1,0));
		relative_sensor_obst.push_back(s5);
	}
	
	// generate 3 goal boxes return Primitives(vector of Primitive*) of boxes to add to grippables
	void generate_boxes(GlobalData& global)
	{
		double length = 0.9;
		double width = 0.9;
		double height = 0.9;
		PassiveBox* b1;
  	b1 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(length, width, height));
  	b1->setColor(Color(0,1,0));
  	b1->setPose(osg::Matrix::rotate(0, 0,0, 1) * osg::Matrix::translate(-5,0,1.5));
		global.obstacles.push_back(b1);
		
		PassiveBox* b2;
  	b2 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(length, width, height));
  	b2->setColor(Color(1,0,0));
  	b2->setPose(osg::Matrix::rotate(0, 0,0, 1) * osg::Matrix::translate(5,0,1.5));
		global.obstacles.push_back(b2);

		PassiveBox* b3;
  	b3 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(length, width, height));
  	b3->setColor(Color(1,0,0));
  	b3->setPose(osg::Matrix::rotate(0, 0,0, 1) * osg::Matrix::translate(0,-5,1.5));
		global.obstacles.push_back(b3);	
		
		boxPrimitives.push_back(b1->getMainPrimitive());
		boxPrimitives.push_back(b2->getMainPrimitive());
		boxPrimitives.push_back(b3->getMainPrimitive());
		
		// adding boxes to obstacle vector so it can be connected to sensors
		relative_sensor_obst.push_back(b1);
		relative_sensor_obst.push_back(b2);
		relative_sensor_obst.push_back(b3);
		
	}	
	
	// set up Playground
	void setup_Playground(GlobalData& global)
	{
		// inner Platform
		double length_pg = 0.0;
		double width_pg = 10.0;
		double height_pg = 1.0;
		
		Playground* playground = new Playground(odeHandle, osgHandle.changeColor(Color(0.6,0.0,0.6)),
				osg::Vec3(length_pg /*length*/, width_pg /*width*/, height_pg/*height*/), /*factorxy = 1*/1, /*createGround=true*/true /*false*/);

		playground->setPosition(osg::Vec3(0,0,0.0));
		global.obstacles.push_back(playground);

		// DSW: outer_pg = outter platform
  	double length_outer_pg = 22.0;
		double width_outer_pg = 5.0;
		double height_outer_pg = 1.0;

		Playground* outer_playground = new Playground(odeHandle, osgHandle.changeColor(Color(0.6,0.0,0.6)),
					osg::Vec3(length_outer_pg /*length*/, width_outer_pg /*width*/, height_outer_pg/*height*/), /*factorxy = 1*/1, /*createGround=true*/true /*false*/);
		outer_playground->setPosition(osg::Vec3(0,0,0.0));
		global.obstacles.push_back(outer_playground);

  	// DSW: outer_pg2 = walls
  	double length_outer_pg2 = 32.0;
		double width_outer_pg2 = 1.0;
		double height_outer_pg2 = 2.0;

		Playground* outer_playground2 = new Playground(odeHandle, osgHandle.changeColor(Color(0.6,0.0,0.6)),
					osg::Vec3(length_outer_pg2 /*length*/, width_outer_pg2 /*width*/, height_outer_pg2/*height*/), /*factorxy = 1*/1, /*createGround=true*/true /*false*/);
		outer_playground2->setPosition(osg::Vec3(0,0,0.0));
  	global.obstacles.push_back(outer_playground2);
	}

	// starting function (executed once at the beginning of the simulation loop)
	void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
	{

		// set initial camera position
//		setCameraHomePos(Pos(0, 40, 10),  Pos(0, 0, 0)); // viewing full screne from side
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
  
		setup_Playground(global);
	
/**************************************************************************************************
***			Set up 4 landmark and 1 goal spheres
**************************************************************************************************/		

		generate_spheres(global);

/**************************************************************************************************
***			Set up 3 pushable boxes and add the first one as graspable
************************************************************************************************/

		generate_boxes(global);
		grippables.push_back(boxPrimitives[0]);

/**************************************************************************************************
***			Set up robot and controller
**************************************************************************************************/

		//0)

		//1) Activate IR sensors
  	FourWheeledConfGripper fconf = FourWheeledRPosGripper::getDefaultConf();

		///2) relative sensors
		for (unsigned int i=0; i < relative_sensor_obst.size(); i++){
			fconf.rpos_sensor_references.push_back(relative_sensor_obst.at(i)->getMainPrimitive());
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
