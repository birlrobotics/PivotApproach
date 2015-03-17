// FOR THE PIVOT APPROACH TO WORK::init YOU MUST INCLUDE A -DPIVOTAPPROACH FLAG IN THE MAKEFILE AT ~/forceSensorPlugin_Pivot/server/Makefile.
/***************************************************************************************************************************************************/
// This code is called when the HIRO Simulation is called.
//  Three main functions are called:
//  1) Init
//  	- Initialize translation and rotation parameters for both arms. Allocates Right and Left arm clases and force sensor classes.
//  2) Setup
//  	- Called when the Simulator Start Control Button is pressed
// 		- Sets up initial force values, joint angles, and cartesian positions.
// 		- Calls Initialization routine on hiroArm clases.
//  	- Reads calibration and trajectory files.
//  3) Control
//		- Executes selected control methods through control mode parameters.
// 		- Input and Output pass the RobotState.
//				- struct RobotState:
//					{
//						DblSequence				angle;			// As input it is the current position. When outputed it's the desired position.
//						DblSequence				velocity;
//						DblSequence				torque;
//						sequence<DblSequence6>	force;
//						sequence<DblSequence3>	rateGyro;
//						sequence<DblSequence3>	accel;
//						sequence<DblSequence9>	attitude;
//						DblSequence3			zmp;
//						DblSequence3			basePos;
//						DblSequence9			baseAtt;
//						sequence<ImageData> 	image;
//						LongSequence			dio;
//					};
//		- Output:
//			- By writing reference angle updates, the HIRO simulation will move to the requested angles.
//
// - Modifications based on "serverv0.1"   (09.05.12)
// 		- Deleted "setDirectTeachingAxis(::CORBA::ULong axisflag)". (09.05.13)
// 		- Deleted "setInertiaDamping(::CORBA::ULong mx,::CORBA::ULong dx)", (09.05.13)
// 		- Added    "setRPWbySensor(::CORBA::ULong r,::CORBA::ULong p, ::CORBA::ULong w)", (09.05.13)
// 		- Added    Control starts by DTES signal( not via corba)
/***************************************************************************************************************************************************/

#include "Body.h"			// In DynamicsSimulator/server
#include <sstream>
#include <cstdlib>
#include <ctime>				// compute cycle time
#include <sys/time.h>
#include "iob.h"				// Controller/IOServer/include
#include "forceSensorPlugin_impl.h"
//----------------------------------------------------------------------------------------------------------------------------------------------------
extern "C" {
#include <unistd.h>			// Provides access to the POSIX operating system API
#include <stdio.h>
}

//-----------------------------------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------------------------------------
// HARDWARE BIT-WISE SWITCH DEFINITIONS
// The HIRO robot is connected to a hardware box that can be modified in four ways to achieve different behvaiors.
#define MSSL  0x80000000    // Motion Select Switch-> left
#define MSSR  0x40000000    // Motion Select Switch-> right
#define ASSL  0x20000000    // Axis Select Switch-> left
#define ASSR  0x10000000    // Axis Select Switch-> right
#define DTES  0x08000000    // DT enable switch (for data acquisition)
//----------------------------------------------------------------------------------------------------------------------------------------------------
// Debugging
#define DB_TIME 			0			// Used to print timing duration of functions
#define FORCE_TEST 			0 			// Used to test if force drivers are working in Old HIRO
#define SIMULATION_TEST 	1	 		// Used to test certain pieces of code within the control method for testing purposes
#define DEBUG 				1 			// Used to test temporary cerr statements
#define LOOPBACK            1
#define DEVIATION           1
//----------------------------------------------------------------------------------------------------------------------------------------------------
// GLOBALS
unsigned long long distate; // Digital state parameter. Holds bit values.
double size;
//----------------------------------------------------------------------------------------------------------------------------------------------------

/***************************** Allocate Plugin*********************************/

//class forceSensorPlugin_impl: public plugin, 
////Virtual public POA_SamplePlugin, virtual public PortableServer::RefCountServantBase
//{};

plugin *create_plugin(istringstream &strm)
{
	forceSensorPlugin_impl *cpImpl = new forceSensorPlugin_impl(strm);
	return cpImpl;
}

void forceSensorPlugin_impl::test()
{
	cout << "test" << endl;
}
/***************************** Create Plugin*********************************/
// Constructor
//forceSensorPlugin_impl::forceSensorPlugin_impl(istringstream &strm)
forceSensorPlugin_impl::forceSensorPlugin_impl(istringstream &strm)
{
#ifdef DEBUG_PLUGIN
	std::cerr << "forceSensorPlugin_impl entered constructor" << std::endl;
#endif

	DT 				= 0.005;   	// [sec]
	assigned_time 	= 0.002; 	// assigned_time = 2[ms]

	// Test Flags used within the control method
	initControl = 0;
	testFlag	= 0;
	num_test 	= 0;

#ifdef DEBUG_PLUGIN
	std::cerr << "forceSensorPlugin_impl(): reading files" << std::endl;
#endif

	// Read Arm and Gain parameters
	readInitialFile("ArmParam");			// Velocity and Accelretion Gains and Trajectories
	readGainFile("GainParam");				// GainP and GainR for HNL modes
#ifdef DEBUG_PLUGIN
	std::cerr << "forceSensorPlugin_impl(): finished reading files" << std::endl;
#endif

#ifdef DEBUG_PLUGIN
	std::cerr << "forceSensorPlugin_impl(): initializing variables" << std::endl;
#endif

	// Pivot Approach Variables Initialization
	for(int i=0;i<3;i++)
	{
		CurXYZ(i)	=0.0;
	}
	for(int i=0;i<6;i++)
	{
		CurrentForces(i)	=0.0;
		CurrentAngles(i)	=0.0;
		JointAngleUpdate(i)	=0.0;
	}
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			CurRot(i,j) = 0.0;
#ifdef DEBUG_PLUGIN
	std::cerr << "forceSensorPlugin_impl(): finished initializing variables" << std::endl;
#endif

	// Flags
	initFlag = false;

#ifdef DEBUG_PLUGIN
	std::cerr << "forceSensorPlugin_impl: leaving constructor" << std::endl;
#endif
}

forceSensorPlugin_impl::~forceSensorPlugin_impl()
{
	close_ifs();

#ifdef WRITELOG
	if(ostr_astate.is_open())
		ostr_astate.close();
	ostr_astate.clear();
	if(ostr_rstate.is_open())
		ostr_rstate.close();
	ostr_rstate.clear();
	if(ostr_force.is_open())
		ostr_force.close();
	ostr_force.clear();
	if(ostr_worldforce.is_open())
		ostr_worldforce.close();
	ostr_force.clear();
#endif
}

/***************************** Initialize Plugin*********************************/
// Runs when similuator script is activated.
// Initializes Left and Right Arm Position/Rotation Variables
// Calls force sensor Contstructor
// Calls Left and Right arm Constructors
// TODO: Add POA single_thread or main_thread policy to this server.(needed?)
/********************************************************************************/
void forceSensorPlugin_impl::init(void)
{
	// Local Variables
	// e = wrist
	// h = hand
	// fs = force sensor
	// P = position, R = rotation

	vector3  ePh;							// Wrist to EndEffector Translation
	matrix33 eRh;							// Wrist to EndEffector Rotation

	vector3  hPfs;							// EndEffector to Force Sense translation
	matrix33 hRfs;							// EndEffector to Force Sensor Tool Center Point Rotation

	std::cout << "In init()" << std::endl;
#ifdef DEBUG_PLUGIN
	std::cerr << "forceSensorPlugin::init()" << std::endl;
#endif

	/*------------------------------------------------------------------------------- Load model file -------------------------------------------------------------------------------*/
	if (!CORBA::is_nil(manager->orb))
	{
		//  body data
		std::cout << "loading Model: " << manager->url << std::endl;
		body = OpenHRP::loadBodyFromModelLoader((manager->url).c_str(),manager->orb);		// 使用方法は		// http://www.openrtp.jp/openhrp3/jp/calc_model.html
	}
	else
		std::cout << "manager->orb is nil" << std::endl;

#ifdef DEBUG_PLUGIN
	std::cerr << "forceSensorPlugin::init() - exiting" << std::endl;
#endif

	/*------------------------------------------------------------------------------- Setup Force Sensor -------------------------------------------------------*/

	// ------------------ FORCE SENSOR ------------------------//
	// Open the oldHIro IFS sensor
#ifdef DEBUG_PLUGIN
	std::cerr << "forceSensorPlugin::Opening the ifs force lib" << std::endl;
#endif

#ifndef SIMULATION
	open_ifs();
#endif

#ifdef DEBUG_PLUGIN
	std::cerr << "forceSensorPlugin::init() - Closeing the ifs force lib" << std::endl;
#endif


#ifndef SIMULATION
	//	if (!fs->init())
	//		std::cerr << "error: fs->init()" << std::endl;
#endif

	/*------------------------------------------------------------------------------- Setup LEFT and RIGHT arms angle limits -------------------------------------*/
	float ang_limit_sub[ARM_DOF][5];
#if 0
	//---------------------- LEFT arm ----------------------------//

#ifdef DEBUG_PLUGIN
	std::cout << "ang_limit_sub left arm: " << std::endl;
#endif

	for (int i = 0; i < ARM_DOF; i++)
	{
		for (int j = 0; j < 5; j++)
		{
			ang_limit_sub[i][j] = ang_limit[9 + i][j];

#ifdef DEBUG_PLUGIN
			std::cout << ang_limit_sub[i][j] << " ";
#endif
		}
#ifdef DEBUG_PLUGIN
		std::cout << std::endl;
#endif
	}

	/***************************** Initialize Variables *************************************/

	/*********************** LEFT ARM ****************************/

	// Position and Rotation Variables
	//ePh = -0.052, 0.0, 0.0;
	ePh = 0.0715, 0.0, 0.0;										// Wrist to EndEffector translation
	eRh = 1.0, 0.0, 0.0,
			0.0, 1.0, 0.0,
			0.0, 0.0, 1.0;									// Wrist to EndEffector rotation
	hPfs = (0.0085 + (0.0315 / 2)), 0.0, 0.0;							// Wrist to Force Sensor Tool Center Point

	// Allocate the Master Arm for Hiro (Left Arm)
#ifdef DEBUG_PLUGIN
	std::cerr << "forceSensorPlugin::Allocating LEFT hiroArm" << std::endl;
#endif

	lArm = new hiroArmMas("left", body, 9, DT, ang_limit_sub, ePh, eRh, hPfs);


#ifdef DEBUG_PLUGIN
	std::cerr << "forceSensorPlugin::Finished allocating LEFT hiroArm" << std::endl;
#endif

	// Set the force pointer
	// lArm->set_FSptr(fs, 0);										// Connect to Force Sensor
	// arg2: int *full_scale
#endif
	/*********************** RIGHT ARM ****************************/
#ifdef DEBUG_PLUGIN
	std::cout << "Right Arm: ang_limit_sub right arm: " << std::endl;
#endif

#ifdef PIVOTAPPROACH

	// Position and Rotation Variables in local wrist coordinates. rot mat={0 0 -1 0 1 0 1 0 0}
	ePh = -0.051, 0.0020, -0.0225; //0.025, 0.0, -0.063; // -0.063, 0.0, -0.025;	// -0.127, 0.025, 0.0;	//-0.127, -0.03, -0.005;	// Wrist to EndEffector (TCP) translation.
	// This accounts for the end-effector that holds the camera molds in the pivot approach.
	// -0.127=height,0.025=frontal, 0.0=horizontal
	eRh = 1.0, 0.0, 0.0,
			0.0, 1.0, 0.0,
			0.0, 0.0, 1.0;
	// Wrist to EndEffector rotation
	hPfs = 0.063 + (0.0315 / 2), 0.0, 0.0;									// EndEffector to Force Sensor Tool Center Point Translation	//hPfs = 53+(31.5/2), 0.0, 60.0;
	hRfs = get_rot33(Y, M_PI / 2.0) * get_rot33(Z, -(3 * M_PI / 4.0));					// EndEffector to Force Sensor Tool Center Point Rotation

	// Allocate the Master Arm for Hiro (Left Arm)
#ifdef DEBUG_PLUGIN
	std::cerr << "forceSensorPlugin()::Allocating RIGHT hiroArm" << std::endl;
#endif

	// Allocate the slave arm - right arm
	int NUM_q0 = 3; // Joint angle at which the right arm starts
	rArm = new hiroArmSla("right", body, NUM_q0, DT, ang_limit_sub, ePh, eRh, hPfs, hRfs);

#ifdef DEBUG_PLUGIN
	std::cerr << "forceSensorPlugin()::Finished allocating RIGHT hiroArm" << std::endl;
#endif

	//rArm->set_FSptr(fs, 1);
	//initialize_ifs(0);

#ifdef DEBUG_PLUGIN
	std::cerr << "forceSensorPlugin::Finished allocating LEFT hiroArm" << std::endl;
#endif

	// Assign Joint Angle Limits
	for(int i = 0; i < ARM_DOF; i++){									// 6 DOF per arm
		for(int j = 0; j < 5; j++){
			ang_limit_sub[i][j] = ang_limit[3 + i][j];
#ifdef DEBUG_PLUGIN
			std::cout << ang_limit_sub[i][j] << " ";
#endif
		}
#ifdef DEBUG_PLUGIN
		std::cout << std::endl;
#endif
	}

	// If not PivotApproach
#else
	// Position and Rotation Variables
	ePh = -0.12, 0.0, 0.0;											// Wrist to EndEffector translation								//ePh = -(53.0+31.5+16.0), 0.0, -60.0;
	eRh = 1.0, 0.0, 0.0,
			0.0, 1.0, 0.0,
			0.0, 0.0, 1.0;										// Wrist to EndEffector rotation
	hPfs = 0.074 + (0.0315 / 2), 0.0, 0.0;												// EndEffector to Force Sensor Tool Center Point Translation	//hPfs = 53+(31.5/2), 0.0, 60.0;
	hRfs = get_rot33(Y, M_PI / 2.0) * get_rot33(Z, -(3 * M_PI / 4.0));					// EndEffector to Force Sensor Tool Center Point Rotation

	// Allocate the slave arm - right arm
	int NUM_q0 = 3; // Joint angle at which the right arm starts
	rArm = new hiroArmSla("right", body, NUM_q0, DT, ang_limit_sub, ePh, eRh, hPfs, hRfs);

	//To save the parameters of rArm
	// Assign Joint Angle Limits
	for(int i = 0; i < ARM_DOF; i++){									// 6 DOF per arm
		for(int j = 0; j < 5; j++){
			ang_limit_sub[i][j] = ang_limit[3 + i][j];
#ifdef DEBUG_PLUGIN
			std::cout << ang_limit_sub[i][j] << " ";
#endif
		}
#ifdef DEBUG_PLUGIN
		std::cout << std::endl;
#endif
	}
#endif // If pivot approach

	/*************************************** Initialize Control Flags ***************************************/
	f_gravity_comp[0] = f_gravity_comp[1] = false;
	controlmode_nr    = NotControlled;
	controlmode_r     = NotControlled;

#ifdef PIVOTAPPROACH		// set the controlmode_nr and controlmode_r parameters to PivotApproach to implement this control mode in ::control
	controlmode_nr = PivotApproach; // Set the enumeration to both controlmode_nr and r
	controlmode_r = PivotApproach;
#elif IMPEDANCE			// set the controlmode_nr and controlmode_r parameters to ImpedanceControl to implement this control mode
	//	controlmode_nr = ImpedanceControl;
	//      controlmode_r = ImpedanceControl;
#endif

#ifdef WRITELOG
	ostr_astate.open("./astate.dat");
	if(!ostr_astate.is_open())
		std::cerr << "astate.dat was not opened." << std::endl;
	ostr_rstate.open("./rstate.dat");
	if(!ostr_rstate.is_open())
		std::cerr << "rstate.dat was not opened." << std::endl;
	ostr_force.open("./localforce.dat");
	if(!ostr_force.is_open())
		std::cerr << "localforce.dat was not opened." << std::endl;
	ostr_worldforce.open("./worldforce.dat");
	if(!ostr_worldforce.is_open())
		std::cerr << "worldforce.dat was not opened." << std::endl;
#endif

#ifdef PLUGIN_DEBUG2
	std::cout << "out init()" << std::endl;
#endif
}

/******************************************************************************************************************************************/
// Setup()
// Sets up initial force values, joint angles, and cartesian positions.
// Calls Initialization routine on hiroArm clases.
// Reads calibration and trajectory files.
// This routine is called when "Start Control" is clicked on the HIRO Simulation dialogue that pops out when the script is started.
// Followed by ::control
/******************************************************************************************************************************************/
bool forceSensorPlugin_impl::setup(RobotState *rs, RobotState *mc)
{
	// 1) Local variable initialization
	int ret = 0;
	double CurAngles[15] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
#ifdef DEBUG_PLUGIN
	std::cerr << "forceSensorPlugin::setup() - entered" << std::endl;
#endif

	/*------------------------------------------------------------- Simulation ---------------------------------------------------*/
#ifdef SIMULATION
	for(int i=0; i<6; i++)
		rArm->raw_forces[i] = rs->force[0][i];
#endif

#ifdef DEBUG_PLUGIN
	cerr << "forceSensorPlugin::setup() - finished\n";
#endif

	// ----------------------------------------------------------- FORCE SENSOR -----------------------------------------------------------------/
#ifndef SIMULATION
	initialize_ifs(0);
	initialize_ifs(1);
#endif

#ifndef SIMULATION
	rs->angle[8] -= 0.531117;
#endif

	/*------------------------------------------------------------ Current Joint Angles --------------------------------------------------------*/
	// 2) Set the desired robot state angles equal to the motion command angles, to avoid jump in the data or a discontinuity
	// Set output angles to input angles
	for (unsigned int i=0; i<DOF; ++i)	// 15 DoF
	{
		body->joint(i)->q 	= rs->angle[i];
		mc->angle[i] 		= rs->angle[i];
		CurAngles[i] 		= rs->angle[i]; // Set the local variables to this
	}
	int NUM_q0=3;
	// And also set CurrentAngles -- private member variable -- to rs->angle
	CurrentAngles[0]=rs->angle[NUM_q0];
	CurrentAngles[1]=rs->angle[NUM_q0+1];
	CurrentAngles[2]=rs->angle[NUM_q0+2];
	CurrentAngles[3]=rs->angle[NUM_q0+3];
	CurrentAngles[4]=rs->angle[NUM_q0+4];
	CurrentAngles[5]=rs->angle[NUM_q0+5];

#ifndef SIMULATION
	mc->angle[8] += 0.531117;
#endif

#ifdef DEBUG_PLUGIN
	std::cerr 	<< "\n\nThe 15 body joint angles in radians are: "
			<< CurAngles[0] << "\t" << CurAngles[1] << "\t" << CurAngles[2] << "\t" << CurAngles[3] << "\t" << CurAngles[4] << "\t" << CurAngles[5] << "\t"
			<< CurAngles[6] << "\t" << CurAngles[7] << "\t" << CurAngles[8] << "\t" << CurAngles[9] << "\t" << CurAngles[10] << "\t" << CurAngles[11]
			                                                                                                                                      << CurAngles[12]<< "\t" << CurAngles[13] << "\t"<< CurAngles[14]<< "\t"
			                                                                                                                                      << "\n/--------------------------------------------------------------------------------------------------------------------------------------------------------------------/" << std::endl;
#endif

	// 3) Calculate the Fwd Kinematics=>Cartesian Positions
	body->calcForwardKinematics();

	/****************************************** PIVOT APPROACH SETUP ************************************************************/
#ifdef PIVOTAPPROACH

	// 4) Initialization routine:
	// Compute the position vector and rotation matrix from base to the wrist the robot has moved to HOMING POSITION
	// This routine behave's different according to user. Assign Desired Behavior.
	// Kensuke: read position and force data for both arms.
#ifdef DEBUG_PLUGIN
	std::cerr << "forceSensorPlugin::setup() - Arm Initialization" << std::endl;
#endif

	ret = rArm->init(body->link(RARM_JOINT5)->p, body->link(RARM_JOINT5)->attitude(),CurAngles); // Body object 15 DOF. Need link8 or RARM_JOINT5.

	/***************************************** LOOP BACK SETUP *****************************************************************/
#ifdef LOOPBACK
	// Initial steps For setting up loop
	initRs = *rs;
	resetTime = 0;
	proState = 0;
	// End of setting up loop
#endif

#ifdef DEVIATION
	//Initial stepsetup deviation
	if (deviation_file.is_open())
		deviation_file.close();
	deviation_file.open("./data/PivotApproach/deviation.dat");
	for (int i=0; i < 6; i ++)
		deviation_file>>deviation[i];

	char mkdirCommand[256] = "";
	sprintf(mkdirCommand, "%s %s %s", mkdir, dataCollectHomePath, "-p");
	system(mkdirCommand);
	for (int i=0; i < 6; i ++)
		rArm->PA->deviation[i] = this->deviation[i];
#endif

	// Update the latest data angles and position
	if(DEBUG)
	{
		// Number of joints
		int n=0;
		n = rArm->m_path->numJoints();
		cerr << "forceSensorPlugin::setup(). Num Joints: " << n << std::endl;

		// Position
		vector3 rpy(0);
		rArm->set_OrgPosRot(CurXYZ,rpy);
		cerr << "forceSensorPlugin::setup(). Current Position: " << CurXYZ << std::endl;
		cerr << "forceSensorPlugin::setup(). Current Pose: " << rpy << std::endl;

		// Angles
		cerr << "forceSensorPlugin::setup(). Current Angles (radians):\t" << CurrentAngles << std::endl;
	}

	// Check result
	if(ret==-1)
	{
#ifdef DEBUG_PLUGIN
		std::cerr << "\nProgram Failed!!";
#endif
	}
	else
	{
#ifdef DEBUGC_PLUGIN2
		std::cerr <<"forceSensorPlugin::setup() - Initialization was successful!" << std::endl;
#endif
	}

#endif

	/****************************************** PIVOT APPROACH ************************************************************/
#ifdef PIVOTAPPROACH
	controlmode_nr = PivotApproach;
	controlmode_r = PivotApproach;
#endif

#ifdef DEBUG_PLUGIN
	std::cerr << "forceSensorPlugin::setup() - Exiting" << std::endl;
#endif

	return true;

}
 
/******************************************************************************************************************************************/
// Control()
// The control function is called by the GrxUI simulation every 2-5ms. This method implements the control routine that will control HIRO.
// This function is passed the RobotState rs which i a structure containing all the data of the robot:
//
// struct RobotState:
//	{
//		DblSequence				angle;
//		DblSequence				velocity;
//		DblSequence				torque;
//		sequence<DblSequence6>	force;
//		sequence<DblSequence3>	rateGyro;
//		sequence<DblSequence3>	accel;
//		sequence<DblSequence9>	attitude;
//		DblSequence3			zmp;
//		DblSequence3			basePos;
//		DblSequence9			baseAtt;
//		sequence<ImageData> 	image;
//		LongSequence			dio;
//	};
//
// Function performs the following tasks:
//		- Read Joint Angles from Simulation
// 		- Compute corresponding Cartesian Coordinates
//
// The second part uses a case-switch statement to determine the control command.
// New control modes can be inserted here
/******************************************************************************************************************************************/
void forceSensorPlugin_impl::control(RobotState *rs, RobotState *mc)
{
	// Local variable
	int ret = 0;

#ifndef SIMULATION
	rs->angle[8] -= 0.531117;
#endif
	//---------------------------------------- FORCE TEST ------------------------------------------/
	if(!FORCE_TEST)
	{
#ifdef DEBUG_PLUGIN
		std::cerr << "/----------------------------------\nforceSensorPlugin::control() - entered\n-------------------------------------/\n\n" << std::endl;
#endif	

		/********************************************************* Resettting Code *****************************************************************/

#ifdef LOOPBACK
		//The loopback_condition is used for determine whether it needs to restart or not
		if (this->loopback_condition())  {
			// set up the whole thing ... what ... same function as setup()
			// state control. should enter this function once to setup mc before initControl
			proState = 0;
			initControl = 0;
			//Record the loop time.
			resetTime += 1;

#ifdef DEVIATION
			// Collect the data of the previous loop.
			this->loopback_collectData();

			// After collecting previous data, read new deviations from file.
			if (!deviation_file.eof())
				for (int i=0;i < 6; i++)
					deviation_file>>deviation[i];

#endif

			//Build a folder for the data last time.

			RobotState* firstRs;
			firstRs = &initRs;
			rs = &initRs;
			// 1) Local variable initialization
				int ret = 0;
				double CurAngles[15] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


				/*------------------------------------------------------------- Simulation ---------------------------------------------------*/
			#ifdef SIMULATION
				for(int i=0; i<6; i++)
					rArm->raw_forces[i] = firstRs->force[0][i];
			#endif
				// ----------------------------------------------------------- FORCE SENSOR -----------------------------------------------------------------/

			#ifndef SIMULATION
				firstRs->angle[8] -= 0.531117;
			#endif

				/*------------------------------------------------------------ Current Joint Angles --------------------------------------------------------*/
				// 2) Set the desired robot state angles equal to the motion command angles, to avoid jump in the data or a discontinuity
				// Set output angles to input angles
				for (unsigned int i=0; i<DOF; ++i)	// 15 DoF
				{
					body->joint(i)->q 	= firstRs->angle[i];
					mc->angle[i] 		= firstRs->angle[i];
					CurAngles[i] 		= firstRs->angle[i]; // Set the local variables to this
				}
				int NUM_q0=3;
				// And also set CurrentAngles -- private member variable -- to rs->angle
				for (unsigned int i=0; i<6; ++i)
					CurrentAngles[i]=firstRs->angle[NUM_q0 + i];
//
			#ifndef SIMULATION
				mc->angle[8] += 0.531117;
			#endif

				// 3) Calculate the Fwd Kinematics=>Cartesian Positions
				body->calcForwardKinematics();

				/****************************************** PIVOT APPROACH SETUP ************************************************************/
#ifdef PIVOTAPPROACH

				// 4) Initialization routine:
				// Compute the position vector and rotation matrix from base to the wrist the robot has moved to HOMING POSITION
				// This routine behave's different according to user. Assign Desired Behavior.
				// Kensuke: read position and force data for both arms.


				//ret = rArm->init(body->link(RARM_JOINT5)->p, body->link(RARM_JOINT5)->attitude(),CurAngles); // Body object 15 DOF. Need link8 or RARM_JOINT5.

				for(int i=0; i<6; i++)
					rArm->raw_forces[i] = firstRs->force[0][i];
				// Update the latest data angles and position

				ret = rArm->init(body->link(RARM_JOINT5)->p, body->link(RARM_JOINT5)->attitude(),CurAngles, deviation); // Body object 15 DOF. Need link8 or RARM_JOINT5.

#endif


				/****************************************** PIVOT APPROACH ************************************************************/
#ifdef PIVOTAPPROACH
				controlmode_nr = PivotApproach;
				controlmode_r = PivotApproach;
#endif

#ifndef SIMULATION
				firstRs->angle[8] += 0.531117;
#endif

				return;
		}
#endif // end of loopback

		if(initControl==0)
				{
					// Initialize the iteration
					rArm->m_time=0;

					//loop init
					proState = 1;

					// Change the flag
					initControl=1;
				}
		// Simulation test is used to skip code that is enclosed the if function.
		if(SIMULATION_TEST)
		{
			// Timing
			timeval startTime, endTime;			// create variables
			double duration = 0;
			if(DB_TIME)	{
				gettimeofday(&startTime,NULL); 						// Initialize startTime
			}

			/*-------------------------------------------------------------- Arm Selection ---------------------------------------------------*/
			// What arm will you use. Left = 0 index. Right = 1 index.
			bool f_control[2];
			f_control[0] = false;
			f_control[1] = true;

			// Set controlmode _r equal to _nr
#if 0	
			if (controlmode_r != controlmode_nr)
				controlmode_r = controlmode_nr;
#endif
			/*----------------------------------------------------------- Simulation Force Values ----------------------------------------*/
#ifdef SIMULATION
			for(int i=0; i<6; i++)
				rArm->raw_forces[i] = rs->force[0][i];
#else

#if 0
			// Read DIN state
			read_di(distate);
			DioState = distate >> 32;
#endif
#ifdef DEBUG_PLUGIN
			std::cerr << "Getting Force Values" << std::endl;
#endif			
#endif

#ifdef DEBUG_PLUGIN
			std::cerr << "The control mode for the PivotApproach should say 6. Currently it is: " << controlmode_r << std::endl;
#endif


			/*------------------------------------------------------------ INITIALIZATION STAGE ----------------------------------------------*/
			if(initFlag==false)
			{
				/*---------------------- Angle Update --------------------------*/
				// For the first iteration make sure that output angles are the same as input angles for all 15 DOF.
				for (unsigned int i = 0; i < DOF; ++i)
				{
					body->joint(i)->q 	= rs->angle[i];	// the body object is for the entire body 15 DoF
					mc->angle[i] 		= rs->angle[i];
				}
				//for(int i=0; i<ARM_DOF; i++)			//body->joint(i)->q = rs->angle[i+3];			// Copy the latest actual angles into our private member variable.

				// change flag
				initFlag = true;
				//controlmode_pre = controlmode_r;

#ifdef DEBUG_PLUGIN
				std::cerr << "\n/---------------------------------------------------------------------------------------------------------------------------\n"
						"forceSensorPlugin - The 15 current angles in radians are:\t"   << rs->angle[0]/**rad2degC*/ << "\t" << rs->angle[1]/**rad2degC*/ << "\t" << rs->angle[2]/**rad2degC*/ << "\t"
						<< rs->angle[3]/**rad2degC*/ << "\t" << rs->angle[4]/**rad2degC*/ << "\t" << rs->angle[5]/**rad2degC*/ << "\t"
						<< rs->angle[6]/**rad2degC*/ << "\t" << rs->angle[7]/**rad2degC*/ << "\t" << rs->angle[8]/**rad2degC*/ << "\t"
						<< rs->angle[9]/**rad2degC*/ << "\t" << rs->angle[10]/**rad2degC*/<< "\t" << rs->angle[11]/**rad2degC*/<< "\t"
						<< rs->angle[12]/**rad2degC*/<< "\t" << rs->angle[13]/**rad2degC*/<< "\t" << rs->angle[14]/**rad2degC*/<<
						"\n/------------------------------------------------------------------------------------------------------------\n" << std::endl;
#endif

				// Timing
				if(DB_TIME)
				{
					// Get end time
					gettimeofday(&endTime,NULL);

					// Compute duration
					duration = (double)(endTime.tv_sec - startTime.tv_sec) * 1000.0; 		// Get from sec to msec
					duration += (double)(endTime.tv_usec - startTime.tv_usec) / 1000.0; 	// From usec to msec

					// Print out the duration of the function
					std::cerr << "Duration of forceSensorPlugin::control() is: " << duration << "ms." << std::endl;
				}

			} // End initFlag
			/*----------------------------------------------------------------------------------------------------------------------------------------------------------*/

#ifdef DEBUG_PLUGIN
			std::cerr << "Calculate Fwd Kins" << std::endl;
#endif

			// Calculate cartesian positions
			body->calcForwardKinematics();


#ifdef DEBUG_PLUGIN
			std::cerr << "Update the current position data." << std::endl;
#endif	


			// Update cartesian position of robot arms
			//lArm->update_currposdata();	// Gets base2wrist translation/rotation, and base2endeffector translation/rotation
			rArm->update_currposdata(); // And, current joint angles for the respective arm

			/*------------------------------------------------------------- Select Control Mode-----------------------------------------------------------------------*/

			// There are eight control modes: NotControlled, GravityCompensation, ResetGravityCompensation, BirateralControl, DirectTeaching, Impedance Control, and Pivot Approach
#ifdef DEBUG_PLUGIN
			std::cerr << "Current  mode is: " << controlmode_r << std::endl;
#endif	

			switch (controlmode_r)
			{

			/*--------------------------------------------------------------------------- Not Controlled ----------------------------------------------------*/

			case NotControlled:
#ifdef DEBUG_PLUGIN
				std::cout << "NotControlled" << std::endl;
#endif

				f_control[0] = f_control[1] = false;
				break;

				/*-------------------------------------------------------------------------------------------- Gravity Compensaation ---------------------------------------------------------------------------------------*/

			case GravityCompensation:
#ifdef DEBUG_PLUGIN
				std::cout << "GravityCompensation" << std::endl;
#endif

				// Go to the Initial Position
				// (A) LEFT ARM
				if (!f_gravity_comp[0])
				{
					// Call gravity compsensation
					int res_gc;
					res_gc = lArm->gravity_comp();

#ifdef DEBUG_PLUGIN
					std::cout << "Gravity Compensation: res_gc = " << res_gc << std::endl;
#endif

					// Success
					if (res_gc == 1)
					{
						f_gravity_comp[0] = true;
						num_test = 0;
					}
					else if (res_gc == 0)
						f_control[0] = true;
				}

				// (B) RIGHT ARM
				else if (!f_gravity_comp[1])
				{

					// Call gravity compensation
					int res_gc;
					res_gc = rArm->gravity_comp();

			// There are eight control modes: NotControlled, GravityCompensation, ResetGravityCompensation, BirateralControl, DirectTeaching, Impedance Control, and Pivot Approach
#ifdef DEBUG_PLUGIN
					std::cout << "Gravity Compensation: res_gc = " << res_gc << std::endl;
#endif

					// Success
					if (res_gc == 1)
					{
						f_gravity_comp[1] = true;
						num_test = 0;
					}
					else if (res_gc == 0)
						f_control[1] = true;
				}

				// Manual Algorithm
				else
				{
					// force sensor data　の　チェック
					lArm->update_currforcedata();	//~ lArm->savedata();
					rArm->update_currforcedata();	//~ rArm->savedata();

					vector3 rF_gc[2], rM_gc[2];

					// Get forces and moments
					lArm->get_forces(rF_gc[0], rM_gc[0]);
					rArm->get_forces(rF_gc[1], rM_gc[1]);

					if (num_test < 2000)
					{
						if (num_test == 0)
						{
							// Initialize
							for (int i = 0; i < 2; i++)
							{
								F_ave[i] 	 =    0.0,    0.0,    0.0;
								M_ave[i] 	 =    0.0,    0.0,    0.0;
								F_sd[i]  	 =    0.0,    0.0,    0.0;
								M_sd[i]  	 =    0.0,    0.0,    0.0;
								F_err_max[i] = -100.0, -100.0, -100.0;
								M_err_max[i] = -100.0, -100.0, -100.0;
							}
						}

						// Compute Averages for Force and Moment
						for (int i = 0; i < 2; i++)
						{
							F_ave[i] += rF_gc[i];
							M_ave[i] += rM_gc[i];

							// Limit Error Computations
							for (int j = 0; j < 3; j++)
							{
								// Square Error
								F_sd[i][j] += rF_gc[i][j] * rF_gc[i][j];
								M_sd[i][j] += rM_gc[i][j] * rM_gc[i][j];

								// Check Limits
								if (F_err_max[i][j] < fabs(rF_gc[i][j]))
									F_err_max[i][j] = fabs(rF_gc[i][j]);
								if (M_err_max[i][j] < fabs(rM_gc[i][j]))
									M_err_max[i][j] = fabs(rM_gc[i][j]);
							}
						}

						// Increase number counter
						num_test++;
						/*lArm->get_raw_forces(fout);
			fprintf(fp_nitta,"%f %f %f %f %f %f\n",fout[0],fout[1],fout[2],fout[0],fout[1],fout[2]);
						 */
					}

					else if (num_test == 2000)
					{
						std::string f_name1 = "/home/hrpuser/yamanobe/data/fs.dat";
						if ((fp = fopen(f_name1.c_str(), "a")) == NULL) std::cerr << "file open error!!" << std::endl;

						for (int i = 0; i < 2; i++)
						{
							// Compute averages by dividing by number of elements
							F_ave[i] = (1.0 / 2000) * F_ave[i];
							M_ave[i] = (1.0 / 2000) * M_ave[i];

							for (int j = 0; j < 3; j++)
							{
								F_sd[i][j] = sqrt(
										((1.0 / 2000) * F_sd[i][j])
										- (F_ave[i][j] * F_ave[i][j]));
								M_sd[i][j] = sqrt(
										((1.0 / 2000) * M_sd[i][j])
										- (M_ave[i][j] * M_ave[i][j]));
							}

							F_zero_lim[i] = -100;
							M_zero_lim[i] = -100;

							double tmp;
							for (int j = 0; j < 3; j++)
							{
								tmp = fabs(F_ave[i][j]) + F_sd[i][j];
								if (tmp > F_zero_lim[i])
									F_zero_lim[i] = tmp;
								tmp = fabs(M_ave[i][j]) + M_sd[i][j];
								if (tmp > M_zero_lim[i])
									M_zero_lim[i] = tmp;
							}
							F_zero_lim[i] = 2.0 * F_zero_lim[i];
							M_zero_lim[i] = 2.0 * M_zero_lim[i];

							fprintf(fp, "sensor %d:\n", i);
							fprintf(fp, "average:     %f %f %f %f %f %f\n", F_ave[i][0],
									F_ave[i][1], F_ave[i][2], M_ave[i][0], M_ave[i][1],
									M_ave[i][2]);
							fprintf(fp, "s_deviation: %f %f %f %f %f %f\n", F_sd[i][0],
									F_sd[i][1], F_sd[i][2], M_sd[i][0], M_sd[i][1],
									M_sd[i][2]);
							fprintf(fp, "max_error:   %f %f %f %f %f %f\n",
									F_err_max[i][0], F_err_max[i][1], F_err_max[i][2],
									M_err_max[i][0], M_err_max[i][1], M_err_max[i][2]);
							fprintf(fp, "zero_limit:   %f %f\n\n", F_zero_lim[i],
									M_zero_lim[i]);

#ifdef DEBUG_PLUGIN
							std::cout << "error of rF_gc = " << F_ave << std::endl;
							std::cout << "error of rM_gc = " << M_ave << std::endl;
							std::cout << "max error F" << F_err_max << std::endl;
							std::cout << "max error M" << M_err_max << std::endl;
#endif
						}
						std::cout << "Gravity Compensation is finished." << std::endl;
						//~ fprintf(fv_L,"Gravity compensation is finished.\n");
						//~ fprintf(fv_R,"Gravity compensation is finished.\n");

						fclose(fp);
						num_test++;
					}
				}
				break;

				/*---------------------------------------------------------------------------------------------- Reset Gravity Compensation -----------------------------------------------------------------------------------------*/

			case ResetGravityCompensation:

#ifdef DEBUG_PLUGIN
				std::cout << "ResetGravityCompensatiPivotApproachon" << std::endl;
#endif

				lArm->reset_gravity_comp();
				rArm->reset_gravity_comp();

				f_gravity_comp[0] = f_gravity_comp[1] = false;
				break;

			case BirateralControl:

#ifdef DEBUG_PLUGIN
				std::cout << "BirateralControl" << std::endl;
#endif
				/*~ if(controlmode_r != controlmode_pre){
	      //~ ofs.open("q_birateral.log");
	      //~ if(!ofs.is_open()) std::cerr << "ERROR: can't open q_birateral.log" << std::endl;
	      ~ }*/

				// Once gravity compensation has taken place
				if (f_gravity_comp[0] && f_gravity_comp[1])
				{
					//~ if(1){

					bool f_new = false;
					if (controlmode_r != controlmode_pre)
						f_new = true;

					// Get force data
					lArm->update_currforcedata();
					rArm->update_currforcedata();

					// Declare force and moment master and slave vectors
					vector3 rF_gc_mas, rM_gc_mas;
					vector3 rF_gc_sla, rM_gc_sla;

					// MAS
					if (!(lArm->get_forces(rF_gc_mas, rM_gc_mas)))
					{
						// if false68
						rF_gc_mas = 0.0, 0.0, 0.0;
						rM_gc_mas = 0.0, 0.0, 0.0;
					}

					// Moment
					else
					{
						// 不感帯の設定
						for (int i = 0; i < 3; i++)
						{
							if (fabs(rF_gc_mas[i]) < F_zero_lim[0])
								rF_gc_mas[i] = 0.0;
							if (fabs(rM_gc_mas[i]) < M_zero_lim[0])
								rM_gc_mas[i] = 0.0;
						}
					}

					// SLA
					if (!(rArm->get_forces(rF_gc_sla, rM_gc_sla)))
					{
						// if false
						rF_gc_sla = 0.0, 0.0, 0.0;
						rM_gc_sla = 0.0, 0.0, 0.0;
					}

					else
					{
						// 不感帯の設定
						for (int i = 0; i < 3; i++)
						{
							if (fabs(rF_gc_sla[i]) < F_zero_lim[1])
								rF_gc_sla[i] = 0.0;
							if (fabs(rM_gc_sla[i]) < M_zero_lim[1])
								rM_gc_sla[i] = 0.0;
						}
					}

					// Calculate reference velocity
					vector3 GainP_tmp, GainR_tmp;

					// GainP
					if (DioState & ASSL)
						GainP_tmp = GainP[0]; // high
					else if (DioState & ASSR)
						GainP_tmp = GainP[2]; // law
					else
						GainP_tmp = GainP[1]; // normal

					// GainR
					if (DioState & MSSL)
						GainR_tmp = GainR[0]; // high
					else if (DioState & MSSR)
						GainR_tmp = GainR[2]; // law
					else
						GainR_tmp = GainR[1]; // normal

#ifdef DEBUG_PLUGIN
					std::cout << "GainP: " << GainP_tmp[0] << " " << GainP_tmp[1] << " "<< GainP_tmp[2] << std::endl;
					std::cout << "GainR: " << GainR_tmp[0] << " " << GainR_tmp[1] << " "<< GainR_tmp[2] << std::endl;
#endif

					// Velocity Control Parameters
					// rate of change in position and orientation?
					vector3 rdP, rW;
					for (int i = 0; i < 3; i++)
					{
						rdP[i] = GainP_tmp[i] * (rF_gc_mas[i] + rF_gc_sla[i]); // Force
						rW[i] = GainR_tmp[i] * (rM_gc_mas[i] + rM_gc_sla[i]);  // Momentum
					}

#ifdef DEBUG_PLUGIN
					std::cout << "DirectTeaching:" << std::endl;
					std::cout << "rdP: " << rdP << std::endl;
					std::cout << "rW: " << rW << std::endl;
#endif

					// for master
					if (lArm->velocity_control(rdP, rW, f_new))
						f_control[0] = true;
					else
						f_control[0] = false;

					// for slave
					if (rArm->velocity_control(rdP, rW, f_new))
						f_control[1] = true;
					else
						f_control[1] = false;

					// save data;
					lArm->savedata();
					rArm->savedata();
				}

				// No gravity control
				else
				{
#ifdef DEBUT_PLUGIN
					std::cerr << "gravity control is not finished." << std::endl;
					f_control[0] = f_control[1] = false;
#endif
				}

				break;

				/*---------------------------------------------------------------------------------------------- Direct Teaching -----------------------------------------------------------------------------------------*/
			case DirectTeaching:

#ifdef DEBUG_PLUGIN
				std::cout << "DirectTeaching" << std::endl;
#endif

				if (f_gravity_comp[0])
				{
					bool f_new = false;
					if (controlmode_r != controlmode_pre)
						f_new = true;

					// Get current Force Data
					lArm->update_currforcedata();

					vector3 rF_gc_mas, rM_gc_mas;
					if (lArm->get_forces(rF_gc_mas, rM_gc_mas))
					{
						// Check minimum limits
						for (int i = 0; i < 3; i++)
						{
							if (fabs(rF_gc_mas[i]) < F_zero_lim[0])
								rF_gc_mas[i] = 0.0;
							if (fabs(rM_gc_mas[i]) < M_zero_lim[0])
								rM_gc_mas[i] = 0.0;
						}

						// Calculate reference velocity
						vector3 GainP_tmp, GainR_tmp;
						// GainP
						if (DioState & ASSL)
							GainP_tmp = GainP[0]; // high
						else if (DioState & ASSR)
							GainP_tmp = GainP[2]; // law
						else
							GainP_tmp = GainP[1]; // normal
						// GainR
						if (DioState & MSSL)
							GainR_tmp = GainR[0]; // high
						else if (DioState & MSSR)
							GainR_tmp = GainR[2]; // law
						else
							GainR_tmp = GainR[1]; // normal

						vector3 rdP, rW;
						for (int i = 0; i < 3; i++)
						{
							rdP[i] = GainP_tmp[i] * rF_gc_mas[i];
							rW[i] = GainR_tmp[i] * rM_gc_mas[i];
						}

						if (lArm->velocity_control(rdP, rW, f_new))
							f_control[0] = true;
						else
							f_control[0] = false;
					}
					else
						f_control[0] = false;
				}

				// No gravity control
				else
				{
#ifdef DEBUG_PLUGIN
					std::cerr << "gravity control is not finished." << std::endl;
					f_control[0] = false;
#endif
				}

				f_control[1] = false;

				break;

				/*---------------------------------------------------------------------------------------------- Impedance Controller -----------------------------------------------------------------------------------------*/
				// By using case statements introduce your own code via  calling an external class.
#ifdef IMPEDANCE
			case ImpedanceControl:
#ifdef DEBUG_PLUGIN
				std::cout << "Impedance Control" << std::endl;
#endif

				// Get latest force readings from simulation
				rArm->update_currforcedata();

				// Call the impedance control algorithms.
				// Generates a desired q_ref output.
				if(rArm->impedance_control2())
					f_control[1] = true;			// Work on the RIGHT ARM.
				else
					f_control[1] = false;
				break;
#endif

				/*---------------------------------------------------------------------------------------------- Pivot Approach -----------------------------------------------------------------------------------------*/
			case PivotApproach:


#ifdef DEBUG_PLUGIN
				std::cerr << "forceSensorPlugin::PivotApproach case - Entering" << std::endl;
#endif	

				/*** Get the latest angles/positions/forces through the Arm class ***/
				//vector3 gc_handForce(0);
				//vector3 gc_handMoment(0);

				// Select to work only on right arm
				f_control[1] = true;

				// Update the forces
#ifdef SIMULATION
				for(int i=0;i<6;i++)
					CurrentForces(i) = rArm->raw_forces[i];
#else
				rArm->update_currforcedata();

				vector3 tmp_force;
				vector3 tmp_moment;
				vector3 rotated_force;
				vector3 rotated_moment;
				matrix33 mod_rot;
				mod_rot = get_rot33(Y,-(M_PI/2.0)) * get_rot33(Z,-(M_PI/4));
				for(int i=0;i<3;i++)
				{
					tmp_force(i)   = rArm->fsF_raw[i];
					tmp_moment(i) = rArm->fsM_raw[i];
				}

				rotated_force = mod_rot * tmp_force;
				rotated_moment = mod_rot * tmp_moment;

				CurrentForces(0) = -rotated_force(0);
				CurrentForces(1) = rotated_force(1);
				CurrentForces(2) = rotated_force(2);
				CurrentForces(3) = -rotated_moment(0);
				CurrentForces(4) = rotated_moment(1);
				CurrentForces(5) = rotated_moment(2);
#endif

#ifdef DEBUG_PLUGIN
				std::cerr << "Current Forces/Moments are:\t" << CurrentForces(0) << "\t" << CurrentForces(1) << "\t" << CurrentForces(2) << "\t" << CurrentForces(3) << "\t" << CurrentForces(4) << "\t" << CurrentForces(5) << std::endl;
#endif				
				//#endif


				// Get current hand (endeffector) position and rotation
				rArm->get_curr_handpos(CurXYZ,CurRot);


#ifdef DEBUG_PLUGIN
				cerr << "\n\n!!!!! forceSensorPlugin::control(). CurXYZ: " << CurXYZ(0) << " " << CurXYZ(1) << " " << CurXYZ(2) << std::endl;
#endif
#ifdef DEBUG_PLUGIN
				std::cerr << "Current Endeffector position:\t" << CurXYZ(0) << " "
						<< CurXYZ(1) << " "
						<< CurXYZ(2) << std::endl;
#endif			

				// Copy current angles
				for(int i=0; i<ARM_DOF; i++)
					CurrentAngles(i) = body->joint(i+3)->q;		// Right arm start at i+3. i.e. chest,pan,tilt,rArm,lArm.

#ifdef DEBUG_PLUGIN
				std::cerr << "Current Joint Angles are:\t" << CurrentAngles(0) << " " << CurrentAngles(1) << " " << CurrentAngles(2) << " "
						<< CurrentAngles(3) << " " << CurrentAngles(4) << " " << CurrentAngles(5) << std::endl;
#endif				

				// Controller time counter
				cur_time=DT*double(rArm->get_Iteration());		// sampling time * iteration
				rArm->set_Iteration();							// Increase iteration

#if 0
				// Perofrm necessary procedures to run the inverse kinematic computations later
				for(int j=0; j<4; j++)
				{
					// rArm->m_path->setIkGain(rArm->ikGains[j]);
					//rArm->m_path->setMaxIKErrorRot(rArm->ikRot[j]);
					//rArm->m_path->setMaxIKErrorTrans(rArm->ikTrans[j]);
				}
#endif

				// Call PivotApproach and retrieve a Joint angle update and CurrentAngles
#ifdef DEBUG_PLUGIN
				std::cerr << "\nforceSensorPlugin::control - Calling hiroArm::PivotApproach()" << std::endl;
#endif			 
				ret=rArm->PivotApproach(cur_time,CurXYZ,CurRot,CurrentForces,JointAngleUpdate,CurrentAngles);
				if(ret==-1)
				{
					cerr << "InverseKinematics failed.\nCurrent time:\t" << cur_time <<
							".\nCurrent iteration:\t" << rArm->get_Iteration() <<
							".\nCurrent Position:\t" << CurXYZ <<
							".\nCurrent Pose:\t" << CurRot <<
							".\nCurrent Angles:\t" << CurrentAngles <<
							".\nCurrent Forces:\t" << CurrentForces << std::endl;
				}


				break;
			}

			/*---------------------------------------------------------------------------------------------- Arm Control -----------------------------------------------------------------------------------------*/
			// Set Desired Joint Angles if PA is NOT defined
#ifndef PIVOTAPPROACH
			for (int i = 0; i < DOF; i++)
				mc->angle[i] = rs->angle[i];
#endif

			// Set the reference angles only.
			// The simulation updates the motions.

			/************************************************************************* Left Arm *******************************************************************************/
			if (f_control[0])
			{

				//~ dvector6 tmp;
				dvector6 qref_l = lArm->get_qref();

				// MC Angles 9-14 belong to the left arm
				for (int i = 0; i < 6; i++)
					mc->angle[9 + i] = qref_l[i];

#ifdef DEBUG_PLUGIN
				std::cout << "q_ref to mc: ";
				for(int i=0; i<6; i++) std::cout << mc->angle[9+i] << ", ";
				std::cout << std::endl;
#endif
			}

			/************************************************************************* Right Arm Arm *******************************************************************************/
			if (f_control[1])
			{
				// Retrive the updated reference joint angles computed by the pivot approach
				dvector6 qref_r = rArm->get_qref();

				// Copy those reference joint angles to the motion command object to move hiro's arm.
				// MC Angles 3-8 belong to the right arm
				for (int i=0; i < ARM_DOF; i++)
					mc->angle[i+3] = qref_r[i];

				// Bug happens here.
				mc->angle[0]=0; mc->angle[1]=0; mc->angle[2]=0;	// Alway sset the chest, pan, tilt angles to zero.
			}

#ifdef DEBUG_PLUGIN
			std::cerr << "\nforceSensorPlugin - The current angles in degrees (including chest and head) are:\t" << mc->angle[0]*rad2degC << "\t" << mc->angle[1]*rad2degC << "\t" << mc->angle[2]*rad2degC << "\t"
					<< mc->angle[3]*rad2degC << "\t" << mc->angle[4]*rad2degC << "\t" << mc->angle[5]*rad2degC << "\t"
					<< mc->angle[6]*rad2degC << "\t" << mc->angle[7]*rad2degC << "\t" << mc->angle[8]*rad2degC << std::endl;
#endif

			/* save data		//~ if(f_gravity_comp[0] && f_gravity_comp[1]){		//~ lArm->savedata();	//~ rArm->savedata();	//~ }*/
			controlmode_pre = controlmode_r;

			// Timing
			if(DB_TIME)
			{
				// Get end time		//gettimeofday(&endTime,NULL);

				// Compute duration
				duration = (double)(endTime.tv_sec - startTime.tv_sec) * 1000.0; 		// Get from sec to msec
				duration += (double)(endTime.tv_usec - startTime.tv_usec) / 1000.0; 	// From usec to msec

				// Print out the duration of the function
				std::cerr << "Duration of forceSensorPlugin::control() is: " << duration << "ms." << std::endl;
			}

		} // END !FORCE_TEST

		else
		{

			rArm->update_currforcedata();

			for(int i=0;i<3;i++)
			{
				CurrentForces(i)   = rArm->fsF_raw[i];
				CurrentForces(i+3) = rArm->fsM_raw[i];
			}

			// Controller time counter
			cur_time=DT*double(rArm->get_Iteration());      // sampling time * iteration
			rArm->set_Iteration();                          // Increase iteration           // Controller time counter
#ifdef DEBUG_PLUGIN		
			// Print value to the error console
			cerr << "\nTime + Forces: " << cur_time << "\t" << CurrentForces[0] << "\t" <<  CurrentForces[1] << "\t" <<  CurrentForces[2] << "\t"
					<< CurrentForces[3] << "\t" <<  CurrentForces[4] << "\t" <<  CurrentForces[5] << "\t"
					<< std::endl;// Print value to the error console

			cout << "\nTime + Forces: " << cur_time << "\t" << CurrentForces[0] << "\t" <<  CurrentForces[1] << "\t" <<  CurrentForces[2] << "\t"
					<< CurrentForces[3] << "\t" <<  CurrentForces[4] << "\t" <<  CurrentForces[5] << "\t"
					<< std::endl;// Print value to the error console
#endif
		}

	}//simulation test

#ifndef SIMULATION
	mc->angle[8] += 0.531117;
#endif

#ifdef WRITELOG
	// Desired Robot Angles for the Right Arm
	ostr_astate << cur_time;
	for(int i=3;i<9;i++)
		ostr_astate << " " << rs->angle[i];
	ostr_astate << std::endl;
	ostr_rstate << cur_time;

	// Actual Robot Angles for the Right Arm
	for(int i=3;i<9;i++)
		ostr_rstate << " " << mc->angle[i];
	ostr_rstate << std::endl;
	ostr_force << cur_time;

	// Current Forces in Local Coordinates
	for(int i=0;i<6;i++)
		ostr_force << " " << CurrentForces(i);
	ostr_force << std::endl;

	// Current Forces in World Coordinates
	vector3 world_force;
	vector3 world_moment;
	vector3 tmp_force;
	vector3 tmp_moment;

	for(int i=0;i<3;i++)
	{
		tmp_force(i) = CurrentForces(i);
		tmp_moment(i) = CurrentForces(i+3);
	}

	world_force = CurRot * tmp_force;
	world_moment = CurRot * tmp_moment;
	ostr_worldforce << cur_time;
	for(int i=0;i<3;i++)
		ostr_worldforce << " " << world_force(i);
	for(int i=0;i<3;i++)
		ostr_worldforce << " " << world_moment(i);
	ostr_worldforce << std::endl;
#endif
}
//`````````````````````````````````````````````````````````````````````````````````````````````````````````````
/************************* CleanUp *********************************/
bool forceSensorPlugin_impl::cleanup(RobotState *rs, RobotState *mc)
{
#if 0
	// Close Files.
	fclose(fv);
	fclose(fv_R);
	fclose(fv_L);

	// Delete Arm and Force Sensor allocated objects
	delete lArm;
	delete rArm;
	//	delete fs;
#endif
	return true;
}

/****************************************************************
 //   Calculation of Rotation Matrix
 //
 *****************************************************************/
matrix33 forceSensorPlugin_impl::get_rot33(int dir, double rad)
{
	// Initialization
	matrix33 rot_temp;

	// before_R_after

	// Rotation about X
	if (dir == X)
	{
		rot_temp = 1, 0, 0, 0, cos(rad), -sin(rad), 0, sin(rad), cos(rad);
		return rot_temp;
	}

	// Rotation about Y
	if (dir == Y)
	{
		rot_temp = cos(rad), 0, sin(rad), 0, 1, 0, -sin(rad), 0, cos(rad);
		return rot_temp;
	}

	// Rotation about z
	if (dir == Z)
	{
		rot_temp = cos(rad), -sin(rad), 0, sin(rad), cos(rad), 0, 0, 0, 1;
		return rot_temp;
	}
	return rot_temp;
}

/*********************************************************************/
// Set Direct teaching method type
/*********************************************************************/
#if 0
::CORBA::Long forceSensorPlugin_impl::setDtType(::CORBA::ULong type)
{

#ifdef DEBUG_PLUGIN
	std::cout << "controlmode_nr =" << controlmode_nr << std::endl;
	std::cout << "controlmode_r ="  << controlmode_r << std::endl;
#endif

	controlmode_nr = type;

	while (controlmode_nr != controlmode_r)
	{
		usleep(1000);
	}
	return 0;
}
#endif
//~ /////////// Get rotation matrix of force sensor based on the DtMethod   ////////////////////
//~ ::CORBA::Long forceSensorPlugin_impl::getLArmFsRot(OpenHRP::DblArray9 rotMatrix)
//~ {
//~ int k=0;
//~ for(int i=0; i<3; ++i)
//~ for(int j=0; j<3; ++j) {rotMatrix[k]=LA.rRfs(i,j); k++; }

//~ return 0;
//~ }

/***************************************************************************
 * changeControlMode()
 ***************************************************************************/
#if 0
::CORBA::Long forceSensorPlugin_impl::changeControlMode(::CORBA::ULong type)
{
	controlmode_nr = type;
	//~ while(controlmode_nr != controlmode_r){
	//~ usleep(1000);
	//~ }
	return 0;
}
#endif
//////////////////////////////////////////////////////////
//
//       Force Sensor Realted function
//
//////////////////////////////////////////////////////////
/*short forceSensorPlugin_impl::IFS_Read(unsigned int addr)
  {

  short data = 0;
  unsigned long address = 0;

  address = MappedAddress + ((addr | Jr3DmAddrMask) << 2);
  data = *(short *)address;
  return data;

  }

  void forceSensorPlugin_impl::IFS_Write(unsigned int addr, int data)
  {
  unsigned long address = 0;

  address = MappedAddress + ((addr | Jr3DmAddrMask) << 2);
 *(int *)address = data;
  }*/

/*
  int forceSensorPlugin_impl::download(unsigned int base0, unsigned int base1)
  {
  int count;
  int index = 0;
  unsigned int Jr3BaseAddressH;
  unsigned int Jr3BaseAddressL;

  Jr3BaseAddressH = base0;
  Jr3BaseAddressL = base1;

  // The first line is a line count
  count = dsp[index++];

  // Read in file while the count is no 0xffff
  while (count != 0xffff)
  {
  int addr;
  // After the count is the address
  addr = dsp[index++];
  // loop count times and write the data to the dsp memory
  while (count > 0)
  {
  // Check to see if this is data memory or program memory
  if (addr & 0x4000)
  {
  int data = 0;
  // Data memory is 16 bits and is on one line
  data = dsp[index++];
  WriteJr3Dm(addr, data);
  count--;
  if (data != ReadJr3Dm(addr))
  {
  printf("data addr: %4.4x out: %4.4x in: %4.4x\n", addr, data, ReadJr3Dm(addr));
  }
  }
  else
  {
  int data, data2;
  int data3;
  // Program memory  24 bits and is on two lines
  data = dsp[index++];
  data2 = dsp[index++];
  WriteJr3Pm2(addr, data, data2);
  count -= 2;

  // Verify the write
  if (((data << 8) | (data2 & 0xff)) != (data3 = ReadJr3Pm(addr)))
  {
  //printf("pro addr: %4.4x out: %6.6x in: %6.6x\n", addr, ((data << 8)|(data2 & 0xff)), // ReadJr3Pm(addr)  data3);
  }
  }
  addr++;
  }
  count = dsp[index++];
  }

  return 0;

  }*/

/***********************************************************************
 * ReadFile
 *
 * Read commands from the file:
 * 	- calibFile
 *
 ***********************************************************************/
void forceSensorPlugin_impl::readInitialFile(const char *filename)
{
	// Declare an input stream
	ifstream infile(filename);

	// Read files
	while (!infile.eof())
	{
		string command;
		infile >> command;

		// Read Commands
		// Calibration
		if (command == "calibFile")
		{
			string fname;
			infile >> fname;
			cout << "calib file = " << fname.c_str() << endl;
			;

			ifstream calibfile(fname.c_str());
			while (!calibfile.eof())
			{
				string command;
				calibfile >> command;

				// Joint Limits
				if (command == "limit")
				{
					int jnum = 0;

					// For all joints
					while (jnum < DOF)
					{
						// Extract joint number
						int jid;
						calibfile >> jid;

						// Convert from degrees to radians
						for (int i = 0; i <= 1; ++i)
						{

							calibfile >> ang_limit[jid][i];
							ang_limit[jid][i] *= deg2radC;
						}
						++jnum;
					}
				}
			}
		}

		// Acceleration commands
		if (command == "velocity_accel_decel")
		{
			int jnum = 0;

			// For all joings
			while (jnum < DOF)
			{
				string num;
				infile >> num;

				if (num[0] == '#')
				{
					char dumy[1000];
					//next line
					//while ((int e = infile.get()) != '\n' && e != EOF);
					infile.getline(dumy, 1000);
				}
				else
				{
					int jid = atoi(num.c_str());
					for (int i = 2; i <= 4; ++i)
					{

						infile >> ang_limit[jid][i];
					}
					++jnum;
				}
			}
		}

		if (command == "velocity_gain")
		{
			double gain;
			infile >> gain;
			for (int jid = 0; jid < DOF; ++jid)
				ang_limit[jid][2] *= gain;
		}

		if (command == "accel_gain")
		{

			double gain;
			infile >> gain;
			for (int jid = 0; jid < DOF; ++jid)
				ang_limit[jid][3] *= gain;
		}

		if (command == "decel_gain")
		{

			double gain;
			infile >> gain;
			for (int jid = 0; jid < DOF; ++jid)
				ang_limit[jid][4] *= gain;
		}

#define READ_PARAM(param_name)						\
		if( command == #param_name )					\
		{								\
			infile >> param_name;						\
			std::cout << #param_name << " = " << param_name << "\n";	\
		}

		double MAX_IK_ITER;
		READ_PARAM( MAX_IK_ITER);
		double IK_GAIN;
		READ_PARAM( IK_GAIN);
		double MAX_IK_ITER_RT;
		READ_PARAM( MAX_IK_ITER_RT);
		double IK_GAIN_RT;
		READ_PARAM( IK_GAIN_RT);

		double Z_LIMIT_MM;
		READ_PARAM( Z_LIMIT_MM);
		//double DT;
		READ_PARAM( DT);

		double pathMaxV;
		READ_PARAM( pathMaxV);
		double pathMaxA;
		READ_PARAM( pathMaxA);
		double pathMaxD;
		READ_PARAM( pathMaxD);

		double ProtectiveStopCutoff;
		READ_PARAM( ProtectiveStopCutoff);
		double ProtectiveStopAccel;
		READ_PARAM( ProtectiveStopAccel);

		double ikGains[4];
		READ_PARAM( ikGains[0]);
		READ_PARAM( ikGains[1]);
		READ_PARAM( ikGains[2]);
		READ_PARAM( ikGains[3]);
		double ikRot[4];
		READ_PARAM( ikRot[0]);
		READ_PARAM( ikRot[1]);
		READ_PARAM( ikRot[2]);
		READ_PARAM( ikRot[3]);
		double ikTrans[4];
		READ_PARAM( ikTrans[0]);
		READ_PARAM( ikTrans[1]);
		READ_PARAM( ikTrans[2]);
		READ_PARAM( ikTrans[3]);
#undef READ_PARAM
	}

	for (int jid = 0; jid < DOF; ++jid)
	{

		printf("id:%3d llim:%9.3f ulim:%9.3f vel:%8.3f acc:%8.3f dec:%8.3f\n",
				jid, ang_limit[jid][0], ang_limit[jid][1], ang_limit[jid][2],
				ang_limit[jid][3], ang_limit[jid][4]);
	}

#ifdef DEBUG_PLUGIN
	std::cerr << "readInitialFile - finished" << std::endl;
#endif
}

/*//~ matrix33 forceSensorPlugin_impl::rotFromRpy(vector3 rpy_in)
//~ {
//~ double cos_x = cos(rpy_in(0)), sin_x = sin(rpy_in(0));
//~ double cos_y = cos(rpy_in(1)), sin_y = sin(rpy_in(1));
//~ double cos_z = cos(rpy_in(2)), sin_z = sin(rpy_in(2));

//~ matrix33 R;

//~ R(0,0) = cos_y*cos_z;
//~ R(0,1) = sin_x*sin_y*cos_z - cos_x*sin_z;
//~ R(0,2) = cos_x*sin_y*cos_z + sin_x*sin_z;
//~ R(1,0) = cos_y*sin_z;
//~ R(1,1) = sin_x*sin_y*sin_z + cos_x*cos_z;
//~ R(1,2) = cos_x*sin_y*sin_z - sin_x*cos_z;
//~ R(2,0) = -sin_y;
//~ R(2,1) = sin_x*cos_y;
//~ R(2,2) = cos_x*cos_y;

//~ return R;
//~ }*/

/***************************************************************
 * readGainFile()
 ***************************************************************/
void forceSensorPlugin_impl::readGainFile(const char *filename)
{
	// Declare Stream
	ifstream infile(filename);

	// Check if open
	if (!infile.is_open())
		std::cerr << "ERROR: can't open gain file." << std::endl;
	else
	{
		// Go throught the file and read commands
		while (!infile.eof())
		{
			string command;
			infile >> command;

			// 1) Extract P-gains
			if (command == "GainP")
			{
				int tmp = 0;

				while (tmp < 3)
				{
					infile >> command;
					int snum = 100; // Gain flag for high, normal, or low

					// Select according to gain classification
					if (command == "H")
						snum = 0;
					else if (command == "N")
						snum = 1;
					else if (command == "L")
						snum = 2;

					// If the flag was successfully assigned
					if (snum != 100)
					{
						int jnum = 0;

						// Set three gains
						while (jnum < 3)
						{
							infile >> GainP[snum][jnum];
							++jnum;
						}
					}
					++tmp;
				}
			}

			// 2) Extract R-gains
			if (command == "GainR")
			{
				int tmp = 0;

				while (tmp < 3)
				{
					infile >> command;
					int snum = 100; // Gain flag for high, normal, or low

					// Select according to gain classification
					if (command == "H")
						snum = 0;
					else if (command == "N")
						snum = 1;
					else if (command == "L")
						snum = 2;

					// If flas was successfully assigned
					if (snum != 100)
					{
						int jnum = 0;

						// Set three gains
						while (jnum < 3)
						{
							infile >> GainR[snum][jnum];
							++jnum;
						}
					}
					++tmp;
				}
			}
		}

		// Print both gain values

		// Gain P
		std::cout << "GainP: ";
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
				std::cout << GainP[i][j] << " ";
		}
		std::cout << std::endl;

		// Gain R
		std::cout << "GainR: ";
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
				std::cout << GainR[i][j] << " ";
		}
		std::cout << std::endl;
	}
}

/************************************************************************/
// get_tick()
// High performance counter
/************************************************************************/
unsigned long long get_tick()
{
	unsigned int l=0, h=0;
	__asm__ __volatile__("rdtsc": "=a" (l), "=d"(h) );
	return (unsigned long long)h<<32;
}

bool forceSensorPlugin_impl::loopback_condition()
{
	cur_time=DT*double(rArm->get_Iteration());
	std::cout<<cur_time<<std::endl;
	return cur_time > 11 && proState == 1;
}

void forceSensorPlugin_impl::loopback_collectData()
{
	char commend[256];
	sprintf(commend, "%s %s/%s/ %s", mkdir, dataCollectHomePath, deviation_toString(), "-p");// $: mkdir ~/home/of/data/resetTime/
	system (commend);
	sprintf(commend, "%s %s %s/%s/", mv, "./data/Results/*", dataCollectHomePath, deviation_toString());// $: mv ./data/Results/* ~/home/of/data/resetTime
	system (commend);
}


char* forceSensorPlugin_impl::deviation_toString ()
{
	sprintf(deviation_name, "LoopbackTest%cx%f%cy%f%cz%f%cxR%f%cyR%f%czR%f", //LoopbackTest+x0.00+y0.00+z0.00+xRoll0.00+yRoll0.00+zRoll0.00
			(deviation[0]>=0)?'+':'-',(deviation[0]>=0)?deviation[0]:(-deviation[0]),
			(deviation[1]>=0)?'+':'-',(deviation[1]>=0)?deviation[1]:(-deviation[1]),
			(deviation[2]>=0)?'+':'-',(deviation[2]>=0)?deviation[2]:(-deviation[2]),
			(deviation[3]>=0)?'+':'-',(deviation[3]>=0)?deviation[3]:(-deviation[3]),
			(deviation[4]>=0)?'+':'-',(deviation[4]>=0)?deviation[4]:(-deviation[4]),
			(deviation[5]>=0)?'+':'-',(deviation[5]>=0)?deviation[5]:(-deviation[5])
			);
	return deviation_name;
}
