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

#include "iob.h"				// Controller/IOServer/include
#include "forceSensorPlugin_impl.h"
//----------------------------------------------------------------------------------------------------------------------------------------------------
extern "C" {
#include <unistd.h>			// Provides access to the POSIX operating system API
#include <stdio.h>
}
//----------------------------------------------------------------------------------------------------------------------------------------------------
// FIRST THING TO DO: is to set if you will be working with the left arm (T/F).
// Then proceed to hiroArm.cpp and set design parameters there. And then to assemblyStrategy.cpp
//----------------------------------------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------------------------------------
// ARM CONFIGUARTIONS
//----------------------------------------------------------------------------------------------------------------------------------------------------
#define LEFT       	0   // Used to indicate index for f_control
#define RIGHT 			1 	// Used to indicate index for f_control

#define LEFT_ARM  		0 	// Flag to enable left Arm computations.
#define GRAVITY_COMP 	0 	// Flag to activate gravity compensation through a control_mode. Automatically set to call the PivotApproach after that.
//----------------------------------------------------------------------------------------------------------------------------------------------------

// HARDWARE BIT-WISE SWITCH DEFINITIONS
//----------------------------------------------------------------------------------------------------------------------------------------------------
// The HIRO robot is connected to a hardware box that can be modified in four ways to achieve different behvaiors.
#define MSSL  0x80000000    // Motion Select Switch-> left
#define MSSR  0x40000000    // Motion Select Switch-> right
#define ASSL  0x20000000    // Axis Select Switch-> left
#define ASSR  0x10000000    // Axis Select Switch-> right
#define DTES  0x08000000    // DT enable switch (for data acquisition)

// Gravity Compensation Parameter Files
#define GRAV_COMP_STAT_DATA_FILE 		"./data/GravComp/gravCompStatData.dat"
#define GRAV_COMP_PARAM_FILE     		"./data/GravComp/gravCompParams.dat"				// Also defined in hiroArm.cpp
// Local Coordinates
#define GRAV_COMP_RIGHT_WRENCH			"./data/Results/R_GC_Torques.dat"
#define GRAV_COMP_LEFT_WRENCH			"./data/Results/L_GC_Torques.dat"
// World Coordinates
#define GRAV_COMP_RIGHT_WORLD_WRENCH	"./data/Results/R_GC_worldTorques.dat"
#define GRAV_COMP_LEFT_WORLD_WRENCH	"./data/Results/L_GC_worldTorques.dat"
//----------------------------------------------------------------------------------------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------------------------------------------------------------------------------------
#define DB_TIME 			0			// Used to print timing duration of functions
#define FORCE_TEST 		0 			// Used to test if force drivers are working in Old HIRO
#define SIMULATION_RUN 	1	 		// Run if using simulation
#define DEBUG 				0 			// Used to print temporary cerr statements

//----------------------------------------------------------------------------------------------------------------------------------------------------
// GLOBALS
//----------------------------------------------------------------------------------------------------------------------------------------------------
unsigned long long distate; 			// Digital state parameter. Holds bit values.
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
	if(DEBUG)
		std::cerr << "forceSensorPlugin_impl entered constructor" << std::endl;

	DT 				= 0.005;   	// [sec]

	// Test Flags used within the control method
	initControl = 0;
	testFlag	= 0;
	num_test 	= 0;

	if(DEBUG)		std::cerr << "forceSensorPlugin_impl(): reading files" << std::endl;

	// Read Arm and Gain parameters
	readInitialFile("ArmParam");			// Velocity and Accelretion Gains and Trajectories
	readGainFile("GainParam");				// GainP and GainR for HNL modes
	if(DEBUG)		std::cerr << "forceSensorPlugin_impl(): finished reading files, initializing variables." << std::endl;

	// Pivot Approach Variables Initialization
	for(int i=0;i<3;i++)
	{
		CurXYZ(i)				= 0.0;		// 3D Point
	}
	for(int i=0;i<6;i++)
	{
		CurrentForces(i)		= 0.0;		// Right Arm
		worldCurrentForces(i)	= 0.0;
		CurrentAngles(i)		= 0.0;
		JointAngleUpdate(i)		= 0.0;
		L_CurrentForces(i)		= 0.0;		// Left Arm
		L_worldCurrentForces(i) = 0.0;
		L_CurrentAngles(i) 		= 0.0;
		L_JointAngleUpdate(i) 	= 0.0;
	}
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++){				// 3x3 Rotation Matrix
			CurRot(i,j) = 0.0;				// Right Arm
			L_CurRot(i,j) = 0.0;			// Left
		}
	if(DEBUG) std::cerr << "forceSensorPlugin_impl(): finished initializing variables" << std::endl;

	// Flags
	initFlag = false;

	if(DEBUG) std::cerr << "forceSensorPlugin_impl: leaving constructor" << std::endl;
}

forceSensorPlugin_impl::~forceSensorPlugin_impl()
{
	close_ifs();

	// Close Gravity Compensation Parameters Files
	if(ostr_gcWrenchR.is_open()) ostr_gcWrenchR.close();
	ostr_gcWrenchR.clear();

	if(ostr_gcWrenchL.is_open()) ostr_gcWrenchL.close();
	ostr_gcWrenchL.clear();

	if(ostr_gc_worldWrenchR.is_open()) ostr_gc_worldWrenchR.close();
	ostr_gc_worldWrenchR.clear();

	if(ostr_gc_worldWrenchL.is_open()) ostr_gc_worldWrenchL.close();
	ostr_gc_worldWrenchL.clear();
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

	if(DEBUG) {
		std::cerr << "In init()" << std::endl;
		std::cerr << "forceSensorPlugin::init()" << std::endl;
	}

	/*------------------------------------------------------------------------------- Load model file -------------------------------------------------------------------------------*/
	if (!CORBA::is_nil(manager->orb))
	{
		//  body data
		std::cerr << "loading Model: " << manager->url << std::endl;
		body = OpenHRP::loadBodyFromModelLoader((manager->url).c_str(),manager->orb);		// 使用方法は		// http://www.openrtp.jp/openhrp3/jp/calc_model.html
	}
	else
		std::cerr << "manager->orb is nil" << std::endl;

	if(DEBUG)		std::cerr << "forceSensorPlugin::init() - exiting" << std::endl;

	/*---------------------------------------------------------------- Setup Gravity Compensation Parameters -------------------------------------------------------*/
	// Initialize: TVMet Vector Struc's
	for (int i = 0; i < 2; i++)
	{
		F_ave[i] 	 =    0.0,    0.0,    0.0;
		M_ave[i] 	 =    0.0,    0.0,    0.0;
		F_sd[i]  	 =    0.0,    0.0,    0.0;
		M_sd[i]  	 =    0.0,    0.0,    0.0;
		F_err_max[i] = -100.0, -100.0, -100.0;
		M_err_max[i] = -100.0, -100.0, -100.0;
	}

	// Gravity Compensation Result Flag
	gravCompRes={0};

	/*------------------------------------------------------------------- Setup Force Sensor -------------------------------------------------------*/

	// ------------------ FORCE SENSOR ------------------------//
	// Open the oldHIro IFS sensor
	if(DEBUG)	std::cerr << "forceSensorPlugin::Opening the ifs force lib" << std::endl;

	#ifndef SIMULATION
		open_ifs();
		//	if (!fs->init())
		//		std::cerr << "error: fs->init()" << std::endl;
	#endif

	if(DEBUG)		std::cerr << "forceSensorPlugin::init() - Closeing the ifs force lib" << std::endl;

	/*------------------------------------------------------------------------------- Setup LEFT and RIGHT arms angle limits -------------------------------------*/
	float ang_limit_sub[ARM_DOF][5];

#ifdef PIVOTAPPROACH
	//---------------------- LEFT arm ----------------------------//
	if(LEFT_ARM)
	{

		if(DEBUG)	std::cerr << "ang_limit_sub left arm: " << std::endl;

		for (int i = 0; i < ARM_DOF; i++) {
			for (int j = 0; j < 5; j++) 	{
				ang_limit_sub[i][j] = ang_limit[9 + i][j];

				if(DEBUG)	std::cerr << ang_limit_sub[i][j] << " ";
			}
			if(DEBUG)	std::cerr << std::endl;
		}

		// Position and Rotation Variables
		//ePh=-0.052, 0.0, 0.0;//-0.0595, 0.00, -0.0255; // -0.051, 0.0020, -0.0225; //0.0715, 0.0, 0.0;
		femaleCam_4snaps_left_ePh = -0.0915, 	0.00, 	-0.0255;									// Wrist to EndEffector translation for female_cam part.
		ePh = femaleCam_4snaps_left_ePh;
		eRh = 1.0, 		0.0, 	0.0,
			  0.0, 		1.0, 	0.0,
			  0.0, 		0.0, 	1.0;										// Wrist to EndEffector rotation
		hPfs = (0.0085 + (0.0315 / 2)), 0.0, 0.0;							// Wrist to Force Sensor Tool Center Point

		// Allocate the Master Arm for Hiro (Left Arm)
		if(DEBUG)	std::cerr << "forceSensorPlugin::Allocating LEFT hiroArm" << std::endl;

		int left_NUM_q0 = 9;
		lArm = new hiroArmMas("left", body, left_NUM_q0, DT, ang_limit_sub, ePh, eRh, hPfs);

		if(DEBUG)	std::cerr << "forceSensorPlugin::Finished allocating LEFT hiroArm" << std::endl;

		// Set the force pointer
		// lArm->set_FSptr(fs, 0);										// Connect to Force Sensor
		// arg2: int *full_scale
	}

	//----------------------------------------------------------------------------- RIGHT ARM --------------------------------------------------------------------------------------//
	if(DEBUG)	std::cerr << "Right Arm: ang_limit_sub right arm: " << std::endl;

	// Position and Rotation Variables in local wrist coordinates. rot mat={0 0 -1 0 1 0 1 0 0}
	//ePh=//-0.051 , 0.0020 , -0.0225; //-0.0785, 0.0020, -0.0245;  //-0.051, 0.0020, -0.0225;//0.025, 0.0, -0.063; // -0.063, 0.0, -0.025;	// -0.127, 0.025, 0.0;	//-0.127, -0.03, -0.005;
	maleCam_4snaps_right_ePh = -0.0785, 0.0020, -0.0245; 								// ePh for snap male camera.
	ePh =  maleCam_4snaps_right_ePh;  													// Wrist to EndEffector translation for male_cam_part.
																						// -0.127=height,0.025=frontal, 0.0=horizontal
	eRh = 1.0, 0.0, 0.0,
			0.0, 1.0, 0.0,
			0.0, 0.0, 1.0;

	// Wrist to EndEffector rotation
	hPfs = 0.063 + (0.0315 / 2), 0.0, 0.0;												// EndEffector to Force Sensor Tool Center Point Translation	//hPfs = 53+(31.5/2), 0.0, 60.0;
	hRfs = get_rot33(Y, M_PI / 2.0) * get_rot33(Z, -(3 * M_PI / 4.0));					// EndEffector to Force Sensor Tool Center Point Rotation

	// Allocate the Master Arm for Hiro (Left Arm)
	if(DEBUG)	std::cerr << "forceSensorPlugin()::Allocating RIGHT hiroArm" << std::endl;

	// Allocate the slave arm - right arm
	int NUM_q0 = 3; // Joint angle at which the right arm starts
	rArm = new hiroArmSla("right", body, NUM_q0, DT, ang_limit_sub, ePh, eRh, hPfs, hRfs);

	if(DEBUG)	std::cerr << "forceSensorPlugin()::Finished allocating RIGHT hiroArm" << std::endl;

	//rArm->set_FSptr(fs, 1);
	//initialize_ifs(0);

	if(DEBUG)	std::cerr << "forceSensorPlugin::Finished allocating LEFT hiroArm" << std::endl;

	// Assign Joint Angle Limits
	for(int i = 0; i < ARM_DOF; i++){									// 6 DOF per arm
		for(int j = 0; j < 5; j++){
			ang_limit_sub[i][j] = ang_limit[3 + i][j];
			if(DEBUG)
				std::cerr << ang_limit_sub[i][j] << " ";

		}
		if(DEBUG)
			std::cerr << std::endl;
	}

	// If not PivotApproach
#else
	// Position and Rotation Variables
	ePh = -0.12, 0.0, 0.0;																// Wrist to EndEffector translation								//ePh = -(53.0+31.5+16.0), 0.0, -60.0;
	eRh = 1.0, 0.0, 0.0,
			0.0, 1.0, 0.0,
			0.0, 0.0, 1.0;																// Wrist to EndEffector rotation
	hPfs = 0.074 + (0.0315 / 2), 0.0, 0.0;												// EndEffector to Force Sensor Tool Center Point Translation	//hPfs = 53+(31.5/2), 0.0, 60.0;
	hRfs = get_rot33(Y, M_PI / 2.0) * get_rot33(Z, -(3 * M_PI / 4.0));					// EndEffector to Force Sensor Tool Center Point Rotation

	// Allocate the slave arm - right arm
	int NUM_q0 = 3; // Joint angle at which the right arm starts
	rArm = new hiroArmSla("right", body, NUM_q0, DT, ang_limit_sub, ePh, eRh, hPfs, hRfs);
	//rArm->set_FSptr(fs, 1);
	//IFS_Init(CHANNEL,FULL_SCALE);

	// Assign Joint Angle Limits
	for(int i = 0; i < ARM_DOF; i++){													// 6 DOF per arm
		for(int j = 0; j < 5; j++){
			ang_limit_sub[i][j] = ang_limit[3 + i][j];
			if(DEBUG)
				std::cerr << ang_limit_sub[i][j] << " ";
		}
		if(DEBUG)
			std::cerr << std::endl;
	}
#endif // If pivot approach

	//-------------------------------------- Initialize Control Flags ----------------------------------------------------------//
	// Set controlmode_nr/controlmode_r GravityCompensration | PivotApproach. GravityCompensation if we are using both arms.
	f_gravity_comp[0] = f_gravity_comp[1] = false;

	controlmode_nr = GravityCompensation;
	controlmode_r  = GravityCompensation;

	if(DEBUG) std::cerr << "out init()" << std::endl;

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

	if(DEBUG)	std::cerr << "forceSensorPlugin::setup() - entered" << std::endl;

	/*------------------------------------------------------------- Simulation ---------------------------------------------------*/
	#ifdef SIMULATION
		for(int i=0; i<6; i++)
		{
			if(LEFT_ARM)
				lArm->raw_forces[i] = rs->force[1][i];
				rArm->raw_forces[i] = rs->force[0][i];
		}
	#endif
		// ----------------------------------------------------------- FORCE SENSOR -----------------------------------------------------------------/

	#ifndef SIMULATION
		initialize_ifs(0);
		initialize_ifs(1);

		rs->angle[8] -= 0.531117;
	#endif

	if(DEBUG)	cerr << "forceSensorPlugin::setup() - finished\n";

	/*------------------------------------------------------------ Current Joint Angles --------------------------------------------------------*/
	// 2) Set the desired robot state angles equal to the motion command angles, to avoid jump in the data or a discontinuity
	// Set output angles to input angles
	for (unsigned int i=0; i<DOF; ++i)	// 15 DoF
	{
		body->joint(i)->q 	= rs->angle[i];
		mc->angle[i] 		= rs->angle[i];
		CurAngles[i] 		= rs->angle[i]; 			// Set the local variables to this
	}

	// Set CurrentAngles: private member variable to rs->angle
	int      NUM_q0 = 3;
	int left_NUM_q0 = 9;

	for(int i=0; i<6; i++) {
		CurrentAngles[i]  =rs->angle[NUM_q0+i];
		L_CurrentAngles[i]=rs->angle[left_NUM_q0+i];
	}

	// Not sure about this adjustment, but it is reversed in control.
	#ifndef SIMULATION
		mc->angle[8] += 0.531117;
	#endif

	if(DEBUG)	std::cerr 	<< "\n\nThe 15 body joint angles in radians are: " << CurAngles << std::endl;

	// 3) Calculate the Fwd Kinematics=>Cartesian Positions
	body->calcForwardKinematics();

	/****************************************** PIVOT APPROACH SETUP ************************************************************/

	// 4) Initialization routine:
	// Compute the position vector and rotation matrix from base to the wrist the robot has moved to HOMING POSITION
	// This routine behave's different according to user. Assign Desired Behavior.
	// Kensuke: read position and force data for both arms.
	if(DEBUG)	std::cerr << "forceSensorPlugin::setup() - Arm Initialization" << std::endl;

	/*----------------------- LEFT ARM ----------------------*/
	if(LEFT_ARM)
	{
		lArm->init(body->link(LARM_JOINT5)->p, 					// Initialize Left Arm HIRO class. Provide end-effector position, orientation, and current angles.
				   body->link(LARM_JOINT5)->attitude(),
				   CurAngles);

		// Number of joints
		int n2=0;
		n2 = lArm->m_path->numJoints();
		if(DEBUG)	cerr << "forceSensorPlugin::setup(). LEFT Num Joints: " << n2 << std::endl;

		// Position
		vector3 rpy2(0);
		lArm->set_OrgPosRot(L_CurXYZ,rpy2);

		if(DEBUG)
		{
			std::cerr << "forceSensorPlugin::setup(). LEFT EndEffector Position: " 			<< L_CurXYZ 		<< std::endl;
			std::cerr << "forceSensorPlugin::setup(). LEFT EndEffector Pose: " 				<< rpy2 			<< std::endl;
			std::cerr << "forceSensorPlugin::setup(). LEFT Current Angles (radians):\t" 	<< L_CurrentAngles 	<< std::endl;
		}
	}

	/*----------------------- RIGHT ARM ----------------------*/
	ret = rArm->init(body->link(RARM_JOINT5)->p, 			// Initialize Left Arm HIRO class. Provide end-effector position, orientation, and current angles.
			         body->link(RARM_JOINT5)->attitude(),
			         CurAngles);

	// Update the latest data angles and position
	if(DEBUG)
	{
		// Number of joints
		int n=0;
		n = rArm->m_path->numJoints();
		std::cerr << "forceSensorPlugin::setup(). RIGHT Num Joints: " << n << std::endl;

		// Position
		vector3 rpy(0);
		rArm->set_OrgPosRot(CurXYZ,rpy);
		std::cerr << "forceSensorPlugin::setup(). RIGHT EndEffector Position: " 	 << CurXYZ 			<< std::endl;
		std::cerr << "forceSensorPlugin::setup(). RIGHT EndEffector Orientation: " 	 << rpy 			<< std::endl;
		std::cerr << "forceSensorPlugin::setup(). RIGHT Current Angles (radians):\t" << CurrentAngles 	<< std::endl;
	}

	// Check result
	if(ret==-1)
		std::cerr << "\nProgram Failed!!";
	else
		if(DEBUG)	std::cerr <<"forceSensorPlugin::setup() - Initialization was successful!" << std::endl;

	// Set Control Modes
	if(GRAVITY_COMP) {								// Gravity Compensation currently terminates with a call to PivotApproach - June 2015.
		controlmode_nr 	 = GravityCompensation;
		controlmode_r 	 = GravityCompensation;
	}
	else {
		controlmode_r  = PivotApproach;
	}

	if(DEBUG) 	std::cerr << "forceSensorPlugin::setup() - Exiting" << std::endl;

	return true;
}

/******************************************************************************************************************************************/
// Control()
// The control function is called by the GrxUI simulation every 2-5ms. This method implements the control routine that will control HIRO.
// This function receives the RobotState rs which is a structure containing all the data of the robot:
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
	int ret    = 0;

	#ifndef SIMULATION
		rs->angle[8] -= 0.531117;	// This number was added in setup and reversed here.
	#endif

	//---------------------------------------- FORCE TEST ------------------------------------------/
	if(!FORCE_TEST)
	{
		if(DEBUG)	std::cerr << "/----------------------------------\nforceSensorPlugin::control() - entered\n-------------------------------------/\n\n" << std::endl;

		// Initial Loop Code: Pose and Joint Angles.
		if(initControl==0)
		{
			// Initialize the iteration
			if(LEFT_ARM) lArm->m_time=0;
						 rArm->m_time=0;


			//---------------------------------------- Update End Effector Pose and Joint Angles ------------------------------------------/
			if(DEBUG) std::cerr << "\n\nforceSensorPlugin::control()-Current time is:\t" 	<< DT*double(rArm->get_Iteration()) << std::endl;

			vector3 rpy(0), rpy2(0);

			rArm->set_OrgPosRot(CurXYZ,rpy);									// Set EE pose
			if(DEBUG) std::cerr << "Right Pose (RPY)" << CurXYZ << rpy << std::endl;

			// IK Computation
			rArm->m_path->calcInverseKinematics(CurXYZ,CurRot);
			// Save orientation, not done correctly above
			CurRot=rArm->m_path->joint(5)->attitude();
			for (int i=0;i<6;i++) CurrentAngles(i) = rArm->m_path->joint(i)->q;

			if(LEFT_ARM)
			{
				lArm->set_OrgPosRot(L_CurXYZ,rpy2);
				if(DEBUG) std::cerr << "Left Pose (RPY)" << L_CurXYZ << rpy2 << std::endl;

				// IK Computation
				lArm->m_path->calcInverseKinematics(L_CurXYZ,L_CurRot);
				// Save orientation, not done correctly above
				L_CurRot=lArm->m_path->joint(5)->attitude();
				for(int i=0;i<6;i++) L_CurrentAngles(i) = lArm->m_path->joint(i)->q;
			}

			// Change the flag
			initControl=1;
		}

		// Simulation test is used to skip code that is enclosed in the if function.
		if(SIMULATION_RUN)
		{

			// Timing
			timeval startTime, endTime;
			double duration = 0;
			if(DB_TIME) gettimeofday(&startTime,NULL); 						// Initialize startTime


			/*-------------------------------------------------------------- Arm Selection ---------------------------------------------------*/
			// What arm will you use. Left = 0 index. Right = 1 index.
			// Use Right Arm
			bool f_control[2];
			f_control[RIGHT] = true;

			// Use Left Arm and Right Arm
			if(LEFT_ARM) f_control[LEFT] = true;
			else         f_control[LEFT] = false;

			/*----------------------------------------------------------------- Force Values ------------------------------------------------------*/
			#ifdef SIMULATION
				if(DEBUG)	std::cerr << "Getting Force Values" << std::endl;
					for(int i=0; i<6; i++)
					{
						if(LEFT_ARM)
							lArm->raw_forces[i] = rs->force[1][i];
							rArm->raw_forces[i] = rs->force[0][i];
					}

			#else
				#if 0
					// Read DIN state
					read_di(distate);
					DioState = distate >> 32;
				#endif
			#endif

			/*------------------------------------------------------------ INITIALIZATION STAGE ----------------------------------------------*/
			if(initFlag==false)
			{

				/*---------------------- Angle Update --------------------------*/
				// For the first iteration make sure that output angles are the same as input angles for all 15 DOF.
				for (int i=0; i<DOF; i++)
				{
					body->joint(i)->q 	= rs->angle[i];	// the body object is for the entire body 15 DoF
					mc->angle[i] 		= rs->angle[i];
				}

				// change flag
				initFlag = true;

				if(DEBUG)	std::cerr << "forceSensorPlugin - The 15 current angles in radians are: " << rs->angle << std::endl;

				/*---------------------- Open Gravity Compensation Files --------------------------*/
				// Local Coordinates
				// Open the right arm gravity compensated torques file: forces_gc
				ostr_gcWrenchR.open(GRAV_COMP_RIGHT_WRENCH);
				if (!ostr_gcWrenchR.is_open())	std::cerr << "File: " << GRAV_COMP_RIGHT_WRENCH << " was not opened." << std::endl;

				// Open the left arm gravity compensated torques file: forces_gc
				ostr_gcWrenchL.open(GRAV_COMP_LEFT_WRENCH);
				if(!ostr_gcWrenchL.is_open())	std::cerr << "File: " << GRAV_COMP_LEFT_WRENCH << " was not opened." << std::endl;

				// World Coordinates
				// Open the right arm gravity compensated torques file: forces_gc
				ostr_gc_worldWrenchR.open(GRAV_COMP_RIGHT_WORLD_WRENCH);
				if (!ostr_gc_worldWrenchR.is_open())	std::cerr << "File: " << GRAV_COMP_RIGHT_WORLD_WRENCH << " was not opened." << std::endl;

				// Open the left arm gravity compensated torques file: forces_gc
				ostr_gc_worldWrenchL.open(GRAV_COMP_LEFT_WORLD_WRENCH);
				if(!ostr_gc_worldWrenchL.is_open())	std::cerr << "File: " << GRAV_COMP_LEFT_WORLD_WRENCH << " was not opened." << std::endl;
				/*---------------------- Start Timing Loop --------------------------*/
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

			/*---------------------- Compute FKins --------------------------*/
			if(DEBUG) std::cerr << "Calculate Fwd Kins" << std::endl;

			for (unsigned int i = 0; i < DOF; ++i)
			{
				body->joint(i)->q 	= rs->angle[i];			// Body object updates entire body (15 DoF)
				mc->angle[i] 		= rs->angle[i];			// Set final outgoing mc->angles to input to prevent jumps in non-updated joints.
			}
			// Calculate cartesian positions
			body->calcForwardKinematics();


			if(DEBUG)	std::cerr << "Update the EndEffector state data (pos, rot, joints)." << std::endl;

			// Update Arms Pose and Angles
			if(LEFT_ARM)
				lArm->update_currposdata();			// Gets base2wrist/base2endeffector translation/rotation and current joint angles
				rArm->update_currposdata();

			/*------------------------------------------------------------- Select Control Mode-----------------------------------------------------------------------*/
			// There are several control modes: NotControlled, GravityCompensation, ResetGravityCompensation, ... ,Pivot Approach
			if(DEBUG) std::cerr << "Current  mode is: " << controlmode_r << std::endl;

			switch (controlmode_r)
			{

			/*--------------------------------------------------------------------------- Not Controlled ----------------------------------------------------*/
			case NotControlled:
				if(DEBUG) std::cerr << "NotControlled" << std::endl;

				f_control[LEFT] = f_control[RIGHT] = false;
				break;

			/*------------------------------------------------------------------------ Gravity Compensaation -------------------------------------------------*/

			case GravityCompensation:
				// Hierarcy of class happens as follows;
				// 1. hiroArm::gravity_comp()
				// 2. hiroArm::calc_gravity_param()
				// 3. hiroArm::update_currforcedata()
				// 4. hiroArm::calc_gc_forces()

				if(DEBUG)	std::cerr << "GravityCompensation" << std::endl;

				// (A) Checking to see if calib parameters are on file. If yes, skip calib routine.
				ret=readGravCompStatData(GRAV_COMP_STAT_DATA_FILE);
				if(ret)
				{
					// Use either arm object to call readGravCompParamData and read data for both arms.
					long position=0;
					f_gravity_comp[0]=lArm->readGravCompParamData("left", GRAV_COMP_PARAM_FILE,position);
					if(f_gravity_comp[0])
					{
						// Set flags to true
						gravCompRes[0]=1;
						lArm->f_gc=true;
						f_gravity_comp[1]=rArm->readGravCompParamData("right", GRAV_COMP_PARAM_FILE,position);
						if(f_gravity_comp[1])
						{
							// Set flags to true
							gravCompRes[1]=1;
							rArm->f_gc=true;
							std::cerr << "Gravity Compensation already calibrated." << std::endl;

							// Call PivotApproach
							controlmode_r = PivotApproach;
							break;
						}
					}
				}

				// (B) Perform Gravity Compensation Calibration Routine
				// LEFT ARM
				if (!f_gravity_comp[0])
				{
					// Call gravity compsensation: 2000 step iteration algorithm computing gravity vectors.
					gravCompRes[0] = lArm->gravity_comp();
					if(DEBUG)	std::cerr << "Gravity Compensation: res_gc = " << gravCompRes[0] << std::endl;

					// Success
					if (gravCompRes[0] == 1)
					{
						f_gravity_comp[0] = true;
						num_test 		  = 0;
					}
					else if (gravCompRes[0] == 0)
						f_control[LEFT] = true;
				}

				else if (!f_gravity_comp[1])
				{
					// Call gravity compensation
					gravCompRes[1] = rArm->gravity_comp();

					if(DEBUG)
						std::cerr << "Gravity Compensation: res_gc = " << gravCompRes[1] << std::endl;

					// Success
					if (gravCompRes[1] == 1)
					{
						f_gravity_comp[1] 	= true;
						num_test 			= 0;
						std::cerr << "GC succeeded." << std::endl;
					}
					else if (gravCompRes[1] == 0)
						f_control[RIGHT] = true;
				}

				// No gravity compensation
				else
				{
					// Update Force Sensor Data
					lArm->update_currforcedata();
					rArm->update_currforcedata();

					vector3 rF_gc[2], rM_gc[2];

					// Get hand forces and moments
					lArm->get_forces(rF_gc[0], rM_gc[0]);
					rArm->get_forces(rF_gc[1], rM_gc[1]);

					if (num_test < 2000)
					{
						if (num_test == 0)
						{
							// Initialize. // Juan: tricky as changing these vectors to a multidimensional vectors, most of size [2][3]
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
					}

					// Last Iteration
					else if (num_test == 2000)
					{
						std::string f_name1 = GRAV_COMP_STAT_DATA_FILE;
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

							if(DEBUG)
							{
								std::cerr << "error of rF_gc = " << F_ave << std::endl;
								std::cerr << "error of rM_gc = " << M_ave << std::endl;
								std::cerr << "max error F" << F_err_max << std::endl;
								std::cerr << "max error M" << M_err_max << std::endl;
							}
						}

						// Close file and Increase Counter
						std::cerr << "Gravity Compensation is finished." << std::endl;
						fclose(fp);
						num_test++;

						// Change control mode
						controlmode_r = PivotApproach;
					}
				}
				break;

				/*---------------------------------------------------------------------------------------------- Reset Gravity Compensation -----------------------------------------------------------------------------------------*/
			case ResetGravityCompensation:

				if(DEBUG) std::cerr << "ResetGravityCompensation" << std::endl;
				lArm->reset_gravity_comp();	rArm->reset_gravity_comp();
				f_gravity_comp[LEFT] = f_gravity_comp[RIGHT] = false;
				break;

				/*---------------------------------------------------------------------------------------------- Pivot Approach -----------------------------------------------------------------------------------------*/
			case PivotApproach:

				if(DEBUG) std::cerr << "forceSensorPlugin::PivotApproach case - Entering" << std::endl;

				// Select Right Arm
				f_control[RIGHT] = true;

				//------------------------------------------------ Update Pose Data -------------------------------------------------------------//
				// Get current hand (endeffector) position and rotation transformations wrt the base
				if(LEFT_ARM)
					lArm->get_curr_handpos(L_CurXYZ,L_CurRot);
					rArm->get_curr_handpos(CurXYZ,CurRot);

				// Perform necessary procedures to run the inverse kinematic computations later
				/*for(int j=0; j<4; j++) { rArm->m_path->setIkGain(rArm->ikGains[j]); rArm->m_path->setMaxIKErrorRot(rArm->ikRot[j]); rArm->m_path->setMaxIKErrorTrans(rArm->ikTrans[j]); } */

				if(DEBUG) std::cerr << "\n\n!!!!! forceSensorPlugin::control(). CurXYZ: " << CurXYZ << std::endl;

				// Copy current angles
				for(int i=0; i<ARM_DOF; i++)
				{
					if(LEFT_ARM)
						L_CurrentAngles(i) 	= body->joint(i+9)->q;	// Left arm starts at i+9.
						CurrentAngles(i) 	= body->joint(i+3)->q;	// Right arm start at i+3. i.e. chest,pan,tilt,rArm,lArm.
				}

				if(DEBUG) std::cerr << "Current Joint Angles are:\t" << CurrentAngles << std::endl;

				//------------------------------------------------ Update Wrench Data -------------------------------------------------------------//
				#ifdef SIMULATION
				// Update latest force: either raw or gravity compensated.
				if(LEFT_ARM)
					lArm->update_currforcedata();
					rArm->update_currforcedata();

				// Read Hand Gravity Compensated Forces into local variables if used for left and right arms.
				vector3 rF_gc[2], rM_gc[2];
				if(LEFT_ARM && gravCompRes[0])	lArm->get_forces(rF_gc[0], rM_gc[0]);
				if(gravCompRes[1])				rArm->get_forces(rF_gc[1], rM_gc[1]);

				// Copy force data over to CurrentForces for Left and Right Arms
				for( int i = 0 ; i < 3 ; i++ ){
					if(LEFT_ARM) {
						if(f_gravity_comp[0]) {
							L_CurrentForces(i)   = rF_gc[0][i];
							L_CurrentForces(3+i) = rM_gc[0][i];
						}
						else
							for(int i=0;i<6;i++) L_CurrentForces(i) = lArm->raw_forces[i];
					}
					if(f_gravity_comp[1]) {
						CurrentForces(i)   = rF_gc[1][i];
						CurrentForces(3+i) = rM_gc[1][i];

					}
					else
						for(int i=0;i<6;i++)   CurrentForces(i) = rArm->raw_forces[i];
				}

				// Convert Local Gravity Compensated Force into World Coordinates
				vector3 temp1,temp2;

				if(LEFT_ARM && f_gravity_comp[0])
				{
					temp1 = L_CurRot*rF_gc[0];
					temp2 = L_CurRot*rF_gc[0];

					// Copy to 6 dimensional vector
					for(int i=0;i<3;i++)
					{
						L_worldCurrentForces(i)   = temp1(i);
						L_worldCurrentForces(i+3) = temp2(i);
					}
				}

				if(f_gravity_comp[1])
				{
					temp1 = CurRot*rF_gc[1];
					temp2 = CurRot*rF_gc[1];

					// Copy to 6 dimensional vector
					for(int i=0;i<3;i++)
					{
						worldCurrentForces(i)   = temp1(i);
						worldCurrentForces(i+3) = temp2(i);
					}
				}

				/****************************************** WRITE DATA TO FILE ******************************************/
				// Write gravity compensated wrenches in LOCAL coordinates for releavnt arm.
				ostr_gcWrenchR << cur_time << "\t";
				for(int i=0; i<6; i++) ostr_gcWrenchR << CurrentForces(i) << " \t";
				ostr_gcWrenchR << std::endl;

				if(LEFT_ARM) {
				ostr_gcWrenchL << cur_time << "\t";
				for(int i=0; i<6; i++) ostr_gcWrenchL << L_CurrentForces(i) << " \t";
				ostr_gcWrenchL << std::endl;
				}

				// Write gravity compensated wrenches in WORLD coordinates for relevant arm.
				ostr_gc_worldWrenchR << cur_time << "\t";
				for(int i=0; i<6; i++) ostr_gc_worldWrenchR << worldCurrentForces(i) << "\t";
				ostr_gc_worldWrenchR << std::endl;

				if(LEFT_ARM) {
				ostr_gc_worldWrenchL << cur_time << "\t";
				for(int i=0; i<3; i++) ostr_gc_worldWrenchL << L_worldCurrentForces(i)   << "\t";
				ostr_gc_worldWrenchL << std::endl;
				}

				if(DEBUG)
				{
					if(LEFT_ARM) std::cerr << "Left Current Wrench before calling SM is:\t" << L_CurrentForces << std::endl;
					else 		  std::cerr << "Right Current Wrench before calling SM is:\t" << CurrentForces   << std::endl;
				}

#else
				// For real robot need to correct force orientation.
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

				// Rotate and change direction.
				rotated_force = mod_rot * tmp_force;
				rotated_moment = mod_rot * tmp_moment;

				CurrentForces(0) = -rotated_force(0);
				CurrentForces(1) = rotated_force(1);
				CurrentForces(2) = rotated_force(2);
				CurrentForces(3) = -rotated_moment(0);
				CurrentForces(4) = rotated_moment(1);
				CurrentForces(5) = rotated_moment(2);
#endif

				// Controller time counter
				cur_time=DT*double(rArm->get_Iteration());		// sampling time * iteration
				if(LEFT_ARM)
					lArm->set_Iteration();						// Increase iteration
					rArm->set_Iteration();

				//------------------------------------------------ Call PivotApproach -------------------------------------------------------------//

				// Call PivotApproach and retrieve a Joint angle update and CurrentAngles
				if(DEBUG)	std::cerr << "\nforceSensorPlugin::control - Calling hiroArm::PivotApproach()" << std::endl;

				if(LEFT_ARM) {
					ret=lArm->PivotApproach(cur_time,L_CurXYZ,L_CurRot,L_CurrentForces,L_JointAngleUpdate,L_CurrentAngles);
					if(ret==-1)
					{
						std::cerr << "ControlComposition failed for Left Arm. \nCurrent time:\t" << cur_time <<
								".\nCurrent iteration: " 		<< rArm->get_Iteration() 	<<
								".\nCurrent Position: " 		<< L_CurXYZ 				<<
								".\nCurrent Orientation Mat: "	<< L_CurRot 				<<
								".\nCurrent Angles: " 			<< L_CurrentAngles 			<<
								".\nCurrent Forces: " 			<< L_CurrentForces 			<< std::endl;
					}
				}

				ret=rArm->PivotApproach(cur_time,CurXYZ,  CurRot,  CurrentForces,  JointAngleUpdate,  CurrentAngles);
				if(ret==-1)
				{
					std::cerr << "ControlComposition failed for Right Arm.\nCurrent time:\t" << cur_time <<
							".\nCurrent iteration:" 		<< rArm->get_Iteration() 	<<
							".\nCurrent Position:" 			<< CurXYZ 					<<
							".\nCurrent Orientation Mat: " 	<< CurRot 					<<
							".\nCurrent Angles: " 			<< CurrentAngles 			<<
							".\nCurrent Forces: " 			<< CurrentForces 			<< std::endl;
				}
				break;
			}

			/*---------------------------------------------------------------------------------------------- Arm Control -----------------------------------------------------------------------------------------*/
			// Set Desired Joint Angles if PA is NOT defined
			#ifndef PIVOTAPPROACH
						for (int i = 0; i < DOF; i++) mc->angle[i] = rs->angle[i];
			#endif

			// Set the reference angles only.
			// The simulation updates the motions.

			/************************************************************************* Left Arm *******************************************************************************/
			if (f_control[LEFT])
			{
				dvector6 qref_l = lArm->get_qref();

				// MC Angles 9-14 belong to the left arm
				for (int i = 0; i < 6; i++)
					mc->angle[9 + i] = qref_l[i];

				if(DEBUG)
				{
					std::cerr << "q_ref to mc: ";

					for(int i=0; i<6; i++)	std::cerr << mc->angle[9+i] << ", ";
					std::cerr << std::endl;
				}
			}

			/************************************************************************* Right Arm *******************************************************************************/
			if (f_control[RIGHT])
			{
				// Retrive the updated reference joint angles computed by the pivot approach
				dvector6 qref_r = rArm->get_qref();

				// Copy those reference joint angles to the motion command object to move hiro's arm.
				// MC Angles 3-8 belong to the right arm
				for (int i=0; i < ARM_DOF; i++)
					mc->angle[i+3] = qref_r[i];

				mc->angle[0]=0; mc->angle[1]=0; mc->angle[2]=0;	// Always set the chest, pan, tilt angles to zero.
			}

			if(DEBUG)
			{
				std::cerr << "\nforceSensorPlugin - The current angles in degrees (including chest and head) are: " << mc->angle[0]*rad2degC << " " << mc->angle[1]*rad2degC << " " << mc->angle[2]*rad2degC << " "
						<< mc->angle[3]*rad2degC << " " << mc->angle[4]*rad2degC << " " << mc->angle[5]*rad2degC << " "
						<< mc->angle[6]*rad2degC << " " << mc->angle[7]*rad2degC << " " << mc->angle[8]*rad2degC << std::endl;
			}
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

		} // END SIMULATION TEST

		else
		{

			rArm->update_currforcedata();

			for(int i=0;i<3;i++)
			{
				CurrentForces(i)   = rArm->fsF_raw[i];
				CurrentForces(i+3) = rArm->fsM_raw[i];
			}

			// Controller time counter
			cur_time=DT*double(rArm->get_Iteration());      						// sampling time * iteration
			rArm->set_Iteration();                          						// Increase iteration           // Controller time counter

			if(DEBUG) cerr << "Time and Forces: " << cur_time << "\t" << CurrentForces << std::endl; 				// Print value to the error console
		}

	} // END FORCE TEST

	#ifndef SIMULATION
		mc->angle[8] += 0.531117;
	#endif
}

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
			cerr << "calib file = " << fname.c_str() << endl;
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
			std::cerr << #param_name << " = " << param_name << "\n";	\
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

	if(DEBUG)
		std::cerr << "readInitialFile - finished" << std::endl;
}

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
		std::cerr << "GainP: ";
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
				std::cerr << GainP[i][j] << " ";
		}
		std::cerr << std::endl;

		// Gain R
		std::cerr << "GainR: ";
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
				std::cerr << GainR[i][j] << " ";
		}
		std::cerr << std::endl;
	}
}

/***********************************************************************
 * Read GravityCompesnationStatData
 * Reads parameters from the file. There are parameters for left and
 * right arms. For each arms you also have the following parameters:
 * 		sensor# - which is the tile of the sensor to use
 * 		average: 		F_ave[i][0-2], 		M_ave[i][0-2]
 * 		s_deviation:	F_sd[i][0-2], 		M_sd[i][0-2]
 * 		max_error(6):	F_err_max[i][0-2], 	M_err_max[i][0-2],
 * 		zero_limit(2):	F_zero_limit[i],	M_zero_limit[i] // index 0 is for left, index 1 is for right.
 *
 * 		Note that sometimes a nan string value gets in there.
 ***********************************************************************/
bool forceSensorPlugin_impl::readGravCompStatData(const char *filename)
{
	// Local variables
	int index=0;				// Represents right or left index of data
	int ctr=0;					// Local counter
	char command[10];			// Chart to store "sensor 0" or "sensor 1"

	float temp = 0.0;
	string title;				// Keeps name of data to save
	string data;				// String representing double
	int size = 0;
	int size_arr[4]={6,6,6,2};	// Size of 4 data strucs
	int len = 0; 				// Length of a given data struc

	// Create input stream
	ifstream istr_gravCompStatData(filename,ios::in);
	if(!istr_gravCompStatData) return false;

	// Extract data for four pairs: F_ave/M_ave, F_sd/M_sd, F_err_max/M_err_max; F_zero_lim/M_zero_lim
	while(istr_gravCompStatData && !istr_gravCompStatData.eof())
	{

		/* 		Should run the function as follows, however, I could not resolve a system call error originated when calling the funciton below:
		// 		Cannot figure out why the following function call is giving a system call error... there is some problem with the prototype, probably with the multiD vector but not sure */
		//		extract_SingleItem(istr_gravCompStatData, F_ave, M_ave, 6);
		//		if(ret)
		//			ret= extract_SingleItem(istr_gravCompStatData, F_sd[index], M_sd[index], 6);
		//		if(ret)
		//			ret= extract_SingleItem(istr_gravCompStatData, F_err_max[index], M_err_max[index], 6);
		//		if(ret)
		//			ret= extract_SingleItem(istr_gravCompStatData, &F_zero_lim[index], &M_zero_lim[index], 2);

		for(int dataIn=0; dataIn<4; dataIn++)
		{
			if(dataIn==0)
			{
				// Extract Arm Index on first line.
				if(ctr==1)
				{
					istr_gravCompStatData.ignore(1,'\0');
					istr_gravCompStatData.sync();
					istr_gravCompStatData.getline(command,10,'\n');
				}
				istr_gravCompStatData.getline(command,10,'\n');

				// Check for first loop: Left Sensor
				if (!strcmp(command, "sensor 0:"))
					index=0; // Left Arm
				else
					index=1; // Right arm

				// Increase ctr
				ctr++;
			}
			size = size_arr[dataIn];

			// Verify size: expecting 2 or 6.
			if((size > 1) || (size < 7))
			{
				if(size==2)						// Corresponds to size of last data structure
					len=size/2;					// Each side left/right only have one element
				else if(size==6)
					len=size/2;
				else
					return false;
			}

			// Extract average values
			// Average
			istr_gravCompStatData >>  title; // This first extraction is for the title. It is discarded.

			// Now extract actual values.
			for (int i=0; i<size; i++)
			{
				istr_gravCompStatData >>  data;

				// check for -nan
				if(strcmp(data.c_str(),"nan")==0 || strcmp(data.c_str(),"-nan")==0)
					data="0\0"; // Set the value to zero and end with null

				if(dataIn==0)
				{
					if(i<len)
					{
						temp = strtod(data.c_str(),NULL);
						F_ave[index][i]=temp;
					}
					else
					{
						temp = strtod(data.c_str(),NULL);
						M_ave[index][i-len]=temp;
					}
				}
				else if(dataIn==1)
				{
					if(i<len)
					{
						temp = strtod(data.c_str(),NULL);
						F_sd[index][i]=temp;
					}
					else
					{
						temp = strtod(data.c_str(),NULL);
						M_sd[index][i-len]=temp;
					}
				}
				else if(dataIn==2)
				{
					if(i<len)
					{
						temp = strtod(data.c_str(),NULL);
						F_err_max[index][i]=temp;
					}
					else
					{
						temp = strtod(data.c_str(),NULL);
						M_err_max[index][i-len]=temp;
					}
				}
				// This last one works different than the previous 3. It is not a vector, it's an array of size 2.
				// 1st element is for sensor 0, 2nd element is for sensor 1.
				else if(dataIn==3)
				{
					// Copy to elment [0] in both structures
					if(index==0)
					{
						if(i==0)
						{
							temp = strtod(data.c_str(),NULL);
							F_zero_lim[index]=temp;
						}
						else
						{
							temp = strtod(data.c_str(),NULL);
							M_zero_lim[index]=temp;
						}
					}
					else // index==1
					{
						if(i==0)
						{
							temp = strtod(data.c_str(),NULL);
							F_zero_lim[index]=temp;
						}
						else
						{
							temp = strtod(data.c_str(),NULL);
							M_zero_lim[index]=temp;
						}
					}
				}
				else
					return 0; // failure

			}
		}
	}

	// Close file
	//if(ret)
		return 1; // All params loaded successfully
	//else
	//	return 0;
}
//----------------------------------------------------------------------------------------------
// extract_SingleItem
// Extracts data from the file used to compute gravity compensation parameters.
// Assumes 4 pairs of structures. The first 3 of size 3 for each memeber of the pairs, and the
// last one of size 1 for each member of the pairs.
//----------------------------------------------------------------------------------------------
bool extract_SingleItem(ifstream& in, Vector3* first, Vector3* second, int size)
{
	// Local variables
	float temp = 0.0;
	string title;
	string data;
	int len = 0; // to fill in array of incoming pointer

	// Verify size: expecting 2 or 6.
	if((size > 1) || (size < 7))
	{
		if(size==2)
			len=1;
		else if(size==6)
			len=3;
		else
			return false;
	}

	// Extract average values
	// Average
	in >>  title; // This first extraction is for the title. It is discarded.

	// Now extract actual values.
	for (int i=0; i<size; i++)
	{
		in >>  data;

		// check for -nan
		if(strcmp(data.c_str(),"nan")==0 || strcmp(data.c_str(),"-nan")==0)
			data="0\0"; // Set the value to zero and end with null

		if(i<len)
		{
			temp = strtod(data.c_str(),NULL);
			first[0][i]=temp;
		}
		else
		{
			temp = strtod(data.c_str(),NULL);
			second[0][i-len]=temp;
		}
	}

	return true;
}

bool extract_SingleItem(ifstream& in, double* first, double* second, int size)
{
  // Local variables
  string title;
  string data;
  int len = 0; // to fill in array of incoming pointer

  // Verify size: expecting 2 or 6.
  if((size > 1) || (size < 7))
    {
      if(size==2)
	len=1;
      else if(size==6)
	len=3;
      else
	return false;
    }

  // Extract average values
  // Average
  in >>  title; // This first extraction is for the title. It is discarded.

  // Now extract actual values.
  for (int i=0; i<size; i++)
    {
      in >>  data;

      // check for -nan
      if(strcmp(data.c_str(),"nan")==0 || strcmp(data.c_str(),"-nan")==0)
	data="0\0"; // Set the value to zero and end with null

      if(i<len)
	first[i]=strtod(data.c_str(),NULL);
      else
	second[i-len]=strtod(data.c_str(),NULL);
    }

   return true;
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
