/** This class is called by forceSensorPlugin_impl
 * @file   hiroArm.cpp
 * @brief  arm class for HIRO
 * @author Natsuki Yamanobe
 * @date   2011/02/16
 * @note
 */
//----------------------------------------------------------------------------------------------------------------------------------------------------
// Includes
//----------------------------------------------------------------------------------------------------------------------------------------------------
#include "hiroArm.h"
//----------------------------------------------------------------------------------------------------------------------------------------------------
// Namespaces
//----------------------------------------------------------------------------------------------------------------------------------------------------
using OpenHRP::dmatrix;
using std::pow;
using std::sqrt;
using std::exp;
using std::ceil;
//----------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINITIONS
//----------------------------------------------------------------------------------------------------------------------------------------------------
// Directories
//----------------------------------------------------------------------------------------------------------------------------------------------------
// Main Directories
#define READ_DIR				"./data/PivotApproach"
#define WRITE_DIR 				"./data/Results"

// To Read Data
#define SL_APPROACH_FILE 		"/PA10/pivotApproachState1.dat"			// Waypoints for State1 in StraightLineApproach for the PA10 Robot
#define PIVOT_APPROACH_FILE 	"/PA10/PA10_pivotApproachState1.dat"	// Waypoints for State1 in PivotApproach for the PA10 Robot
#define SIDE_APPROACH_FILE 		"/HIRO/sideApproachState1.dat"			// Waypoints for State1 in SideApproach for the HIRO Robot
#define FAILURE_CHARAC_FILE 	"/FC/failureCaseXDir.dat"				// Waypoints for State1 in FailureCase. Three files: failureCaseXDir.dat, failureCaseYDir.dat, and failureCaseRollDir.dat
//----------------------------------------------------------------------------------------------------------------------------------------------------
// To Write Data (used in AssemblyStrategy::OpenFiles)
#define ANGLES_FILE				"/Angles.dat"							// Save joint angles of robot
#define CARTPOS_FILE			"/CartPos.dat"							// Save CartPos of End-Effector in world coordinates
#define STATE_FILE				"/State.dat"							// Save State Transition times for SideApproach
#define FORCES_FILE				"/Torques.dat"							// Save Joint Torques for robot
#define MANIP_TEST_FILE			"/manipulationTestAxis.dat"				// Used to test force and moment controllers behavior
//----------------------------------------------------------------------------------------------------------------------------------------------------
// DEBUGGING
//----------------------------------------------------------------------------------------------------------------------------------------------------
#define DB_TIME 				0			// Used to print timing duration of functions
//----------------------------------------------------------------------------------------------------------------------------------------------------
// TEST MODE
//----------------------------------------------------------------------------------------------------------------------------------------------------
#define TEST    				0			// If true, a test is activated to see the response of the force/moment controllers. Reads an int from file to known which axis to activate.
//----------------------------------------------------------------------------------------------------------------------------------------------------
// CONTROL METHODS
//----------------------------------------------------------------------------------------------------------------------------------------------------
#define USE_MOTION_DAT 			0			// Should be zero if used with control basis approach or reading a trajectory from file
#define CONTROL_BASIS			1			// If using the control basis approach
//----------------------------------------------------------------------------------------------------------------------------------------------------
// ASSEMBLY STRATEGY TYPES
//----------------------------------------------------------------------------------------------------------------------------------------------------
#define STRAIGHT_LINE_FLAG 		0 			// Masking Flags to determine which trajectory to choose
#define PIVOT_APPROACH_FLAG		0			// Pivot approach. 								If true, STRAIGHT_LINE_FLAG=0,SIDE_APPROACH_FLAG=0, FAILURE_CHARAC_FLAG=0.
#define SIDE_APPROACH_FLAG		1			// Similar to PivotApproach but no alignment. 	If true, STRAIGHT_LINE_FLAG=0,PIVOT_APPROACH_FLAG=0,FAILURE_CHARAC_FLAG=0.
#define FAILURE_CHARAC_FLAG		0			// Uses params from SideApproach. 				If true, STRAIGHT_LINE_FLAG=0,PIVOT_APPROACH_FLAG=0,SIDE_APPROACH_FLAG=0.

//----------------------------------------------------------------------------------------------------------------------------------------------------
#define STRAIGHT_LINE_APPROACH  1 			// Numbers assigned in correlation to the enumeration CtrlStrategy in AssemblyStrategy.h
#define PIVOT_APPROACH 			2
#define SIDE_APPROACH			3
#define FAILURE_CHARAC 			4
//----------------------------------------------------------------------------------------------------------------------------------------------------
// ASSIGN VARIABLES TO BE USED WITH PA->INITIALIZE(...)
//----------------------------------------------------------------------------------------------------------------------------------------------------
#if(USE_MOTION_DAT==0) 						//I.e. We are using the control basis approach, then choose between strategies. File name assigned during constructor to private member.

	// (A) Define the Control Strategy
	#define CONTROL_TYPE CONTROL_BASIS

	// (B) Define the Assembly Strategy
	#if(STRAIGHT_LINE_FLAG)
		#define MOTION_FILE 	SL_APPROACH_FILE
		#define APPROACH_TYPE 	STRAIGHT_LINE_APPROACH
	#elif(PIVOT_APPROACH_FLAG)
		#define MOTION_FILE 	PIVOT_APPROACH_FILE
		#define APPROACH_TYPE 	PIVOT_APPROACH
	#elif(SIDE_APPROACH_FLAG)
		#define MOTION_FILE 	SIDE_APPROACH_FILE
		#define APPROACH_TYPE 	SIDE_APPROACH
	#elif(FAILURE_CHARAC_FLAG)
		#define MOTION_FILE 	FAILURE_CHARAC_FILE
		#define APPROACH_TYPE 	FAILURE_CHARAC
	#endif
#else
	#define CONTROL_TYPE USE_MOTION_DAT
#endif
//----------------------------------------------------------------------------------------------------------------------------------------------------
// Constructor Definition : Variable Initialisation List
//----------------------------------------------------------------------------------------------------------------------------------------------------
hiroArm::hiroArm(std::string name_in, BodyPtr body_in, unsigned int num_q0_in, double period_in, float ang_limits_in[6][5], vector3 ePh_in, matrix33 eRh_in, vector3 hPfs_in)
  :f_moving(false), 	f_reached(false),		initFlag(false),	m_time(0),
   f_gc(false),     	f_gc_init(false),   	step_gc(0),			fp1(NULL),
   mass(0.0),       	GACC(9.8),          	wait_step(200),		fp2(NULL),
   /*fs(0),*/           step_fs(0),        	 	max_step_fs(500),	fp3(NULL),
   MAX_JVEL(0.5),   	TIME_LIMIT(100.0), 		MIN_PERIOD(1.0e-9), TOLERANCE(1.0e-6)
{
#ifdef DEBUG_PLUGIN2
  std::cerr << "hiroArm(): Entering hiroArm constructor" << std::endl;
#endif

  // Save to local members
  name 		= name_in;										// Right or left arm
  body 		= body_in;										// Body object extracted from ModelLoader
  NUM_q0 	= num_q0_in;									// Which joint is the starting joint. Right arm=3, Left arm=9
  DEL_T 	= period_in;									// Time step set at 0.0005 secs in calling function
  for(int i = 0; i < ARM_DOF; i++){
	  for(int j = 0; j < 5; j++){
		  ang_limits[i][j] = ang_limits_in[i][j];			// Set angle limits
	  }
  }

  // Arms' Current state
  ePh 	= ePh_in;											// Wrist 2 End Effector Left: <0.0715, 0.0, 0.0>, Right: <-0.12,0.0,0.0>
  eRh 	= eRh_in;											// Equivalent Rotation matrix. Set to the identity matrix.
  hPfs 	= hPfs_in;											//

#ifdef DEBUG_PLUGIN2
  std::cerr << "hiroArm(): Setting body links." << std::endl;
#endif

  // Set Body linksOB
  //Link* S0 = body->link(CHEST_JOINT);						// SO link starts at the chest joint. It's the first link, then two for the head, then six for the right arm, then six for the left arm
  Link* S0 = body->joint(CHEST_JOINT0);

  //Link* W6 = body->joint(NUM_q0 + ARM_DOF - 1);			// The wrist occurs six links later (i.e.start+6-1).
  Link* W6;
  if(NUM_q0 == RARM_JOINT0)
	  W6 = body->joint(RARM_JOINT5); // Is it joint or link?
  else
	  W6 = body->joint(LARM_JOINT5); // Is it joint or link?
	
  if(!S0)
    std::cerr << "S0 is null\n";

  if(!W6)
    std::cerr << "W6 is null\n";
#ifdef DEBUG_PLUGIN2
  std::cerr << "hiroArm(): Getting the joint path." << std::endl;
#endif

  m_path = body->getJointPath(S0,W6);				// Base to Wrist Path

#ifdef DEBUG_PLUGIN2
  cout << "Joint path:" << m_path->numJoints() << std::endl;
  cerr << m_path->joint(0)->name << std::endl;
  cerr << m_path->joint(1)->name << std::endl;
  cerr << m_path->joint(2)->name << std::endl;
  cerr << m_path->joint(3)->name << std::endl;
  cerr << m_path->joint(4)->name << std::endl;
  cerr << m_path->joint(5)->name << std::endl;
#endif


#if 0
  m_path->setMaxIKIter(200);					// Maximum limit of iterations to numerically compute the inverse kinematics
#endif
  // Inverse Kinematics gains for
  ikGains[0] 	= 0.3;
  ikGains[1] 	= 0.6;
  ikGains[2] 	= 0.7;
  ikGains[3] 	= 0.8;

  ikRot[0] 	= 1e-2;
  ikRot[1] 	= 1e-7;
  ikRot[2] 	= 1e-8;
  ikRot[3] 	= 1e-10;

  ikTrans[0] 	= 1e-2;
  ikTrans[1] 	= 1e-7;
  ikTrans[2] 	= 1e-8;
  ikTrans[3] 	= 1e-10;

  // Program Design Parameters
  p_param.method 	= CONSTANT_VELOCITY;				// Interpolation Method: Velocity
  p_param.max_jvel 	= MAX_JVEL;							// With MAX_JVEL as maximum joint velocity limit = 0.5
  controlmode 		= NOCONTROL;						// One of six control modes. No control.

  //******************************************** Pivot Approach ***********************************************************************/
  // Takes the following parameters int NUM_q0, vector3 base2endEffectorPos, matrix33 base2endEffectorRot, dmatrix jacobian, double curr_time, vector6 curr_force
  vector3 pos(0);
  matrix33 rot;//x(0);
  double momentGainFactor = 10.0;	// Used to scale the gain for the control basis for the moment.

#ifdef DEBUG_PLUGIN2
  std::cerr << "hiroArm(): Getting current position and attitude values" << std::endl;
#endif


#ifdef DEBUG_PLUGIN2
  std::cerr << "hiroArm(): Retrieving the position" << std::endl;
#endif
  // HOMING Position and rotation matrix for right arm's WRSIT
  // Right arm's last joint is joint 8 (index starts at 0, chest pan tilt...)
  pos=m_path->joint(5)->p; // instead of RARM_JOINT5 i had 5 before.
#ifdef DEBUG_PLUGIN2
  std::cerr << "hiroArm(): Position retrieved" << std::endl;
#endif

#ifdef DEBUG_PLUGIN2
  std::cerr << "hiroArm(): Retrieving the attitude." << std::endl;
#endif
  //rot=m_path->joint(5)->attitude(); 		// used in linux simulation
  rot=m_path->joint(5)->segmentAttitude();    // Instead of RARM_JOINT5 I had a 5 before.
#ifdef DEBUG_PLUGIN2
  std::cerr << "hiroArm(): Attitude retrieved." << std::endl;
#endif


#ifdef DEBUG_PLUGIN2
  std::cerr << "hiroArm(): Allocating AssemblyStrategy" << std::endl;
#endif

  // Pivot Approach variables
  PA = new AssemblyStrategy(NUM_q0,pos,rot,ePh,momentGainFactor);	// Wrist position, Wrist Rotation Matrix, Wrist2EndEffXform

#ifdef DEBUG_PLUGIN2
  std::cerr << "hiroArm(): Finished allocating AssemblyStrategy" << std::endl;
#endif

	
#ifdef DEBUG_PLUGIN2
  std::cout << "hiroArm: exit constructor"  << std::endl;
#endif
}

// Destructor
hiroArm::~hiroArm()
{
  fclose(fp1);
  fclose(fp2);
  fclose(fp3);
}

/*************************************************************************************************************/
// Init()
/*************************************************************************************************************/
void hiroArm::init()
{
  update_currposdata();
  rPh_ref = rPh;
  rRh_ref = rRh;
  q_ref = q;
}

/*************************************************************************************************************/
// init(vector3 pos, matrix33 rot, double CurAngles[15])
// Initialization Routine for a selected arm. Compute the position vector and rotation matrix from base to end effector
// Different behavior according to user.
// Kensuke: Open position and force data for left and right arms.
// Yamanobe: reads force, position, and velocity information.
/*************************************************************************************************************/
int hiroArm::init(vector3 pos, matrix33 rot, double CurAngles[15], double* deviation)
{
  // Local variables
#ifdef DEBUG_PLUGIN2
  std::cerr << "\nhiroArm::init - entered" << std::endl;
#endif 
  int ret = 0;

#ifdef PIVOTAPPROACH
  //******************************************** Pivot Approach ***********************************************************************/

  /*************************************** Initialize Strategy ***************************************/
  // PA can use three optional user specified files to read or write information (designed for one arm):
  // Read: Trajectory File - Reads Trajectory.
  // Write: State File - writes times at which new states start.
  //        Forces File - writes forces and moments experienced by the simulation
  // If no arguments are passed, the files can be found at:
  // /home/hrpuser/forceSensorPlugin_Pivot/data/PivotApproach/

  // Path trunk
  // Folder where robot joint angles, cart. position, and forces will be saved and desired trajectory read from.

  // Assign path trunk to each of the FIVE char variables
  strcpy(TrajState1,	READ_DIR);
  strcpy(TrajState2,	READ_DIR);
  strcpy(manipTest,  	READ_DIR);
  strcpy(Angles,	    WRITE_DIR);
  strcpy(CartPos,		WRITE_DIR);
  strcpy(State,	    	WRITE_DIR);
  strcpy(Forces,	    WRITE_DIR);

  // Concatenate with appropriate endings
  strcat(TrajState1,	MOTION_FILE);							// Desired Trajectory
  strcat(TrajState2,	"/PA10/PA10_pivotApproachState2.dat");	// Desired Trajectory
  strcat(manipTest,		MANIP_TEST_FILE);						// What test axis do you want to try
  strcat(Angles,		ANGLES_FILE);							// Robot Joint Angles
  strcat(CartPos,		CARTPOS_FILE);							// Cartesian Positions
  strcat(State,			STATE_FILE);							// New States Time Occurrence
  strcat(Forces,		FORCES_FILE);							// Robot Forces/Moments

  // Initialize AssemblyStrategy Class:
  // 1) Open files associated with the directories to read/write data
  // 2) Assign homing Cartesian Position, Joint Angles, and Rotation matrix position.
  // 3) Assign Motion and Control Strategies
  // 4) Declare and Allocate Filtering Object (consider using a static object instead for faster real-time performance).
  ret=PA->Initialize(TrajState1,TrajState2,Angles,CartPos,State,Forces, 			// Data Directories
		  	  	  	 pos, rot, CurAngles,											// Homing Data
		  	  	  	 APPROACH_TYPE, CONTROL_TYPE, deviation);									// Assembly Strategy and Control Method Used

#endif

  // Compute the current position vector and rotation matrix from base to end effector and current joint angles.
  update_currposdata();
  rPh_ref = rPh;			// Base2EndEff position translation
  rRh_ref = rRh;			// Base2EndEff rotation matrix
  q_ref = q;				// At home joint angles

#ifdef DEBUG_PLUGIN2
  std::cerr << "\nhiroArm::init - exited" << std::endl;
#endif

  return ret;
}

void hiroArm::savedata()
{
  // Initialization
  vector3 tmp = OpenHRP::rpyFromRot(rRh);						// Get RPY terms for the wrist

  // Hand (not arm) Gravity Compensation terms for force and moment
  fprintf(fp1,"%f %f %f %f %f %f\n",							 rFh_gc[0],	rFh_gc[1],	rFh_gc[2], rMh_gc[0],	rMh_gc[1],rMh_gc[2]);
  fprintf(fp2,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",rPh[0], rPh[1], rPh[2],		// Position translation from base 2 end effector
	  tmp[0], tmp[1], tmp[2],		// RPY terms
	  rRh(0,0),rRh(0,1),rRh(0,2),	// Rotation matrix terms for base 2 end effector
	  rRh(1,0),rRh(1,1),rRh(1,2),
	  rRh(2,0),rRh(2,1),rRh(2,2));
  /*printf(fp1,"%f %f %f %f %f %f\n",rFfs_gc[0],rFfs_gc[1],rFfs_gc[2],rMfs_gc[0],rMfs_gc[1],rMfs_gc[2]); arm terms
	vector3 tmp = OpenHRP::rpyFromRot(rRh);
	fprintf(fp2,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",rPh[0],rPh[1],rPh[2],tmp[0],tmp[1],tmp[2], rRh(0,0),rRh(0,1),rRh(0,2), rRh(1,0),rRh(1,1),rRh(1,2), rRh(2,0),rRh(2,1),rRh(2,2));
  */
}

/*******************************************************************************************************
 // get_name()
 // Return whether right or left arm
 *******************************************************************************************************/
std::string hiroArm::get_name() { return name; }

/********************************************************************************
 * update_currposdata()
 * Update current arm configuration by:
 * First, retreving:
 * 	The wrist position,
 * 	The base2wrist rotation, and
 * 	The current joint angles of the body
 * Second, by computing the:
 * 	base2EndEffectorPosition
 *********************************************************************************/
void hiroArm::update_currposdata()
{
	// base2wrist position. Homing pos: <0.3019, -0.1724, 0.064>
	rPe = body->joint(NUM_q0 + ARM_DOF -1)->p;
#ifdef DEBUG_PLUGIN2
	cerr << "hiroArm::updated_currposdata - rPe is: " << rPe(0) << " " << rPe(1) << " " << rPe(2) << std::endl;
#endif
	// base2wrist rotation matrix
	rRe = body->joint(NUM_q0 + ARM_DOF -1)->segmentAttitude();

	// Current joint angles
	for(int i=0; i<ARM_DOF; i++)
		q[i] = body->joint(NUM_q0 + i)->q;

	wrist2EndEffXform(/*in*/rPe, /*in*/rRe, /*out*/rPh, /*out*/rRh);
#ifdef DEBUG_PLUGIN2
	cerr << "hiroArm::updated_currposdata - rPh is: " << rPh(0) << " " << rPh(1) << " " << rPh(2) << std::endl;
#endif
}

/********************************************************************************
 * wrist2EndEffXform()
 * Takes the base2wrist position and rotation and transforms them to base2endeff
 * position and orientation
 *********************************************************************************/
void hiroArm::wrist2EndEffXform(/*in*/vector3 rPe, /*in*/matrix33 rRe, /*out*/vector3& rPh, /*out*/matrix33& rRh)
{
  // base2EndEffector
  // Caculation rPh & rRh from rPe & rRe
  rRh = rRe * eRh;						// base2EndEffRot=base2wristRot*wrist2EndEffRot
  rPh = rPe + rRe * ePh;					// base2EndEffPos=base2wristPos + transformed wrist2EndEffPos
}

/********************************************************************************
 * wrist2EndEffXform()
 * Takes the base2wrist position and rotation and transforms them to base2endeff
 * position and orientation
 *********************************************************************************/
void hiroArm::EndEff2wristXform(/*in*/vector3 rPh, /*in*/matrix33 rRh, /*out*/vector3& rPe)
{
  // base2EndEffector
  // Caculation rPh & rRh from rPe & rRe
  rPe = rPh - rRe * ePh;					// base2EndEffPos=base2wristPos + transformed wrist2EndEffPos
}

/*******************************************************************************************************
 // get_qref()
 // Retrieve reference joint angle in degrees
 *******************************************************************************************************/
dvector6 hiroArm::get_qref()
{
#ifdef DEBUG_PLUGIN2
  std::cout << "q_ref in hiroArm::get_qref: " << q_ref*180/M_PI << std::endl;
#endif

  return q_ref;
}

/*******************************************************************************************************
 // get_qcur()
 // Retrieve currentjoint angle in degrees
 *******************************************************************************************************/
dvector6 hiroArm::get_qcur()
{
#ifdef DEBUG_PLUGIN2
  std::cout << "q_ref in get_qref: " << q_ref*180/M_PI << std::endl;
#endif

  return q;
}

/*******************************************************************************************************
 // update_currforcedata()
 // Return latest force and moment data.
 // If gravitational compensation is consider, it uses that information to compute the latest values
 *******************************************************************************************************/
void hiroArm::update_currforcedata()
{
	// update current force-moment data
	//	if(fs != 0)
	//	{
	double f_out[6];

	// A) Read raw forces
	//get_raw_forces(f_out); // Updated Aug 2012 for OldHiro
	ifs_read_data(0,f_out);

	// B) Separate them into force and moment
	for(int i=0; i<3; i++)
	{
		fsF_raw[i] = f_out[i];
		fsM_raw[i] = f_out[i+3];
	}

	// If gravitational component flag is true
	if(f_gc)
	{
		// Calculate the hand's gravitational force and moment
		calc_gc_forces(rFfs_gc, rMfs_gc);			// Arm's force and moment
		calc_gc_forces_at_hand(rFh_gc, rMh_gc);		// Hand's force and moment
	}
	//	}
}

#if 0
/******************************************************************************************************/
// set_FSptr()
// Set the Force Sensor pointer from the niitaFs class. Give an identifier for left and right arms.
// Left arm has id 0.
/******************************************************************************************************/
void hiroArm::set_FSptr(nittaFS *fs_in, int NO_fs_in)
{
  fs = fs_in;
  NO_fs = NO_fs_in;
}
#endif

/*******************************************************************************************************
 // get_forces()
 // Retrieve the HAND's gravity compensation terms for force and moment
 *******************************************************************************************************/
bool hiroArm::get_forces(vector3 &rFh_gc_out, vector3 &rMh_gc_out)
{
  if(f_gc)
    {
      rFh_gc_out = rFh_gc;	// force
      rMh_gc_out = rMh_gc;	// moment
      return true;
    }
  else return false;
}

/*******************************************************************************************************
 // get_curr_handpos()
 // Retrieve the HAND's position translation vector and rotation matrix from base to end-effecter
 *******************************************************************************************************/
void hiroArm::get_curr_handpos(vector3 &rPh_out, matrix33 &rRh_out){

  rPh_out = rPh; 				// base2endeffecter translation vector
  rRh_out = rRh;				// base2endeffecter rotation matrix
}

/*******************************************************************************************************
 // set_ref_handpos()
 // Write the HAND's reference position translation vector and rotation matrix from base to end-effecter
 *******************************************************************************************************/
void hiroArm::set_ref_handpos(vector3 rPh_ref_in, matrix33 rRh_ref_in)
{
  rPh_ref = rPh_ref_in;			// position
  rRh_ref = rRh_ref_in;			// rotation
}

/*******************************************************************************************************
 // get_ref_handpos()
 // Retrieve the HAND's reference position translation vector and rotation matrix from base to end-effecter
 *******************************************************************************************************/
void hiroArm::get_ref_handpos(vector3 &rPh_ref_out, matrix33 &rRh_ref_out)
{
  rPh_ref_out = rPh_ref;
  rRh_ref_out = rRh_ref;
}

/*******************************************************************************************************
 // get_Iteration()
 // Retrieve the current iteration of the simulation
 *******************************************************************************************************/
unsigned long hiroArm::get_Iteration()
{
  return m_time;
}

/*******************************************************************************************************
 // set_Iteration()
 // Increase iteration by one
 *******************************************************************************************************/
void hiroArm::set_Iteration()
{
  m_time++;
}

/*******************************************************************************************************
 // velocity_control()
 // Used in Bilateral control.
 // Takes in rdP, rate of change in position; rW, rate of change in orientation.
 *******************************************************************************************************/
bool hiroArm::velocity_control(vector3 rdP, vector3 rW, bool f_new)
{
  // If NOT vel control or NOT new, assign base2endeffector pos/rot, reference angles, and set to vel control
  if((controlmode != VEL_CONTROL) || f_new)
    {
      rPh_ref = rPh;
      rRh_ref = rRh;
      q_ref = q;
      controlmode = VEL_CONTROL;
    }

  // 1) Calculate the next step joint-angle using the Jacobian.
  dvector6 q_next, dq;
  dvector6 dx; 															// velocity at the arm end frame

  // Change the input velocity from endeffector to wrist
  vector3 rdPe, rWe;													// base-to-wrist
  rWe = rW;

  rdPe = rdP + cross(rW, (rRe*(-ePh)));
  for(int i=0; i<3; ++i)
    {
      dx(i)	=rdPe(i);													// Lin Velocity at wrist
      dx(i+3)	=rWe(i);												// Angular Velocity at Wrist
      //~ dx(i)=rdP(i);
      //~ dx(i+3)=rW(i);
    }

  //~ fprintf(fp3,"%f %f %f %f %f %f %f %f %f %f %f %f %f %d %d %d:: %f %f %f %f %f %f:: %f %f %f %f %f %f\n",rdP(0),rdP(1),rdP(2),rW(0),rW(1),rW(2), dx(0),dx(1),dx(2),dx(3),dx(4),dx(5), suprate,jlimit,lim_tmp,mability,q(0),q(1),q(2),q(3),q(4),q(5),q_ref(0),q_ref(1),q_ref(2),q_ref(3),q_ref(4),q_ref(5));

  // 2) Check angular velocity limit
  double suprate 	= 1;                   								// Suppression rate. Change the value of dx for each iteration of computation
  int supflag    	= 0;												// Flag: Joint angles surppased
  int supexit		= 1;												// While loop flag
  dmatrix jacob(6,6), inv_jacob(6,6);

  // Compute the Jacobian and the Pseudoinverse
  jacob 	 = m_path->Jacobian(); 										//inv_jacob = OpenHRP::inverse(jacob);
  bool ret   = OpenHRP::calcPseudoInverse(jacob, inv_jacob,1.0e-18);

  // Check if correct
  if(ret !=0)
    {
      std::cout << "ERROR(velocity_control): calcPseudoInverse ret = " << ret << std::endl;
      return false;
    }

  // 3) Calculate the next step joint-angle
  while(supexit)
    {
      // Joint-Angle Step = inv_Jacobian*Velocity Step
      //~ dq = prod(OpenHRP::inverse(m_path->Jacobian()),dx);
      dq = prod(inv_jacob, dx);

      // Check angle limits for each axis
      for(int i=0; i<ARM_DOF; i++)
	{
	  // If limits surpased
	  if(fabs(dq(i)) > ang_limits[i][2])
	    {
	      //~ if(fabs(dq(i)) > MAX_JVEL){
	      supflag=1;
#ifdef DEBUG_PLUGIN2
	      std::cout << "dq = " 		<< dq << std::endl;
	      std::cout << "suprate = " 	<< suprate << std::endl;
#endif
	    }
	  // if(fabs(nextdth(i)) > 5*deg2radC) supflag=1;
	}

      // Update the suppresion value used to update the step velocity parameter
      if(supflag)
	{
	  suprate -= 0.1;
	  supflag = 0;
	}
      // An admissible computation for the next step joint-angle has been achieved
      else
	supexit=0;
      dx = suprate * dx;
    }

  //~ std::cout<< "jacob: " << jacob << std::endl;
  //~ std::cout<< "inv_jacob: " << inv_jacob << std::endl;
  //~ std::cout<< "inv_jacob * jacob: " << prod(inv_jacob,jacob) << std::endl;
  //~ std::cout<< "dq" << dq << std::endl;
  //~ std::cout<< "jacob*dq" << prod(jacob,dq) << std::endl;
  //~ std::cout << std::endl;

  // 4) Update the joint angle reference position by adding the step update to the current position
  q_next = q_ref + DEL_T * dq;


  // 5) Check lower and upper joint limits
  bool flag 	= false;
  bool jlimit = true;

  // For each axis
  for(int i=0; i<ARM_DOF; i++)
    {
      if(q_next(i) < ang_limits[i][0] || q_next(i) > ang_limits[i][1])
	jlimit = false;
    }

  // If limits have been surpassed, then
  if(jlimit)
    {
      // Check the difference limit between q_next and q_cur
      bool jdifflimit = true;

      // If the difference is greater than the limit
      for(int i=0; i<ARM_DOF; i++)
	{
	  if(fabs(q_next(i)-q(i)) > ang_limits[i][2] * DEL_T)
	    jdifflimit = false;
	}


      // If the difference is not greater than the limit:
      // We have a singularity. Perform manipulability to avoid the singularity.
      if(jdifflimit)
	{
	  // Skip the point and set the joint angle to q_next
	  // ublas det(,), trans, prod. jacobian is dmatrix type(ublas)
	  for(int i = 0; i < ARM_DOF; i++)
	    body->joint(NUM_q0 + i)->q = q_next(i);

	  // Then compute the forward kinmeatics
	  body->calcForwardKinematics();

	  // Compute the determinant of the Jacobian at this new position
	  //bool mability = true;
	  double det_value = OpenHRP::det(m_path->Jacobian());

	  // Recompute Fwd Kinematics for current position
	  for(int i = 0; i < ARM_DOF; i++)
	    body->joint(NUM_q0 + i)->q = q(i);
	  body->calcForwardKinematics();

	  // If the determinant is not zero,
	  if(fabs(det_value) > 0.00001)
	    flag = true;
	}
    }

  // joint limit avoidance	// double LAUpLimit[6]={84,114,-1,249,94,124};	// double LALoLimit[6]={-84,-174,-154,-60,-94,-124};
  //~ fprintf(fp3,"%f %f %f %f %f %f:: %f %d %d %d:: %f %f %f %f %f %f:: %f %f %f %f %f %f\n",rdP(0),rdP(1),rdP(2),rW(0),rW(1),rW(2), suprate,jlimit,lim_tmp,mability,q(0),q(1),q(2),q(3),q(4),q(5),q_ref(0),q_ref(1),q_ref(2),q_ref(3),q_ref(4),q_ref(5));
  fprintf(fp3,"%f %f %f %f %f %f %f %f %f %f %f %f %f %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
	  rdP(0),rdP(1),rdP(2),																			// Position
	  rW(0),rW(1),rW(2), 																				// Orientation
	  dx(0),dx(1),dx(2),																				// Lin Vel
	  dx(3),dx(4),dx(5), 																				// Rotational Vel
	  suprate, jlimit, 																				// Params
	  dq(0),		dq(1),		dq(2),		dq(3),		dq(4),		dq(5),								// Joint Vel
	  q_next(0),	q_next(1),	q_next(2),	q_next(3),	q_next(4),	q_next(5));							// Next Joint position

#ifdef DEBUG_PLUGIN2
  std::cout << "q_ref = " << q_ref << std::endl;
  std::cout << "q = " 	<< q << std::endl;
#endif

  // Set the reference joint position to q_next
  if(flag)
    {
      q_ref = q_next;
      return true;
    }
  else
    return false;
}

/****************************************************************************************************
 // Impedance Control
 // Mapping from generalized velocities to generalized forces
 ** Sets the damping and inertia coefficient values
 ****************************************************************************************************/
bool hiroArm::impedance_control()
// DSRG's direct teaching function
{
  // If control mode is not set to impedance_control mode, correct.
  if(controlmode != IMP_CONTROL)
    {
      rPh_ref = rPh;
      rRh_ref = rRh;
      controlmode = IMP_CONTROL;
    }

  // 1a) Set desired inertia and damping factor(for POSITION)
  const double  Dlh[2]= {100,250};        							// Threshold of damping coef.[low, high]
  const double  vlh[2]= {0.005,0.1};      							// Threshold of velocity.    [low, high]
  double Dxth[3];													// Damping coefficients
  double Mxth[3];            										// Inertia coefficients With values 5,15,20

#if 1
  // double size = sqrt(pow(rdXd(X),2)+pow(rdXd(Y),2)+pow(rdXd(Z),2));
  // Norm of velocity. Measure of velocity.
  double size = sqrt(pow(rdXd(X),2)+pow(rdXd(Y),2)+pow(rdXd(Z),2));


  for(int i=0; i<3; i++)
    {
      // If the velocity is under our velocity threshold: low velocity zone
      if(fabs(size) <= vlh[0])
	{
	  // Set the damping and inertia coefficients
	  Dxth[i]= Dlh[1];
	  Mxth[i]=3;
	}

      // Middle velocity zone
      else if(fabs(size) > vlh[0] && fabs(size)< vlh[1] )
	{
	  Dxth[i]= ((Dlh[0]-Dlh[1])/(vlh[1]-vlh[0]))*(fabs(rdXd(i))-vlh[0])+Dlh[1];
	  Mxth[i]=5;
	}

      // High velocity zone
      else if(fabs(size) >= vlh[1])
	{
	  Dxth[i]= Dlh[0];
	  Mxth[i]=10;
	}

      // Other case
      else
	{
	  Dxth[i]= Dlh[1];
	  Mxth[i]=15;
	}
    }
#endif

  // Place coefficients in matrix form
  Mt = Mxth[X],0,0, 0,Mxth[Y],0, 0,0,Mxth[Z];
  Dt = Dxth[X],0,0, 0,Dxth[Y],0, 0,0,Dxth[Z];

  //1b) Set desired inertia and damping factor(for ROTATION)
  const double  Drlh[2]= {15,40};        	// threshold of damping coef.[low, high]
  const double  vrlh[2]= {0.2,0.7};       // thrheshold of velocity. [low, high]
  double Drth[3];							// Damping coefficients for wrist={10,10,10};
  double Mrth[3];							// Inertia coefficients for wrist={1,1,1};


#if 1
  size = sqrt(pow(rdRd(X),2)+pow(rdRd(Y),2)+pow(rdRd(Z),2));

  for(int i=0; i<3; i++)
    {
      // Low velocity zone
      if(fabs(size) <= vrlh[0])
	{
	  Drth[i]= Drlh[1];
	  Mrth[i]=1;
	}

      // Middle velocity zone
      else if(fabs(size) > vrlh[0] && fabs(size)< vrlh[1] )
	{
	  Drth[i]= ((Drlh[0]-Drlh[1])/(vrlh[1]-vrlh[0]))*(fabs(rdRd(i))-vrlh[0])+Drlh[1];
	  Mrth[i]=3;
	}

      // High velocity zone
      else if(fabs(size) >= vrlh[1])
	{
	  Drth[i]= Dlh[0];
	  Mrth[i]=5;
	}

      // Other
      else
	{
	  Drth[i]= Drlh[1];
	  Mrth[i]=7;
	}
    }

#endif

  // Compile damping and inertia coefficients for rotation into a matrix
  Mr = Mrth[X],0,0, 0,Mrth[Y],0, 0,0,Mrth[Z];
  Dr = Drth[X],0,0, 0,Drth[Y],0, 0,0,Drth[Z];

  /* Mt = 10, 0, 0, 0, 10, 0, 0, 0, 10;
     Dt = 100, 0, 0, 0, 100, 0, 0, 0, 100;
     Mr = 5, 0, 0, 0, 5, 0, 0, 0, 5;
     Dr = 15, 0, 0, 0, 15, 0, 0, 0, 15;*/


  // Compute the velocity updates in the position and orientation
  for(int i=0;i<3; i++)
    {
      rdXd(i) = exp((-Dt(i,i)/Mt(i,i))*DEL_T)*rdXd(i)+((1-exp((-Dt(i,i)/Mt(i,i))*DEL_T))/Dt(i,i))*rFfs_gc(i);
      rdRd(i) = exp((-Dr(i,i)/Mr(i,i))*DEL_T)*rdRd(i)+((1-exp((-Dr(i,i)/Mr(i,i))*DEL_T))/Dr(i,i))*rMfs_gc(i);

      rRd(i) = rdRd(i)*DEL_T;
    }
  //rRd=ASMRo*rRd;
  rPh_ref += rdXd*DEL_T;
  rRh_ref = rRh_ref * get_rot33(Z,rRd(Z))*get_rot33(Y,rRd(Y))*get_rot33(X,rRd(X)); //RzRyRx

#ifdef DEBUG_PLUGIN2
  std::cout << "direct teaching: " << std::endl;
  std::cout << "rdXd = " << rdXd << std::endl;
  std::cout << "rFfs_gc = " << rFfs_gc << std::endl;
  std::cout << "rMfs_gc = " << rMfs_gc << std::endl;
#endif

  // Target hand first position, fortation, and then calculation
  // 目標手先位置・姿勢の計算 (rPh_ref, rRh_ref --> rPe_ref, rRe_ref --> q_ref)
  // robot2endeffector to robot2wrist
  calc_rPe_rRe(rPh_ref, rRh_ref, rPe_ref, rRe_ref);

  bool ret;
  dvector6 q_ref_tmp;

  // Compute reference joint angles
  ret = calc_qref(rPe_ref, rRe_ref, q_ref_tmp);
  if(ret)	q_ref = q_ref_tmp;

#ifdef DEBUG_PLUGIN2
  std::cout << "q_ref = " << q_ref << std::endl;
  std::cout << "q = " 	<< q << std::endl;
#endif

  return ret;
}

#ifdef IMPEDANCE
bool hiroArm::impedance_control2()
{
  if(controlmode != IMP_CONTROL)
    {
      rPh_ref = rPh;					// Set the base2wrist parameters as ref params: position
      rRh_ref = rRh;
      controlmode = IMP_CONTROL;		// Change the control mode to impedance control
    }

  // Inertia and damping matrix coefficients for position/orientation
  Mt = 10, 0, 0, 0, 10, 0, 0, 0, 10;
  Dt = 100, 0, 0, 0, 100, 0, 0, 0, 100;
  Mr = 5, 0, 0, 0, 5, 0, 0, 0, 5;
  Dr = 15, 0, 0, 0, 15, 0, 0, 0, 15;

  matrix33 Kt, Kr;
  Kt = 200, 0, 0, 0, 200, 0, 0, 0, 200;
  Kr = 100, 0, 0, 0, 100, 0, 0, 0, 100;

  matrix33 Ft, Fr;
  Kt = 2, 0, 0, 0, 2, 0, 0, 0, 2;
  Kr = 1, 0, 0, 0, 1, 0, 0, 0, 1;

  // Desired position and rotation data
  vector3 rXd_des, rRd_des, rFd_des, rMd_des;
  if(!fin_rpos.eof()) fin_rpos >> rXd_des(0);	// Right Pos Data is read in
  if(!fin_rpos.eof()) fin_rpos >> rXd_des(1);
  if(!fin_rpos.eof()) fin_rpos >> rXd_des(2);
  if(!fin_rpos.eof()) fin_rpos >> rRd_des(0);	// Right Rot Data is read in
  if(!fin_rpos.eof()) fin_rpos >> rRd_des(1);
  if(!fin_rpos.eof()) fin_rpos >> rRd_des(2);
  for(int i=0; i<9; i++)
    {
      double t;
      fin_rpos >> t;
    }
  // Desired Force and Moment Data
  if(!fin_rfc.eof()) fin_rfc >> rFd_des(0);	// Right Force Data is read in
  if(!fin_rfc.eof()) fin_rfc >> rFd_des(1);
  if(!fin_rfc.eof()) fin_rfc >> rFd_des(2);
  if(!fin_rfc.eof()) fin_rfc >> rMd_des(0);
  if(!fin_rfc.eof()) fin_rfc >> rMd_des(1);
  if(!fin_rfc.eof()) fin_rfc >> rMd_des(2);

  // k*pos_error - t*force_error...
  for(int i=0;i<3; i++)
    {
      rdXd(i) = rdXd(i) - (Dt(i,i)*rdXd(i) + Kt(i,i)*(rPh_ref(i) - rXd_des(i)) - Ft(i,i)*(rFfs_gc(i) - rFd_des(i)))/Mt(i,i)*DEL_T;							//rdXd(i) = exp((-Dt(i,i)/Mt(i,i))*DEL_T)*rdXd(i)+((1-exp((-Dt(i,i)/Mt(i,i))*DEL_T))/Dt(i,i))*rFfs_gc(i);
      rdRd(i) = rdRd(i) - (Dr(i,i)*rdRd(i) + Kr(i,i)*(OpenHRP::rpyFromRot(rRh_ref)(i) - rRd_des(i)) - Fr(i,i)*(rMfs_gc(i) - rMd_des(i)))/Mr(i,i)*DEL_T;		//rdRd(i) = exp((-Dr(i,i)/Mr(i,i))*DEL_T)*rdRd(i)+((1-exp((-Dr(i,i)/Mr(i,i))*DEL_T))/Dr(i,i))*rMfs_gc(i);

      rRd(i) = rdRd(i)*DEL_T;			//rRd=ASMRo*rRd;
    }

  rPh_ref += rdXd*DEL_T;
  rRh_ref = rRh_ref * get_rot33(Z,rRd(Z))*get_rot33(Y,rRd(Y))*get_rot33(X,rRd(X));	// RzRyRx

#ifdef DEBUG_PLUGIN2
  std::cout << "rXd_des = " 			<< rXd_des 	<< std::endl;
  std::cout << "direct teaching: " 	<< 			   std::endl;
  std::cout << "rdXd = " 				<< rdXd 	<< std::endl;
  std::cout << "rXd = " 				<< rPh_ref 	<< std::endl;
  std::cout << "rFfs_gc = " 			<< rFfs_gc 	<< std::endl;
  std::cout << "rMfs_gc = " 			<< rMfs_gc 	<< std::endl;
#endif

  // 目標手先位置・姿勢の計算 (rPh_ref, rRh_ref --> rPe_ref, rRe_ref --> q_ref)
  calc_rPe_rRe(rPh_ref, rRh_ref, rPe_ref, rRe_ref);
  bool ret;
  dvector6 q_ref_tmp;
  ret = calc_qref(rPe_ref, rRe_ref, q_ref_tmp);
  if(ret)	q_ref = q_ref_tmp;

#ifdef DEBUG_PLUGIN2
  std::cout << "q_ref = " << q_ref 	<< std::endl;
  std::cout << "q = " 	<< q 		<< std::endl;
#endif
  return ret;
}
#endif

//------------------------------------------------------------------------------------------
// PivotApproach(...)
// Two modes. 
// (1) A test mode that allows you test the force controller as part of the control 
// basis for a desired axis. That axis is indicated by a file found in data/PivotApproach/manipulationTestAxis.dat
//
// (2) Actual Pivot Approach. 
// 	There are three modes (other than testing) thus far: 
// 		1. StraightLineApproach, originally implemented with the PA10.
// 		2. PivotApproach: implemented by HIRO on a two snap cantilever snap.
// 		3. SideApproach: implemented by HIRO on a four snap cantilever snap.
// Save the updated value of CurrAngles into the private member q_ref for the appropriate HIRO arm.
//------------------------------------------------------------------------------------------
int hiroArm::PivotApproach(double 	    cur_time,			/*in*/
			   vector3      pos,				/*in*/	// end effector position
			   matrix33 	rot,				/*in*/	// end effector rotation
			   dvector6 	currForces,			/*in*/
			   dvector6& 	JointAngleUpdate,	/*out*/
			   dvector6& 	CurrAngles)			/*out*/
{
#ifdef DEBUG_PLUGIN2
  std::cerr << "Entering hiroArm::PivotApproach" << std::endl;
#endif		
		
  // Local variables
  int		ret = 0;
  dmatrix Jac;
  dmatrix PseudoJac;
  ::AssemblyStrategy::TestAxis testAxis;
    
  // Timing    
  //timeval startTime, endTime, sJ, eJ;			// create variables
#if 0
  unsigned long long sJ,eJ,endTick;
  double duration = 0.0;
  double dJ = 0.0;    
  double freq = 0;
  freq = SYSPAGE_ENTRY(qtime)->cycles_per_sec;	// define operational frequency
  std::cerr << "The frequency is: " << freq << std::endl;    

  if(DB_TIME)
    {
      //gettimeofday(&startTime,NULL); 			// Initialize startTime for this function
      //gettimeofday(&sJ,NULL);					// start time for jacobian computations
      sJ = get_tick();	
    }
#endif    
  // Compute Jacobian and PseudoJac
  m_path->calcJacobian(Jac);							// Calculate the Jacobian based on the base-2-wrist path segment we have assigned. This is a JointPathPtr type
  //OpenHRP::calcPseudoInverse(Jac,PseudoJac);		// Not used with HIRO
	
  // Finalize jacobian time computations
#if 0
  if(DB_TIME)
    {
      //gettimeofday(&eJ,NULL);
      //dJ  = (double)(eJ.tv_sec -  sJ.tv_sec)  * 1000.0; 	// Get from sec to msec
      //dJ += (double)(eJ.tv_usec - sJ.tv_usec) / 1000.0; 	// From usec to msec
		
      eJ = get_tick();
      dJ = ((double)(eJ-sJ)/(freq))*1000.0; //ms		
				
      // Print out the duration of the function
      std::cerr << "Duration of Jacobian Computations in hiroArm::PivotApproach() is: " << dJ << "ms." << std::endl;		
    }
#endif	

	// Run test or actual pivot approach strategy
	if(TEST)
	{
		// Run tests individually. Proceed to the next state when the completionFlag is true.

		// Open the file stream for testing
		ifstream mT; int axis = 0;
		mT.open(manipTest);
		if (!mT.is_open())
			std::cerr << "\nThe manipulation test file!!\n" << std::endl;

		// Read index from file to indicate with of the 12 force/moment directions you would like to test
		mT >> axis;
		// Close the file
		mT.close();

		// Axis selection
		if(axis==0)			testAxis = PA->posFx;
		else if(axis==1)		testAxis = PA->negFx;
		else if(axis==2)		testAxis = PA->posFy;
		else if(axis==3)		testAxis = PA->negFy;
		else if(axis==4)		testAxis = PA->posFz;
		else if(axis==5)		testAxis = PA->negFz;
		else if(axis==6)		testAxis = PA->posMx;
		else if(axis==7)		testAxis = PA->negMx;
		else if(axis==8)		testAxis = PA->posMy;
		else if(axis==9)		testAxis = PA->negMy;
		else if(axis==10)		testAxis = PA->posMz;
		else if(axis==11)		testAxis = PA->negMz;
		else // don't do anything
			return 1;

		// Call the state machine with a test ctrl strategy.
		PA->StateMachine(testAxis,PA->ManipulationTest,m_path,body,cur_time,pos,rot,currForces,JointAngleUpdate,CurrAngles,Jac,PseudoJac);
	}
	
	else
	{
		// Invoke the state machine to run the pivot approach using the control basis
		ret = PA->StateMachine( PA->none,										// Use all axis to run test. Not applicable here since the flag test is off.
								(AssemblyStrategy::CtrlStrategy)APPROACH_TYPE,	// Type of approach. Can choose between the straight line approach and the pivot approach
								m_path,											// Pointer containing Arm Path (joints and links)
								body,											// Pointer containting whole body (joints and links)
								cur_time,										// Current internal clock time (not synced to GrxUI Simulation clock)
								pos,											// Current wrist position in Cartesian Coordinates
								rot,											// Rotation state of wrist
								currForces,										// Current Torques and Moments at the wrist
								JointAngleUpdate,								// Joint Angle Update produced by control basis
								CurrAngles,										// Current Arm Joint Angles
								Jac,											// Jacobian
								PseudoJac);										// JPseudo Jacobian
	}

	// Copy CurrAngles to qref if kinematics did not fail
	if(ret!=-1)
		for(int i=0;i<6;i++)
			q_ref(i) = CurrAngles(i);
	else
		return ret;
	
	#ifdef DEBUG_PLUGIN2
		std::cerr << "Ending hiroArm::PivotApproach" << std::endl;
	#endif
		
#if 0
		if(DB_TIME)
		{
			// Timing
			// Get end time
			//gettimeofday(&endTime,NULL);

			// Compute duration
			//duration  = (double)(endTime.tv_sec - startTime.tv_sec)   * 1000.0; 	// Get from sec to msec
			//duration += (double)(endTime.tv_usec - startTime.tv_usec) / 1000.0; 	// From usec to msec


			endTick = get_tick();
			duration = ((double)(endTick-sJ)/(freq))*1000.0; //ms

			// Print out the duration of the function
			std::cerr << "Duration of hiroArm::PivotApproach() is: " << duration << "ms." << std::endl;
		}
#endif

	return ret;
}

// Copy the latest position and rpy data into the original EndEff position and rpy
void hiroArm::set_OrgPosRot(vector3& pos, vector3& RPY)
{

  // Local variable
  matrix33 rot;

  // Read the latest current end-effector position
  update_currposdata(); // stores hand pos in rPh and rRh

  pos = rPh;
  rot = rRh;

  cerr << "KH1 " << pos << std::endl;

  RPY = rpyFromRot(rRh);

  // Copy these values to PA's EndEff_p_org
  PA->EndEff_p_org = rPh;
  PA->EndEff_r_org = RPY;

  // Set the wrist values as well
  // For the first iteration they are both the same.                                     
  PA->wrist_p = PA->EndEff_p_org;
  PA->wrist_r = PA->EndEff_r_org;  

}

/**************************************************************************/
// moveto_q_goal
// moves hiro to the goal position and returns 0, or else return an error -1.
// If the robot is at positino, returns 1.
/**************************************************************************/
int hiroArm::moveto_q_goal(dvector6 q_goal_in)
{
  // Initialize variables
  int ret_err  = -1;
  int ret_next = 0;
  int ret_fin  = 1;

  // While we are not at the goal
  if(!f_reached)
    {
      // If we successfully moved to the goal position
      if(moveto(q, q_goal_in))
	return ret_next;
      else
	return ret_err;
    }
  else
    {
      f_reached = false;
      return ret_fin;
    }
}

/*bool hiroArm::position_control(dvector6 vref){return true;}*/

/**************************************************************************/
// reset_gravity_comp
/**************************************************************************/
void hiroArm::reset_gravity_comp()
{
  f_gc 		= false;
  f_gc_init 	= false;
  step_gc 	= 0;
  mass 		= 0.0;
  fsF_offset 	= 0.0, 0.0, 0.0;
  fsM_offset 	= 0.0, 0.0, 0.0;
  rG_vec 		= 0.0, 0.0, 0.0;
  fsPgc 		= 0.0, 0.0, 0.0;
}

/**************************************************************************/
// reset_gravity_comp
// return 1: gravity compensation is finished
// return 0: continued
// return minus values: error ocurrs
/**************************************************************************/
int hiroArm::gravity_comp()
{
  int ret_err = -1;
  int ret_next = 0;
  int ret_fin = 1;

#ifdef DEBUG_PLUGIN2
  std::cout << "step_gc = " << step_gc << std::endl;
#endif

  // If we have a good force pointer, continue
  //	if(fs != 0)
  //{

  // If gravity compensation has not yet been executed
  if(step_gc == 0)
    {
      // If we have not innitialized our variable
      if(!f_gc_init)
	{
	  // save initial hand pos & ori
	  //bool ret;
	  dvector6 q_ref_tmp;
	  vector3  rPe_tmp;
	  matrix33 rRe_tmp;

	  rPh_initial = rPe;
	  rRh_initial = rRe;

	  // 目標手先位置・姿勢の計算 (rPh_ref, rRh_ref --> rPe_ref, rRe_ref --> q_ref)
	  //~ calc_rPe_rRe(rPh_initial, rRh_initial, rPe_tmp, rRe_tmp);
	  //~ #ifdef DEBUG_PLUGIN2
	  //~ std::cout << "rPe: " << rPe << std::endl;
	  //~ std::cout << "rRe: " << rRe << std::endl;
	  //~ std::cout << "rPh: " << rPh_initial << std::endl;
	  //~ std::cout << "rRh: " << rRh_initial << std::endl;
	  //~ #endif
	  //~ ret = calc_qref(rPe_tmp, rRe_tmp, q_ref_tmp);
	  // Compute the reference joint angles you desire
	  if(calc_qref(rPh_initial, rRh_initial, q_ref_tmp))
	    {
	      //~ if(ret){
	      q_gc_ref[0] = q_ref_tmp;
	      fsRr_gc[0]  = tvmet::trans(rRh_initial * calc_hRfs(q_ref_tmp));

#ifdef DEBUG_PLUGIN2
	      std::cout << "hRfs" << calc_hRfs(q) << std::endl;
	      std::cout << "fsRr_gc" << fsRr_gc[0] << std::endl;
#endif
	    }
	  else
	    {
	      std::cerr << "ERROR(gravity_comp): calc_q_ref() " << std::endl;
	      return ret_err;
	    }

	  // Compute measuring points (joint angle)
	  matrix33 hRh_ref[4];
	  matrix33 rRh_ref_tmp;
	  double ang = (M_PI/6.0);

	  // case 0:
	  hRh_ref[0] = get_rot33(Z,-ang);
	  // case 1:
	  hRh_ref[1] = get_rot33(Z, ang);
	  // case 2:
	  hRh_ref[2] = get_rot33(Y,-ang);
	  // case 3:
	  hRh_ref[3] = get_rot33(Y, ang);

	  for(int i=0; i<4; i++)
	    {
	      rRh_ref_tmp = rRh_initial * hRh_ref[i];
	      //~ calc_rPe_rRe(rPh_initial, rRh_ref_tmp, rPe_tmp, rRe_tmp);
	      //~ ret = calc_qref(rPe_tmp, rRe_tmp, q_ref_tmp);

	      if(calc_qref(rPh_initial, rRh_ref_tmp, q_ref_tmp)){
		//~ if(ret){
		q_gc_ref[i+1] = q_ref_tmp;

#ifdef DEBUG_PLUGIN2
		std::cout << "rRh_ref_tmp: " << rRh_ref_tmp << std::endl;
		std::cout << "q_ref_tmp: " << q_ref_tmp << std::endl;
#endif
	      }

	      else
		{
		  std::cerr << "ERROR(gravity_comp): calc_q_ref()" << std::endl;
		  return ret_err;
		}
	    }

	  // Clear parameters for gravity compensation
	  mass = 0.0;
	  fsF_offset  = 0.0, 0.0, 0.0;
	  fsM_offset 	= 0.0, 0.0, 0.0;
	  rG_vec 		= 0.0, 0.0, 0.0;
	  fsPgc 		= 0.0, 0.0, 0.0;

	  step_gc 	= 0;
	  step_fs 	= 0;
	  f_reached 	= false;
	  f_moving 	= false;
	  //f_gc_init = true;

	  f_gc_init = true;
	  return ret_next;
	}

      // Stay here
      else
	{
	  if(step_fs < wait_step)
	    {
	      step_fs++;
	      return ret_next;
	    }

	  // Get force-moment data
	  else if(step_fs < max_step_fs + wait_step)
	    {
	      if(step_fs == wait_step)
		{
		  fsF_tmp[step_gc] = 0.0,0.0,0.0;
		  fsM_tmp[step_gc] = 0.0,0.0,0.0;
		  fsRr_gc[step_gc] = tvmet::trans(rRh * calc_hRfs(q));

#ifdef DEBUG_PLUGIN2
		  std::cout << "hRfs" << calc_hRfs(q) << std::endl;
		  std::cout << "fsRr_gc" << fsRr_gc[step_gc] << std::endl;
#endif
		}

	      double f_out[6];
	      get_raw_forces(f_out);

	      for(int i=0; i<3; i++)
		{
		  fsF_tmp[step_gc][i] += f_out[i];
		  fsM_tmp[step_gc][i] += f_out[i+3];
		}

#ifdef DEBUG_PLUGIN2
	      std::cout << "[" << step_fs << "]"<< "fsF_tmp = " << fsF_tmp[step_gc] << std::endl;
#endif

	      step_fs++;
	      return ret_next;
	    }

	  // get average force-moment data
	  // and go to next step
	  else if(step_fs == max_step_fs + wait_step)
	    {
	      //vector3 = tmp_f;
	      //tmp_f = fsF_tmp[step_gc];
	      //for(int i=0; i<3; i++){
	      fsF_tmp[step_gc] = (1.0/max_step_fs) * fsF_tmp[step_gc];
	      fsM_tmp[step_gc] = (1.0/max_step_fs) * fsM_tmp[step_gc];
	      //}

#ifdef DEBUG_PLUGIN2
	      std::cout << "[" << step_fs << "]" << "fsF_tmp = " << fsF_tmp[step_gc] << std::endl;
#endif

	      step_gc++;
	      step_fs 	= 0;
	      f_reached 	= false;
	      f_moving 	= false;

	      return ret_next;
	    }

	  else return ret_err;
	}
    }
#if 0

  else
    {
      while(step_gc < 5)
	{
	  // Move to reference pos & ori
	  if(!f_reached)
	    {
	      if(moveto(q, q_gc_ref[step_gc]))
		return ret_next;
	      else
		return ret_err;
	    }

	  // get force-moment data
	  else{
	    // stay here
	    if(step_fs < wait_step)
	      {
		step_fs++;
		return ret_next;
	      }
	    else if(step_fs < max_step_fs + wait_step)
	      {
		if(step_fs == wait_step)
		  {
		    fsF_tmp[step_gc] = 0.0,0.0,0.0;
		    fsM_tmp[step_gc] = 0.0,0.0,0.0;
		    fsRr_gc[step_gc] = tvmet::trans(rRh * calc_hRfs(q));
		  }

		double f_out[6];
		get_raw_forces(f_out);

		for(int i=0; i<3; i++)
		  {
		    fsF_tmp[step_gc][i] += f_out[i];
		    fsM_tmp[step_gc][i] += f_out[i+3];
		  }
		step_fs++;
		return ret_next;
	      }

	    // get average force-moment data
	    // and go to next step
	    else if(step_fs == max_step_fs + wait_step)
	      {
		fsF_tmp[step_gc] = (1.0/max_step_fs) * fsF_tmp[step_gc];
		fsM_tmp[step_gc] = (1.0/max_step_fs) * fsM_tmp[step_gc];

#ifdef DEBUG_PLUGIN2
		std::cout <<  "[" << step_fs << "]" << "fsF_tmp = " << fsF_tmp[step_gc] << std::endl;
#endif

		step_gc++;
		step_fs = 0;
		f_reached = false;
		f_moving = false;

		return ret_next;
	      }
	    else return ret_err;
	  }
	}
      if(step_gc == 5)
	{
	  // move to initial pos & ori
	  if(!f_reached)
	    {
	      if(moveto(q, q_gc_ref[0])) return ret_next;
	      else return ret_err;
	    }
	  // go to next step
	  else
	    {
	      step_gc++;
	      f_reached = false;
	      f_moving  = false;

	      return ret_next;
	    }
	}

      else if(step_gc == 6)
	{
	  //compute parameters of gravity
	  //if(!calc_gravity_param_shimizu()){
	  if(!calc_gravity_param())
	    {
	      std::cerr << "ERROR(gravity comp): Failed to calc gravity comp parameters." << std::endl;
	      return ret_err;
	    }
	  else
	    {
	      f_gc = true;
	      step_gc++;
	      return ret_fin;
	    }
	}
      else return ret_err;
    }
}
#endif
 else{
   std::cout << "ERROR(gravity comp): The arm has no force sensor." << std::endl;
   return ret_err;
 }
}

/**********************************************************************************************************************/
// protected functions
/**********************************************************************************************************************/
bool hiroArm::calc_qref(vector3 rPe_ref_in, matrix33 rRe_ref_in, dvector6 &q_out)
{
#ifdef DEBUG_PLUGIN2
  std::cout << "[cacl_qref]" << rPe_ref_in << rRe_ref_in << std::endl;
#endif
  bool ikRet;
#if 0
  for(int j=0; j<4; j++) {
    m_path->setIkGain(ikGains[j]);
    m_path->setMaxIKErrorRot(ikRot[j]);
    m_path->setMaxIKErrorTrans(ikTrans[j]);
    ikRet = m_path->calcInverseKinematics(rPe_ref_in,rRe_ref_in);

    if(!ikRet)
      break;
  }
#endif
#ifdef DEBUG_PLUGIN2
  std::cout << "q_ref: " ;
  for (int i =0; i<ARM_DOF; i++)
    std::cout << (body->joint(NUM_q0 + i)->q )*180/M_PI << ", ";
  std::cout << std::endl;
#endif

  if(ikRet){
    for(int i=0; i<ARM_DOF; i++)	q_out[i] = body->joint(NUM_q0 + i)->q;
    return true;
  }
  else return false;
}

matrix33 hiroArm::calc_hRfs(dvector6 q_in)
{
  // Initialize
  matrix33 hRfs;

#ifdef DEBUG_PLUGIN23
  std::cout << "calc_hRfs in hiroArm" << std::endl;
#endif

  //~ //hRfs = get_rot33(X,q_in[5])*get_rot33(Y,-M_PI/2);
  //~ hRfs = get_rot33(Y,M_PI/2.0) * get_rot33(Z,-q_in[5]);
  return hRfs;
}

/*******************************************************************************************************/
// calc_rPe_rRe
// Convert from robot2endeffector to robot2wrist position and rotation
/*******************************************************************************************************/
void hiroArm::calc_rPe_rRe(vector3 rPh_in, matrix33 rRh_in, vector3 &rPe_out, matrix33 &rRe_out)
{
  matrix33 rRe_tmp;
  rRe_tmp = rRh_in * tvmet::trans(eRh);
  rRe_out = rRe_tmp;
  rPe_out = rPh_in - rRe_tmp * ePh;
}


bool hiroArm::calc_gravity_param_shimizu(){

  //TODO
  int i;
  vector3 f1, f2, m1, m2;
  matrix33 dfsRr1, dfsRr2;

  // difference between 0&1 and 2&3
  f1 = fsF_tmp[2] - fsF_tmp[1];
  f2 = fsF_tmp[4] - fsF_tmp[3];
  m1 = fsM_tmp[2] - fsM_tmp[1];
  m2 = fsM_tmp[4] - fsM_tmp[3];
  dfsRr1 = fsRr_gc[2] - fsRr_gc[1];
  dfsRr2 = fsRr_gc[4] - fsRr_gc[3];

  // compute mass and gravity vector
  {
    vector3 p1, p2;
    SkewToVector(dfsRr1, p1);
    SkewToVector(dfsRr2, p2);

    double p1sq = tvmet::dot(p1, p1);
    double p2sq = tvmet::dot(p2, p2);
    double p12 = tvmet::dot(p1, p2);

    vector3 q1, q2;
    q1 = tvmet::cross(f1,p1);
    q2 = tvmet::cross(f2,p2);
    q1 = (1/p1sq) * q1;
    q2 = (1/p2sq) * q2;

    double denom = p1sq*p2sq - p12*p12;
    if(fabs(denom) < 1.0e-20){
      std::cerr<< "Cannot compute gravity vector" << std::endl;
      return false;
    }
    double k1 =(p2sq*tvmet::dot(p1, q2) + p12*tvmet::dot(p2, q1))/denom;
    double k2 =(p12*tvmet::dot(p1, q2) + p1sq*tvmet::dot(p2, q1))/denom;

    //calcurate mass, rG_vec
    vector3 mg;
    mg = (q1 + q2 + k1*p1 + k2*p2)*0.5;
    mass = tvmet::norm2(mg);
    rG_vec = (1/mass) * mg;
    mass = mass/GACC;

#ifdef DEBUG_PLUGIN2
    std::cout << "GravityComp parameters" << std::endl;
    std::cout << "mass =" << mass << std::endl;
    std::cout << "rG_vec =" << rG_vec << std::endl;
#endif
  }

  // compute centroid
  {
    vector3 bp1, bp2;
    double f1sq = tvmet::dot(f1,f1);
    double f2sq = tvmet::dot(f2,f2);
    double f12 = tvmet::dot(f1,f2);
    double k1,k2,denom;

    bp1 = tvmet::cross(f1,m1);
    bp2 = tvmet::cross(f2,m2);
    bp1 = bp1/f1sq;
    bp2 = bp2/f2sq;

    denom = f1sq*f2sq - f12*f12;
    if(fabs(denom) < 1.0e-20){
      std::cerr<< "Cannot compute centroid" << std::endl;
      return false;
    }
    k1 = (f2sq*tvmet::dot(f1,bp2) + f12*tvmet::dot(f2,bp1))/denom;
    k2 = (f12*tvmet::dot(f1,bp2) + f1sq*tvmet::dot(f2,bp1))/denom;

    for(i=0;i<3;i++)
      fsPgc = (bp1 + bp2 + k1*f1 + k2*f2)*0.5;

#ifdef DEBUG_PLUGIN2
    std::cout << "fsPgc =" << fsPgc << std::endl;
#endif
  }

  // compute offset
  {
    vector3 fsF_grav, fsM_grav;
    fsF_offset = 0.0, 0.0, 0.0;
    fsM_offset = 0.0, 0.0, 0.0;

    for(int p = 0; p < 5; p++){
      fsF_grav = fsRr_gc[p] * (mass * GACC * rG_vec);
      fsM_grav = tvmet::cross(fsPgc, fsF_grav);

      fsF_offset += (fsF_tmp[p] - fsF_grav);
      fsM_offset += (fsM_tmp[p] - fsM_grav);
    }

    fsF_offset = 0.2 * fsF_offset;
    fsM_offset = 0.2 * fsM_offset;

#ifdef DEBUG_PLUGIN2
    std::cout << "fsF_offset =" << fsF_offset << std::endl;
    std::cout << "fsM_offset =" << fsM_offset << std::endl;
#endif
  }

  return true;
}

bool hiroArm::calc_gravity_param()
{
  //  dvector dF(12), dM(12);
  dmatrix dF(12,1), dM(12,1);
  dmatrix dR(12,3), dF_hat(12,3);
  //double dR[12][3], dF_hat[12][3];

  for(int i=0; i<4; i++){
    vector3 tmp;
    matrix33 tmp_dF_hat;
    vector3 tmp_dF(fsF_tmp[i+1] - fsF_tmp[0]);

    tmp = -1 * tmp_dF;
    tmp_dF_hat = OpenHRP::hat(tmp);
#ifdef DEBUG_PLUGIN2
    std::cout << "tmp_dF" << tmp_dF << std::endl;
    std::cout << "tmp_dF_hat" << tmp_dF_hat << std::endl;
#endif

    for(int j=0; j<3; j++){
      dF(i*3+j,0) = tmp_dF[j];
      dM(i*3+j,0) = fsM_tmp[i+1][j] - fsM_tmp[0][j];
      for(int k=0; k<3; k++){
	dR(i*3+j,k) = fsRr_gc[i+1](j,k) - fsRr_gc[0](j,k);
	dF_hat(i*3+j,k) = tmp_dF_hat(j,k);
      }
    }
  }

#ifdef DEBUG_PLUGIN2
  for(int i=0; i<5; i++)
    {
      std::cout << "fsRr_gc" << fsRr_gc[i] << std::endl;
    }
  std::cout << "dF" << dF << std::endl;
  std::cout << "dF_hat" << dF_hat << std::endl;
  std::cout << "dM" << dM << std::endl;
  std::cout << "dR" << dR << std::endl;
#endif

  // pseudo inverse matrixの計算
  dmatrix dR_inv(3,12), dF_hat_inv(3,12);
  int ret;
  ret = OpenHRP::calcPseudoInverse(dR, dR_inv,1.0e-18);
  if(ret !=0){
    std::cout << "ERROR(gravity_comp): calcPseudoInverse ret = " << ret << std::endl;
    return false;
  }
  ret = OpenHRP::calcPseudoInverse(dF_hat, dF_hat_inv,1.0e-18);
  if(ret !=0){
    std::cout << "ERROR(gravity_comp): calcPseudoInverse ret = " << ret << std::endl;
    return false;
  }

#ifdef DEBUG_PLUGIN2
  std::cout << "dR_inv" << dR_inv << std::endl;
  std::cout << "dR_inv * dR" << prod(dR_inv, dR) << std::endl;
  std::cout << "dF_hat_inv" << dF_hat_inv << std::endl;
  std::cout << "dF_hat_inv * dF_hat" << prod(dF_hat_inv,dF_hat) << std::endl;
#endif

  dmatrix tmp1(3,1), tmp2(3,1);
  tmp1 = prod(dR_inv,dF);
  tmp2 = prod(dF_hat_inv,dM);
  for(int i =0; i<3; i++){
    rG_vec[i]  = tmp1(i,0);
    fsPgc[i]= tmp2(i,0);
  }
#ifdef DEBUG_PLUGIN2
  std::cout << "tmp1" << tmp1 << std::endl;
  std::cout << "tmp2" << tmp2 << std::endl;
  std::cout << "rG_vec" << rG_vec << std::endl;
  std::cout << "fsPgc" << fsPgc << std::endl;
#endif

  mass = tvmet::norm2(rG_vec)/GACC;
#ifdef DEBUG_PLUGIN2
  std::cout << "mass =" << mass << std::endl;
  double mass2 = sqrt((rG_vec[0] * rG_vec[0]) + (rG_vec[1] * rG_vec[1]) + (rG_vec[2] * rG_vec[2]))/GACC;

  vector3 rG_vec2;
  rG_vec2 = (1/(mass2*GACC)) *rG_vec;
  std::cout << "mass2 =" << mass2 << std::endl;
  std::cout << "rG_vec2 =" << rG_vec2 << std::endl;
#endif
  rG_vec = tvmet::normalize(rG_vec);

  //offsetの計算
  fsF_offset = 0.0, 0.0, 0.0;
  fsM_offset = 0.0, 0.0, 0.0;
  for(int i=0; i<5; i++){
    vector3 fsF_grav, fsM_grav;
    fsF_grav = fsRr_gc[i]*(mass*GACC*rG_vec);
    //fsM_grav = OpenHRP::hat(fsPgc)*fsF_grav;
    fsM_grav = tvmet::cross(fsPgc, fsF_grav);

    fsF_offset += (fsF_tmp[i] - fsF_grav);
    fsM_offset += (fsM_tmp[i] - fsM_grav);
  }
  fsF_offset = 0.2 * fsF_offset;
  fsM_offset = 0.2 * fsM_offset;

#ifdef DEBUG_PLUGIN2
  std::cout << "GravityComp parameters" << std::endl;
  std::cout << "mass =" << mass << std::endl;
  std::cout << "rG_vec =" << rG_vec << std::endl;
  std::cout << "fsPgc =" << fsPgc << std::endl;
  std::cout << "fsF_offset =" << fsF_offset << std::endl;
  std::cout << "fsM_offset =" << fsM_offset << std::endl;
#endif

  return true;
}

/*********************************************************************************/
// get_raw_forces
// Extract the raw forces from the forcetorque sensor
/*********************************************************************************/
bool hiroArm::get_raw_forces(double f_out[6])
{
  // If we have a good pointer to the nittaFS class, extract values
#if 0
  if(fs != 0)
    {
#endif
      // In simulation simply extract raw values
#ifdef SIMULATION
      for(int i=0; i<6; i++) f_out[i] = raw_forces[i];

      // With the real robot transform from left hand coordinates
#else
      //fs->get_forces(NO_fs, f_out);
#ifdef DEBUG_PLUGIN2
      cerr << "hiroArm::get_raw_forces(): Reading ifs force data." << std::endl;
#endif
      ifs_read_data(0,f_out);
      // The NittaR FT Sensor uses a left-handed coordinate system. We need to convert to a right-hand system here
      // Change the sign of the force and moment z-axis
      f_out[2] = -f_out[2];
      f_out[5] = -f_out[5];
#endif

#ifdef DEBUG_PLUGIN2
      std::cout << "get_raw_forces: \t\t" << f_out[0] << " " << f_out[1] << " " << f_out[2] << " " << f_out[3] << " " << f_out[4] << " " << f_out[5] << std::endl;
#endif
			
      return true;
#if 0
    }
  else
    return false;
#endif
}

/*********************************************************************************/
// calc_gc_forces
// Extract the raw forces from the forcetorque sensor
/*********************************************************************************/
bool hiroArm::calc_gc_forces(vector3 &rFfs_gc_out, vector3 &rMfs_gc_out)
{
  if(f_gc) //  && (fs != 0)) // OldHiroArm Update
    {
      vector3 rF_grav, rM_grav;			// Gravity vector. Magnitude and direction.
      //int i,j;

      // TODO:この計算を左右で変更する必要あり // Left and right need to be changed in this calculation

      // Calculate hRfs
      matrix33 rRfs, hRfs;
      hRfs = calc_hRfs(q);
      rRfs = rRh * hRfs;

      // Gravity Vector - Force
      rF_grav = mass*GACC*rG_vec;					// mg*\hat_g

      // Gravity Vector - Moment
      vector3 oGC_pos;							//matrix33 oGC_pos_hat;
      oGC_pos = rRfs * fsPgc;						//oGC_pos_hat = OpenHRP::hat(oGC_pos);		//rM_grav = oGC_pos_hat * rF_grav;
      rM_grav = tvmet::cross(oGC_pos, rF_grav);

      // Offset
      rFfs_gc_out = (rRfs * (fsF_raw - fsF_offset)) - rF_grav;
      rMfs_gc_out = (rRfs * (fsM_raw - fsM_offset)) - rM_grav;

#ifdef DEBUG_PLUGIN2
      std::cout << "calc_gc_forces: " << std::endl;
      std::cout << "rF_grav = " << rF_grav << std::endl;
      std::cout << "rM_grav = " << rM_grav << std::endl;
      std::cout << "oGC_pos = " << oGC_pos << std::endl;
      std::cout << std::endl;
      //f_out[0] <<" " << f_out[1] <<" " << f_out[2] <<" " << f_out[3] <<" " << f_out[4] <<" " << f_out[5] << std::endl;
#endif

      return true;
    }
  else return false;
}

/************************************************************************************/
// calc_gc_forces_at_hand()
// Compute the arm's force and moment gravitational components at the hand
/************************************************************************************/
void hiroArm::calc_gc_forces_at_hand(vector3 &rFh_out, vector3 &rMh_out)
{
  // 手先での力・モーメントに変換 // Converted to forces and moments in the hand
  rFh_out = rFfs_gc;					// Arm force gravitational component
  rMh_out = rMfs_gc + cross((rRh*hPfs), rFfs_gc);			// Arm moment gravitational component
}

/*********************************************************************************************************************************/
// moveto()
// return false when no path is generated
//------------------------------------------------------------------------------------------------------------------------------
// n-yamanobe 2011/02/23
// TODO: 変な指令値が入力されたときのチェックをしないといけない?  //Do not check If you do not change when the value directive has been entered?
// TODO: QUINTIC_FUNCTIONを実装 // Implement the quintic function
/*********************************************************************************************************************************/
bool hiroArm::moveto(dvector6 q_start_in, dvector6 q_goal_in)
{
  // If not moving
  if(!f_moving)
    {
      // Initialize path parameters
      if(init_path_params(q_start_in, q_goal_in))
	return true;
      else
	return false;
    }

  // If moving
  else
    {
      // Are we at the goal
      if((p_param.q_goal[0] == q_goal_in[0]) && (p_param.q_goal[1] == q_goal_in[1]) && (p_param.q_goal[2] == q_goal_in[2]))
	{
	  // If we have reached the goal, change our flags
	  if(is_reached())
	    {
	      // reached to the reference configuration
	      f_reached = true; f_moving = false; m_time = 0;
	      return true;
	    }

	  // If we have not reached the goal
	  else
	    {
	      // Increase the time step
	      m_time++;

	      // For the constant velocity interpolation method
	      if(p_param.method == CONSTANT_VELOCITY)
		{
		  // Set the qref parameter
		  q_ref = p_param.q_start + DEL_T * m_time * p_param.jvel;

#ifdef DEBUG_PLUGIN2
		  std::cout << "q_ref in moveto: " << q_ref *180/M_PI << std::endl;
		  std::cout << "m_time = " << m_time << std::endl;
		  //~ for (int i =0; i<ARM_DOF; i++)
		  //~ std::cout << (body->joint(NUM_q0 + i)->q )*180/M_PI << ", ";
		  //~ std::cout << std::endl;
#endif

		  f_moving = true;
		  return true;
		}

	      // No methods yet implemented for other interpolation methods
	      else if(p_param.method == QUINTIC_FUNCTION)
		{
		  return false;
		}
	      else
		return false;
	    }
	}
      else
	{
	  if(init_path_params(q_start_in, q_goal_in))
	    return true;
	  else
	    return false;
	}
    }
}

/************************************************************************************/
// init_path_params()
// Initialize path parameters, six of them only if the interpolation method is constant velocity:
// interpolation method, Max. Angular Joint Vel, starting goal joint position,
// ending goal joint position, Distance, and duration.
/************************************************************************************/
bool hiroArm::init_path_params(dvector6 q_start_in, dvector6 q_goal_in)
{
  // calc signed distance
  p_param.distance 	= q_goal_in - q_start_in;	// In radians
  p_param.q_goal 		= q_goal_in;
  p_param.q_start 	= q_start_in;

  // initialize the parameters for the path
  if(p_param.method == CONSTANT_VELOCITY)
    {
      double jvel_tmp = 0.0;
      if(p_param.max_jvel != 0.0)
	jvel_tmp = p_param.max_jvel;
      else
	jvel_tmp = MAX_JVEL;

      // Calculate duration & joint velocities
      if(!calc_duration_jvel(p_param.distance, jvel_tmp))
	return false;
      else
	{
	  if(DEL_T == 0.0)
	    return false;
	  else
	    {
	      //p_q_start = q_start_in;
	      //p_q_goal = q_goal_in;
	      m_time = 0; m_time++;
	      q_ref = p_param.q_start + DEL_T * m_time * p_param.jvel;

#ifdef DEBUG_PLUGIN2
	      std::cout << "jvel = " << p_param.jvel*180/M_PI << std::endl;
	      std::cout << "duration = " << p_param.duration << std::endl;
	      std::cout << "q_ref in moveto: " << q_ref *180/M_PI << std::endl;
#endif

	      f_moving = true;
	      return true;
	    }
	}
    }
  else if(p_param.method == QUINTIC_FUNCTION)
    {
      return false;
    }
  else
    {
      return false;
    }
}

/************************************************************************************/
// calc_duration_jvel
// Duration is calculated from distance and velocity
// TODO: エラー処理 // Error Handling
/************************************************************************************/
bool hiroArm::calc_duration_jvel(dvector6 distance_in, double max_jvel_in)
{
  int 	err_num = 0;	// Error count
  double  t_max = -1.0;	// Variable to hold the longest durations

  // Calc duration for each joint
  for(int i=0; i<ARM_DOF; i++)
    {
      double d = fabs(distance_in[i]);
      double t = 0.0;

      // For very close proximities
      if(d < TOLERANCE)
	err_num++;						// If too near, increase error count

      // For standard range calculations
      else
	{
	  t = d/max_jvel_in;				// time = distance/velocity
	  if(t > TIME_LIMIT)
	    err_num++;					// If too long, increase error count

	  // Keep max duration in variable
	  else
	    {
	      if(t > t_max)
		t_max = t;
	    }
	}
    }

  // If there is an error for each of the joints, then this can't be done.
  if(err_num == ARM_DOF)
    return false;
  else
    {
      // Save duration to protected member
      p_param.duration = ceil(t_max/DEL_T);

      //Calc each joint velocity
      for(int i=0; i<ARM_DOF; i++)
	{
	  double d = fabs(distance_in[i]);

	  // Check distance limits
	  if(d<TOLERANCE) p_param.jvel[i] = 0.0;    // too near

	  // Calculate the joint velocity by: distance/time
	  else
	    {
	      p_param.jvel[i] = distance_in[i]/(p_param.duration*DEL_T);
	    }
	}
      return true;
    }
}

/*double path_generator::calc_duration(double distance, double speed)

  double d = fabs(distance);
  double v = fabs(speed);
  double t = -1.0;
  if(d < TOLERANCE) return MIN_PERIOD;
  if(v < TOLERANCE) return -1.0;  // too slow
  t = d/v;
  if(t > TIME_LIMIT) return -1.0; // too long time
  return t;
  }*/

bool hiroArm::is_reached()
{

  if(!f_moving)
    return false;

  if(m_time <= p_param.duration)
    return false;
  else
    {
      bool res = true;
      //~ for(int i=0; i<ARM_DOF; i++){
      //~ if(fabs(q[i]-p_param.q_goal[i]) > TOLERANCE){
      //~ res = false;
      //~ break;
      //~ }
      //~ }
      return res;
    }
}

bool hiroArm::is_moving()
{
  return f_moving;
}

/****************************************************************************/
// set_path_params()
// Takes a path_params structure that describes: interpolation method,
// Max. Angular Joint Vel, starting goal joint position, ending goal joint position,
// Distance, and duration.
/****************************************************************************/
bool hiroArm::set_path_params(path_params param_in)
{
  if(f_moving)
    return false;
  p_param = param_in;
  return true;
}

/****************************************************************************/
// set_path_method()
// Set one of three interpolation methods:
// 1) Constant velocity, 2) Quintic function, or 3) End of Interpolation
/****************************************************************************/
bool hiroArm::set_path_method(interpolation method_in)
{
  if(f_moving)
    return false;

  // Set path method to protected member
  p_param.method = method_in;
  return true;
}

/****************************************************************************/
// set_path_max_jvel
// Check joint angle velocity limits. Adjust for ceil/floor if necessary.
/****************************************************************************/
bool hiroArm::set_path_max_jvel(double max_jvel_in)
{
  // Don't change values while moving
  if(f_moving)
    return false;

  /*for(int i = 0; i < ARM_DOF; i++){if(max_v_in[i] > MAX_JVEL) p_param.max_v[i] = MAX_JVEL;else p_param.max_v[i] = max_v_in[i];}*/

  // surpases top velocity, place ceiling
  if(max_jvel_in > MAX_JVEL)
    p_param.max_jvel = MAX_JVEL;

  // If it drops under least velocity, place a floor
  else if(max_jvel_in < 0.0)
    p_param.max_jvel = -max_jvel_in;

  // Else keep the same
  else
    p_param.max_jvel = max_jvel_in;
  return true;
}

/*bool hiroArm::set_path_max_a(double max_duration_in){
  if(f_moving) return false;
  p_param.max_a = max_a_in;
  return true;
  }*/

/*void hiroArm::set_sampling_perio(double period_in){
  period = period_in;
  }*/

/**********************************************************************************/
// get_rot33()
// Perform a rotation about the indicated dir direction by rad radians.
/**********************************************************************************/
matrix33 hiroArm::get_rot33(int dir, double rad)
{
  // Initialize
  matrix33 rot_temp;

  // X-Rotation
  if(dir==X)
    {
      rot_temp= 	1,         0,         0,
	0,  cos(rad), -sin(rad),
	0,  sin(rad),  cos(rad);
      return rot_temp;
    }

  // Y-Rotation
  if(dir==Y)
    {
      rot_temp= 	cos(rad),	0, sin(rad),
	0,	1,        0,
	-sin(rad),	0, cos(rad);
      return rot_temp;
    }

  // Z-Rotation
  if(dir==Z)
    {
      rot_temp= 	cos(rad), -sin(rad),	0,
	sin(rad),  cos(rad),	0,
	0,         0,	1;
      return rot_temp;
    }
  return rot_temp;
}

/**********************************************************************************/
// SkewToVector
// Convert a skew matrix into a vector
/**********************************************************************************/
void hiroArm::SkewToVector(matrix33 skew, vector3 &vec)
{
  vec[0] = (skew(2,1) - skew(1,2))*0.5;
  vec[1] = (skew(0,2) - skew(2,0))*0.5;
  vec[2] = (skew(1,0) - skew(0,1))*0.5;
}

/*matrix33 hiroArm::rotFromRpy(vector3 rpy_in)
  {
  double cos_x = cos(rpy_in(0)), sin_x = sin(rpy_in(0));
  double cos_y = cos(rpy_in(1)), sin_y = sin(rpy_in(1));
  double cos_z = cos(rpy_in(2)), sin_z = sin(rpy_in(2));

  R_out(0,0) = cos_y*cos_z;
  R_out(0,1) = sin_x*sin_y*cos_z - cos_x*sin_z;
  R_out(0,2) = cos_x*sin_y*cos_z + sin_x*sin_z;
  R_out(1,0) = cp*sy;
  R_out(1,1) = sin_x*sin_y*sin_z + cos_x*cos_z;
  R_out(1,2) = cos_x*sin_y*sin_z - sin_x*cos_z;
  R_out(2,0) = -sin_y;
  R_out(2,1) = sin_x*cos_y;
  R_out(2,2) = cos_x*cos_y;

  return R_out;
  }*/
/*
 *  hiroArm Master class
 */

hiroArmMas::hiroArmMas(std::string name_in, BodyPtr body_in, unsigned int num_q0_in, double period_in, float ang_limits_in[6][5],
		       vector3 ePh_in, matrix33 eRh_in, vector3 hPfs_in)
  :hiroArm(name_in, body_in, num_q0_in, period_in, ang_limits_in, ePh_in, eRh_in, hPfs_in)
{
#ifdef DEBUG_PLUGIN2
  std::cout << "exit hiroArmMas constructor" << std::endl;
#endif
}

/*
 * //~ void hiroArmMas::savedata()
 //~ {
 //~ fprintf(fp1,"%f %f %f %f %f %f\n",rFfs_gc[0],rFfs_gc[1],rFfs_gc[2],rMfs_gc[0],rMfs_gc[1],rMfs_gc[2]);
 //~ vector3 tmp = OpenHRP::rpyFromRot(rRh);
 //~ fprintf(fp2,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",rPh[0],rPh[1],rPh[2],tmp[0],tmp[1],tmp[2], rRh(0,0),rRh(0,1),rRh(0,2), rRh(1,0),rRh(1,1),rRh(1,2), rRh(2,0),rRh(2,1),rRh(2,2));
 //~ }
 */

// Calculate rotation for moment vector
matrix33 hiroArmMas::calc_hRfs(dvector6 q_in)
{

#ifdef DEBUG_PLUGIN23
  std::cout << "calc_hRfs in hiroArmMas" << std::endl;
#endif
  matrix33 hRfs;

  hRfs = get_rot33(X,-q_in[5]) * get_rot33(Y,-(M_PI/2.0));	//hRfs = get_rot33(X,q_in[5])*get_rot33(Y,-M_PI/2);
  return hRfs;
}

/*
 *  hiroArm Slave class
 */
hiroArmSla::hiroArmSla(	std::string name_in, BodyPtr body_in, unsigned int num_q0_in, double period_in, float ang_limits_in[6][5], vector3 ePh_in, matrix33 eRh_in, vector3 hPfs_in, matrix33 hRfs_in)
  :hiroArm(name_in, body_in, num_q0_in, period_in, ang_limits_in, ePh_in, eRh_in, hPfs_in)
{

  hRfs = hRfs_in;													//hRfs = get_rot33(Y,M_PI/2.0) * get_rot33(Z,(3*M_PI/4.0));
#ifdef DEBUG_PLUGIN2
  std::cout << "exit hiroArmSla constructor" << std::endl;
#endif
}

matrix33 hiroArmSla::calc_hRfs(dvector6 q_in)
{

#ifdef DEBUG_PLUGIN2
  std::cout << "calc_hRfs in hiroArmSla" << std::endl;
#endif

  return hRfs;
}

/*//~ void hiroArmSla::calc_forces_at_hand(vector3 &rFh_out, vector3 &rMh_out)
//~ {
//~ // 手先での力・モーメントに変換
//~ rFh_out = rFfs_gc;
//~ rMh_out = rMfs_gc + cross((rRh*hPfs), rFfs_gc);
//~ }
*/

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
