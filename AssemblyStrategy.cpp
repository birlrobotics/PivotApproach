/*--------------------------------
  ANGLES * AssemblyStrategy.cpp
  *  Created on: Mar 19, 2012
  *      Author: juan
  --------------------------------*/
#include <sys/types.h>
#include <sys/stat.h>
#include "AssemblyStrategy.h"
///---------------------------------------------------------------------------------------------------------------------------------------------------//
/************************************************************* DESIGN PARAMETERS AND FLAGS ************************************************************/
// ----------------------------------------------------- PLEASE SEE MORE DESIGN PARAMETERS IN hiroArm ------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------------------------------
#define PA10					0
#define HIRO					1
//----------------------------------------------------------------------------------------------------------------------------------------------------
// ASSEMBLY_STRATEGY_AUTOMATA STATES
//----------------------------------------------------------------------------------------------------------------------------------------------------
#define PA_FINISH 				10 			// value returned when the assembly is finished
//----------------------------------------------------------------------------------------------------------------------------------------------------
// FAILURE CHARACTERIZATION VARIABLES
//----------------------------------------------------------------------------------------------------------------------------------------------------
#define PATH_DEVIATION_MAGNITUDE_X   0.0105		// xDir (4) 0.0105 // (3) 0.0095 // (2) 0.0085 // (1)0.0075 // Parameter used to study Failure Characterization.
#define PATH_DEVIATION_MAGNITUDE_Y   0.0		// yDir (4) 0.0105 // (3) 0.0095 // (2) 0.0085 // (1)0.0075
#define ANGLE_DEVIATION_MAGNITUDE  0.4363		// YawDir 0.08725, 0.1745, 0.2618, 0.3490, 0.4363, 0.5235

//----------------------------------------------------------------------------------------------------------------------------------------------------
// FILTERING
//----------------------------------------------------------------------------------------------------------------------------------------------------
#define   FILT_FLAG				1			// Used to enable or disable the filtering of the torques signal. Filtering uses FilterTools class and is called in ::StateMachine
//-----------------------------------------------------------------------------------------------------------------------------------------
// DEBUGGING
//----------------------------------------------------------------------------------------------------------------------------------------------------
#define DEBUG_AS				0			// Flag used to test functions with hard-coded data
#define DB_PRINT				0 			// Used to write angles, cart positions, forces, and states to std::cerr
#define DB_WRITE				1	   		// Used to write angles, cart position, forces, and states to FILE.
#define DB_TIME 				0		 	// Used to print timing duration of functions
//----------------------------------------------------------------------------------------------------------------------------------------------------
// WORLD COORDINATES
//----------------------------------------------------------------------------------------------------------------------------------------------------
// Vertical Axes in WORLD coordinates using a right handed coordinate system: +Z:Up, +Y:Right, +X:Out
#if(PA10==1)
	#define UP_AXIS  			2   		// Defines the local wrist axis for a robot. Used to set desired forces.
#else // HIRO
	#define UP_AXIS  			0			// x becomes up/down after transform
	#define FWD_AXIS			2			// z becomes backward/forward after transform
	#define SIDE_AXIS 			1
#endif
//----------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------------------------------
// Default Constructor
//----------------------------------------------------------------------------------------------------------------------------------------------------
AssemblyStrategy::AssemblyStrategy()
{
#ifdef DEBUG_PLUGIN3
  std::cerr << "AssemblyStrategy(): Entering Constructor" << std::endl;
#endif

  // Control Method
  controlType = controlBasis;

  // Strategy Default
  approachType = SideApproach;
  approachFlag = false;

  // Pivot Approach Contact
  noContact = true;
  completionFlag = false;

  // Manipulation Test
  testCounter 			= 0;
  compositionTypeTest	= 0;	// Force or Moment
  DesForceSwitch		= 0;	// Switch which element to activate
  initialFlag 			= true;
  completionFlag 		= false;

  // State Transition
  signChanger			= 0.0;
  switchFlag 			= true;
  nextState 			= false;
  hsaHIROTransitionExepction = normal;
  State 				= 1;
  transitionTime		= 0.0;
  transitionTimebool	= false;
  state3_zPos			= 0.0;
  state3_zPrevPos		= 0.0;
  SA_S4_Height 	 		= 0.0;
  mating2EndTime 		= 1.0;

  // Control Basis Members
  NumCtlrs				= 1;
  ErrorFlag 			= true;
  ctrlInitFlag			= true;

  // Data vectors
  // DesIKin holds the final desired position for contact with the pivoting dock
  if(PA10)
    {
      DesIKin(0) =  0.0000;
      DesIKin(1) = -0.7330;
      DesIKin(2) =  0.0075;
      DesIKin(3) =  3.1415;
      DesIKin(4) =  0.0000;
      DesIKin(5) = -3.1415;
    }
  else if(HIRO)
    {
      DesIKin(0) =  0.0000;
      DesIKin(1) = -0.7330;
      DesIKin(2) =  0.0075;
      DesIKin(3) =  3.1415;
      DesIKin(4) =  0.0000;
      DesIKin(5) = -3.1415;
    }
  else
    for(int i=0; i<6; i++) DesIKin(i)=0.0;

  for(int i=0; i<6; i++)
    {
      CurCartPos(i)			= 0.0;
      DesCartPos(i)			= 0.0;
      contactPos(i)			= 0.0;
      CurJointAngles(i)		= 0.0;
      JointAngleUpdate(i)	= 0.0;
    }

  // Control Basis and FilterTools
  c1 					= 0;
  c2 					= 0;
  c3 					= 0;
  ft 					= 0;

  // Kinematics
  IKinTestFlag  		= 0;
  zerothJoint			= 0;
  transWrist2CamXEff	= 0.0;
  ContactWristAngle		= 0.0, -1.5708, 0.0;
  for(int i=0; i<6; i++) avgSig = 0.0;

  // Motion
  wrist_r				= (0);
  EndEff_r_org			= (0);
  wrist_p				= (0);
  EndEff_p_org			= (0);
  divPoint 				= (0);

  // File Paths: save global path's to internal variables
  strcpy(strState, 			"");			// File designed to save the time at which new states occur
  strcpy(strForces,			"");			// File designed to save the value of forces and moments registered in the task
  strcpy(strTrajState1,  	"");			// File that contains the desired position trajectory to be followed
  strcpy(strAngles,  		"");			// File designed to save the joint angles during the task
  strcpy(strCartPos, 		"");			// File designed to save the cartesian positions during the task

  // Imported Values
  cur_time				= 0.0;						// used in transitions
  flagFiltering 		= FILT_FLAG;				// Set filtering flag to parameter define in preprocessor
  // Control Basis
  momentGainFactor 		= 1.0;

#ifdef DEBUG_PLUGIN3
  std::cerr << "AssemblyStrategy(): Exiting Constructor" << std::endl;
#endif

}
//----------------------------------------------------------------------------------------------------------------------------------------------------
// Overloaded Constructor
// Called from the hiroArm class.
// Constructor called from HIRO Simulation and passes variables like position, rotation, jacobian, curr_time, curr_force for a given arm. just working with one arm so far.
//----------------------------------------------------------------------------------------------------------------------------------------------------
AssemblyStrategy::AssemblyStrategy(int NUM_q0, vector3 base2endEffectorPos, matrix33 base2endEffectorRot, vector3 ePh, double momentGainFac)
{
	// Default Control Methodology
	controlType = controlBasis;

	// Strategy Default
	approachType = SideApproach;
	approachFlag = false;

	// Pivot Approach Contact
	noContact = true;
	completionFlag = false;

	// Manipulation Test
	testCounter 			= 0;
	compositionTypeTest		= 0;	// Force or Moment
	DesForceSwitch			= 0;	// Switch which element to activate
	initialFlag 			= true;

	// State Transition
	signChanger				= 0.0;
	switchFlag 				= true;
	nextState 				= false;
	hsaHIROTransitionExepction = normal;
	State 					= 1;
	transitionTime			= 0.0;
	transitionTimebool 		= false;
	state3_zPos				= 0.0;
	state3_zPrevPos			= 0.0;
	SA_S4_Height 	 		= 0.0;
	mating2EndTime 			= 1.00;

	// Control Basis Members
	NumCtlrs				= 1;
	ErrorFlag 				= true;
	ctrlInitFlag			= true;

	// Data vectors
	// DesIKin holds the final desired position for contact with the pivoting dock
	if(PA10)
	{
		DesIKin(0) =  0.0000;
		DesIKin(1) = -0.7330;
		DesIKin(2) =  0.0075;
		DesIKin(3) =  3.1415;
		DesIKin(4) =  0.0000;
		DesIKin(5) = -3.1415;
	}
	else if(HIRO)
	{
		DesIKin(0) =  0.0000;
		DesIKin(1) = -0.7330;
		DesIKin(2) =  0.0075;
		DesIKin(3) =  3.1415;
		DesIKin(4) =  0.0000;
		DesIKin(5) = -3.1415;
	}
	else
		for(int i=0; i<6; i++) DesIKin(i)=0.0;

	for(int i=0; i<6; i++)
	{
		CurCartPos(i)		= 0.0;
		DesCartPos(i)		= 0.0;
		contactPos(i)		= 0.0;
		CurJointAngles(i)	= 0.0;
		JointAngleUpdate(i)	= 0.0;
	}

	// Control Basis
	c1 						= 0;
	c2 						= 0;
	c3 						= 0;
	ft 						= 0;

	// Kinematics
	IKinTestFlag  			= 0;
	zerothJoint 			= NUM_q0;
	transWrist2CamXEff		= ePh;					// wrist2endeffecter
	ContactWristAngle		= 0.0, -1.5708, 0.0;
	for(int i=0; i<6; i++) avgSig = 0.0;

	#ifdef DEBUG_PLUGIN3
		std::cerr << "AssemblyStrategy(): Converting rot2rpy and position transformation." << std::endl;
	#endif

	// Motion: true end-effector position and rotation
	EndEff_r_org 			= rpyFromRot(base2endEffectorRot);
	wrist_r = EndEff_r_org;

	EndEff_p_org			= base2endEffectorPos;
	wrist_p = EndEff_p_org;

	divPoint(0) = 0;

	#ifdef DEBUG_PLUGIN3
		std::cerr << "The EndEffector position during the constructor is: " << wrist_p(0) << "\t" << wrist_p(1) << "\t" << wrist_p(2) << "\t" << wrist_r(0) << "\t" << wrist_r(1) << "\t" << wrist_r(2) << std::endl;
	#endif

	// File Paths: save global path's to internal variables
	strcpy(strState, 		"");				// File designed to save the time at which new states occur
	strcpy(strForces,		"");				// File designed to save the value of forces and moments registered in the task
	strcpy(strTrajState1,  	"");				// File that contains the desired position trajectory to be followed
	strcpy(strAngles,  		"");				// File design to save the value of joint angles registered in the task
	strcpy(strCartPos, 		"");				// File designed to save the Cartesian positions registered in the task

	// Imported Values
	cur_time				= 0.0;					// used in transitions
	flagFiltering 			= FILT_FLAG;			// Set filtering flag to parameter define in preprocessor

	// ControlBasis
	momentGainFactor = momentGainFac;

	#ifdef DEBUG_PLUGIN3
		std::cerr << "AssemblyStrategy::AssemblyStrategy(num_q0,base2endeffPoss,base2endeffRot,ePh,momentGainFac) - exiting\n-----------------------------------------------------------------------------------------" << std::endl;
	#endif
}

//----------------------------------------------------------------------------------------------------------------------------------------------------
// Destructor
//----------------------------------------------------------------------------------------------------------------------------------------------------
AssemblyStrategy::~AssemblyStrategy()
{
  // Set controller pointers to null.
  if(c1!=NULL) c1 = NULL;
  if(c2!=NULL) c2 = NULL;
  if(c3!=NULL) c3 = NULL;

  // Delete the filter object
  delete ft;
  ft = NULL;

  // Close all read and write files
  CloseFiles();
}

//----------------------------------------------------------------------------------------------------------------------------------------------------
// Initialize
//----------------------------------------------------------------------------------------------------------------------------------------------------
// 1) Allows user to set specific path files for the MOTION_FILE, ANGLES_FILE, CART_POS_FILE, STATE_FILE, and FORCE_FILE.
// 2) Extracts information from waypoints found in the trajectory file to be used in moveRobot().
// 3) Saves the homing position and rotation matrix of the wrist
// 4) Assigns the kind of control method and strategy motion that we will use. Currently (4) options: Straight Line Approach (PA10), Pivot Approach (PA10), Side Approach (HIRO), Failure Characterization (HIRO).
// 5) Allocates the filter class
//----------------------------------------------------------------------------------------------------------------------------------------------------
int AssemblyStrategy::Initialize(char TrajState1[STR_LEN], char TrajState2[STR_LEN], char AnglesDir[STR_LEN], char CartPosDir[STR_LEN], char StateDir[STR_LEN], char ForcesDir[STR_LEN],
									 vector3 pos, matrix33 rot, double CurAngles[15],
									 int strategyType, int controlMethodType)
{
#ifdef DEBUG_PLUGIN3
  std::cerr << "\nAssemblyStrategy::Initialize - entered" << std::endl;
  std::cerr << "\n These are the file names: \n" << TrajState1 << "\n" << TrajState2 << "\n" << AnglesDir << "\n" << CartPosDir << "\n" << StateDir << "\n" << ForcesDir << "\n" << std::endl;
#endif

  //--------------------------------------------------------------------------------------------------------------------------------
  // 1) Read user specified files. If null strings, go with default values. Trajectory File for State1. If not null, then read. Otherwise we used pre-saved values that were written during the constructor.
  //--------------------------------------------------------------------------------------------------------------------------------
  if(abs(strcmp(TrajState1,"")))
    strcpy(strTrajState1,TrajState1);

  // Trajectory File for State2. If not null, then read
  if(abs(strcmp(TrajState2,"")))
    strcpy(strTrajState2,TrajState2);

  // Joint Angles File. If not null, then read
  if(abs(strcmp(AnglesDir,"")))
    strcpy(strAngles,AnglesDir);

  // Cartesian Positions File. If not null, then read
  if(abs(strcmp(CartPosDir,"")))
    strcpy(strCartPos,CartPosDir);

  // State File. If not null, then read
  if(abs(strcmp(StateDir,"")))
    strcpy(strState,StateDir);

  // Trajectory File. If not null, then read
  if(abs(strcmp(ForcesDir,"")))
    strcpy(strForces,ForcesDir);

  //--------------------------------------------------------------------------------------------------------------------------------
  // 2) Open files for reading and writing data.
  //--------------------------------------------------------------------------------------------------------------------------------
  OpenFiles();

  // Here we read the desired trajectory file for state 1 and save the data in local variables for position and orientation
#ifdef DEBUG_PLUGIN3
  std::cerr << "\nAssemblyStrategy::Initialize - extract information from waypoint file: " << strTrajState1 << std::endl;
#endif

  //--------------------------------------------------------------------------------------------------------------------------------
  // 3) Reassign original end effector position and rotation
  //--------------------------------------------------------------------------------------------------------------------------------
  EndEff_r_org 			= rpyFromRot(rot);
  wrist_r = EndEff_r_org;

  EndEff_p_org			= pos;
  wrist_p = EndEff_p_org;

#ifdef DEBUG_PLUGIN3
  // Cartesian Position
  std::cerr 	<< "/--------------------------------------------------------------------------------------------------------------------------------------------------------------------/\n"
    "AssemblyStrategy::Initialize(): \nThe EndEffector position during initialization is: "
		<< wrist_p(0) << "\t" << wrist_p(1) << "\t" << wrist_p(2) << "\t" << wrist_r(0) << "\t" << wrist_r(1) << "\t" << wrist_r(2)
    // Joint Angles
		<< "\n\nThe 15 body joint angles in radians are: "
		<< CurAngles[0] << "\t" << CurAngles[1] << "\t" << CurAngles[2] << "\t" << CurAngles[3] << "\t" << CurAngles[4] << "\t" << CurAngles[5] << "\t"
		<< CurAngles[6] << "\t" << CurAngles[7] << "\t" << CurAngles[8] << "\t" << CurAngles[9] << "\t" << CurAngles[10] << "\t" << CurAngles[11]
		<< CurAngles[12] << "\t" << CurAngles[13] << "\t" << CurAngles[14] << "\t"
		<< "\n/--------------------------------------------------------------------------------------------------------------------------------------------------------------------/" << std::endl;
#endif

  vector3 rpy=rpyFromRot(rot);

  // Extract information from the way point file for state 1
  ProcessTrajFile(strTrajState1,1,pos,rpy,0.0);	// string path,state,curr_pos,cur_time

  // Copy wrist (endeffector) position and rotation matrix to internal variables
  EndEff_p_org = pos;
  EndEff_r_org = rpy;

  // AssemblyStrategy needs end-effector position/rotation.
  // hiroArm needs wrist position/orientation.
  // When a position comes into AssemblyStrategy it is immediately converted to end-effector position/rotation/
  // AssemblyStrategy::moveRobot() will convert it back to wrist coordinates which is then used by fwd/inv kinematics.
  wrist2EndEffTrans(EndEff_p_org,EndEff_r_org);

  //--------------------------------------------------------------------------------------------------------------------------------
  // 4) Assign control method and motion strategy
  //--------------------------------------------------------------------------------------------------------------------------------

  // (A) Control Method Selection
  if(controlMethodType==motionData)
  {
	  controlType = motionData;
  }
  else if(controlMethodType==controlBasis)// Control Basis Approach
	  controlType = controlBasis;

  // (B) Strategy Selection
  if(strategyType==StraightLineApproach)
  {
	  approachFlag = true;
	  approachType = StraightLineApproach;
  }
  else if(strategyType==PivotApproach)
  {
	  approachFlag = false;
	  approachType = PivotApproach;
  }
  else if(strategyType==SideApproach)
  {
	  approachFlag = false;
	  approachType = SideApproach;
  }

  else if(strategyType==FailureCharacerization)
  {
	  approachFlag = false;
	  approachType = FailureCharacerization;

	  // Assign appropriate values to the divPoint array which will modify waypoint values.
	  /** Failure Case Characterization Vector **/
	  // Will only write values into x,y,roll,and yaw since these will not greatly affect the motion of the robot.

	  // Keep the z-axis, the pitch, and yaw at zero
	  divPoint(2)=0; divPoint(4)=0;divPoint(5)=0;

	  divPoint(0) 						= PATH_DEVIATION_MAGNITUDE_X;			// x-axis
	  divPoint(1) 						= PATH_DEVIATION_MAGNITUDE_Y; 			// y-axis
	  if(divPoint(0)>0.00) divPoint(2) 	=-0.005;								// Change z-axis
	  else		  		   divPoint(2) 	= 0.000;								// If there is deviation in the x-axis, then we need to add a deviation in the z-axis with value of -0.005 such that there is contact after moving forward.
	  divPoint(3) 						= ANGLE_DEVIATION_MAGNITUDE;			// Yaw ANGLE_DEVIATION_MAGNITUDE
  }

  // For the first iteration they are both the same.
  wrist_p = EndEff_p_org;
  wrist_r = EndEff_r_org;
  //--------------------------------------------------------------------------------------------------------------------------------
  // 5) Filter Class Allocation
  //--------------------------------------------------------------------------------------------------------------------------------
  ft = new FilterTools();

#ifdef DEBUG_PLUGIN3
  std::cerr << "\nAssemblyStrategy::Initialize - exited" << std::endl;
#endif

  return 1;
}

/****************************************************************************************************************************************************************************************************/
// StateMachine()
// The state machine will contain the appropriate states and desired actions that correspond to a desired control strategy.
// Three common control strategies are: StraightLineApproach, PivotApproach, and a variant of the PivotApproach, the SideApproach.
// Each state will call the ControlBasis class to execute a simple or compound controller with a defined desired position or force values.
// Transitions are defined in StateSwitcher() and are unique to each approach.
// The output of the function is the updated CurrentJointAngles which are passed to hiroArm class and then to the forceSensorPlugin class.
//
// Notes about position:
// The position received by this function corresponds to the endeffector position. For the IKinComposition, which uses inverse kinematics for position controll, the updated desired position is
// translated back to the wrist position and then the inverse kinematics are computed from there.
//
// Coordinate Frame of Reference:
// For force/moment control compositions the incoming desired forces and moments are set according to the world coordinate frame. Within each state those forces/moments are rotated according to the local
// wrist coordinate frame.
// Position is always computed with respect to the base/world coordinate frame.
/****************************************************************************************************************************************************************************************************/
int AssemblyStrategy::StateMachine(TestAxis 		axis,				/*in*/
										CtrlStrategy 	approach,			/*in*/
										JointPathPtr 	m_path,				/*in*/
										BodyPtr 		bodyPtr,			/*in*/
										double 			cur_time,			/*in*/
										vector3&		pos,				/*in*/	// end-effector pos
										matrix33& 		rot,				/*in*/	// end-effector rot
										dvector6 		currForces,			/*in*/
										dvector6& 		JointAngleUpdate,	/*out*/
										dvector6& 		CurrAngles,			/*out*/
										dmatrix 		Jacobian,			/*in*/
										dmatrix 		PseudoJacobian) 	/*in*/
{
	/*----------------------------- Local variables ----------------------------------------*/
	//float 		MaxErrNorm = 0.0;
	int 		ret = 0;

	double 		filteredSig[6];
	double 		temp[6];
	double 		ErrorNorm1 = 0, ErrorNorm2 = 0;

	dvector6 	n6;
	matrix33 	attitude(0);
	Vector3 	n(0), DesXYZ(0), DesRPY(0), DesForce(0), DesMoment(0), CurXYZ(0), CurRPY(0);

	// Time variables
	double duration = -1.0;
	timeval startTime, endTime;			// start and end time variables

#ifdef DEBUG_PLUGIN3
	std::cerr << "Entering AssemblyStrategy::StateMachine" << std::endl;
#endif	

	if(DB_TIME)
		gettimeofday(&startTime,NULL); 		// Start computing cycle time for the control function

	/************************************************* LOW PASS FILTERING *************************************************************/
	if(flagFiltering)	// Copy force/torque data into a double array to be used by the Low-Pass Filter
	{
		for(int i=0; i<ARM_DOF; i++)
			temp[i]=currForces(i);

		ft->LowPassFilter(temp,filteredSig);
//		ft->LowPassFilter(currForces,avgSig);
//		ft->secOrderFilter(temp,filteredSig);
//		ft->secOrderFilter(currForces,avgSig);

		for(int i=0;i<6;i++)
			avgSig(i) = filteredSig[i];
		//ft->LowPassFilter(currForces,avgSig);
	}
	else
		avgSig = currForces;

	/****************************************** Wrist/EndEffecter Transformation *****************************************************/
	// Convert from wrist position/rotation to endEffector position/rotation. I.e:
	// 		AssemblyStrategy needs endEffector position/rotation.
	// 		hiroArm needs wrist position/orientation.
	// When a position comes into AssemblyStrategy it is immediately converted to endEffector position/rotation/
	// AssemblyStrategy::mveRobot() will convert it back wrist coordinates which is then used by fwd/inv kinematics.
	CurRPY = rpyFromRot(rot);
	//wrist2EndEffTrans(pos,CurRPY);

	/*************************************************************** TEST APPROACH ********************************************************************************/
	if(approach==ManipulationTest)	// something strange where forceSensorPlugin cannot see ManipulationTest... rigged here.
	{
		manipulationTest(axis, completionFlag,
						m_path,bodyPtr,
						JointAngleUpdate,CurrAngles,
						DesForce,DesMoment,n6,
						pos,rot,cur_time,
						Jacobian,PseudoJacobian);
	}

	/*------------------------------------------------------------------ PIVOT APPROACH -------------------------------------------------------------------*/
	else if(approach==PivotApproach)
	{
		switch (State)
		{
		/*-------------------------------------------------- Approach --------------------------------------------------*/
		case paApproach:// Move, such that the TCP makes contact with the docking pivot in the male camera part at an angle.
		{
			// Initialize
			if(ctrlInitFlag)
			{
#ifdef DEBUG_PLUGIN3
				std::cerr << "State 1 in AssemblyStrategy::StateMachine" << std::endl;
#endif	 

				nextState	 = false;
				ctrlInitFlag = false;
			}

			// Primitive Ikin controller: Use a preplanned trajectory to move the wrist at an angle until contact.
			// DesIKin are hardcoded into the code.
			ret=ControlCompositions(m_path,bodyPtr,JointAngleUpdate,CurrAngles,approach,IKinComposition,n,DesForce,n6,ErrorNorm1,ErrorNorm2,pos,rot,cur_time,Jacobian,PseudoJacobian);

			// 0) Check for state termination.
			StateSwitcher(approach,State,ErrorNorm1,ErrorNorm2,pos,rot,CurrAngles,avgSig,cur_time); // Fix to add ErrorNorm2
		}
		break;

		/*-------------------------------------------------- Rotation --------------------------------------------------*/
		case paRotation: // Rotate until contact
		{
			// Initialize
			if(ctrlInitFlag)
			{
#ifdef DEBUG_PLUGIN3
				std::cerr << "State 2 in AssemblyStrategy::StateMachine" << std::endl;
#endif	 

				nextState	 = false;
				ctrlInitFlag = false;
				// Set up a new trajectory file with current information
				ProcessTrajFile(strTrajState2,State,pos,CurRPY,cur_time);
				EndEff_p_org = pos;
				EndEff_r_org = CurRPY;
			}

			//  Primitive Ikin controller: Use a preplanned trajectory to rotate wrist until further contact
			ret=ControlCompositions(m_path,bodyPtr,JointAngleUpdate,CurrAngles,approach, IKinComposition,n,DesForce,n6,ErrorNorm1,ErrorNorm2,pos,rot,cur_time,Jacobian,PseudoJacobian);

			// Check for state termination. Also free allocated resources.
			StateSwitcher(approach,State,ErrorNorm1,ErrorNorm2,pos,rot,CurrAngles,avgSig,cur_time);
		}
		break;

		/*-------------------------------------------------- Alignment --------------------------------------------------*/
		case paAlignment:
		{
			// Initialize
			if(ctrlInitFlag)
			{
#ifdef DEBUG_PLUGIN3
				std::cerr << "State 3 in AssemblyStrategy::StateMachine" << std::endl;
#endif	 

				nextState	 = false;
				ctrlInitFlag = false;
			}

			// Set the desired force according to world coordinates. It will be modified within the control compositions to local coordinates.
			DesForce(UP_AXIS) = currForces(UP_AXIS);
			if( DesForce(UP_AXIS)<0 )
				DesForce(UP_AXIS) = CONST_FORCE_STATE3;

			// Moment Alignment
			ret=ControlCompositions(m_path,bodyPtr,JointAngleUpdate,CurrAngles,approach,MomentForceComposition,DesMoment,DesForce,DesIKin,ErrorNorm1,ErrorNorm2,pos,rot,cur_time,Jacobian,PseudoJacobian); 		// takes the DesIKin wrist position


			// Check for state termination. Also free allocated resources.
			StateSwitcher(approach,State,ErrorNorm1,ErrorNorm2,pos,rot,CurrAngles,avgSig,cur_time);
		}
		break;

		/*-------------------------------------------------- Compliant Insertion Controller --------------------------------------------------*/
		case paInsertion:
		{
			// Initialize
			if(ctrlInitFlag)
			{
#ifdef DEBUG_PLUGIN3
				std::cerr << "State 4 in AssemblyStrategy::StateMachine" << std::endl;
#endif	 

				nextState	 = false;
				ctrlInitFlag = false;
			}

			// Set the desired force according to world coordinates. It will be modified within the control compositions to local coordinates.
			DesForce(UP_AXIS)=VERTICAL_FORCE;			// PUSH DOWN
			//DesForce(FWD_AXIS)=5.0;				// PUSH AGAINST PIVOTDING DOCK
			DesMoment(1) = ROTATIONAL_FORCE;				// ROTATE TO CLOSE

			/******************************************** Call controller ********************************************/
			ControlCompositions(m_path,bodyPtr,JointAngleUpdate,CurrAngles,approach, MomentForceComposition,DesMoment,DesForce,DesIKin,ErrorNorm1,ErrorNorm2,pos,rot,cur_time,Jacobian,PseudoJacobian);

			// Check for state termination. Also free allocated resources.
			StateSwitcher(approach,State,ErrorNorm1,ErrorNorm2,pos,rot,CurrAngles,avgSig,cur_time);
		}
		break;

		/*--------------------------------------------------Mating --------------------------------------------------*/
		case paMating: // Maintain Position
		{
			// Change the paradigm to call directcompliancecontrol directly
			//controlType=motionData;

			// Save current wrist position to wrist_r, wrist_p, that's where we will direct the wrist.
			//wrist_p=pos;
			//wrist_p=rpyFromRot(arm_path->joint(6)->attitude()); // Used in linux simulation
			//wrist_p=rpyFromRot(arm_path->joint(6)->segmentAttitude());

			//  Compound Ikin controller: Compute the joint angles
			// //ComputeKinematics();

			// Compute the torques to keep the position
			// //DirectComplianceControl(0);

		}
		break;
		}
	}

	/*---------------------------------------------------------------------- SIDE APPROACH ------------------------------------------------------------------------*/
	else if(approach==SideApproach || approach==FailureCharacerization)
	{
		switch (State)
		{
		/*-------------------------------------------------- Approach --------------------------------------------------*/
		case hsaApproach:// Move, such that the TCP makes contact with the docking pivot in the male camera part at an angle.
		{
			// Initialize
			if(ctrlInitFlag)
			{
#ifdef DEBUG_PLUGIN3
				std::cerr << "State 1 in AssemblyStrategy::StateMachine" << std::endl;
#endif	 

				// In this first step ensure that the original end-effector position and orientation are set correctly
				EndEff_r_org = rpyFromRot(rot);
				wrist_r = EndEff_r_org;

				EndEff_p_org = pos;
				wrist_p = EndEff_p_org;

#ifdef DEBUG_PLUGIN3
				std::cerr << "/*******************************************************************************************\n"
						"The original position of the EndEffector is: " << wrist_p(0) << "\t" << wrist_p(1) << "\t" << wrist_p(2)
						<< "\t" << wrist_r(0) << "\t" << wrist_r(1) << "\t" << wrist_r(2) <<
						"\n*******************************************************************************************/" << std::endl;
#endif


				nextState	 = false;
				ctrlInitFlag = false;
			}

			// Primitive Ikin controller: Use a preplanned trajectory to move the wrist at an angle until contact.
			// DesIKin are hardcoded into the code.
			// Output is JointAngleUpdate
			ret=ControlCompositions(m_path,bodyPtr,JointAngleUpdate,CurrAngles,approach,IKinComposition,n,DesForce,n6,ErrorNorm1,ErrorNorm2,pos,rot,cur_time,Jacobian,PseudoJacobian);

			// 0) Check for state termination.
			StateSwitcher(approach,State,ErrorNorm1,ErrorNorm2,pos,rot,CurrAngles,avgSig,cur_time); // Fix to add ErrorNorm2
		}
		break;

		/*-------------------------------------------------- Rotation --------------------------------------------------*/
		case hsaRotation: // Rotate until contact
		{
			// Initialize
			if(ctrlInitFlag)
			{
#ifdef DEBUG_PLUGIN3
				std::cerr << "State 2 in AssemblyStrategy::StateMachine" << std::endl;
#endif	 

				nextState	 	= false;
				ctrlInitFlag 	= false;
				// Set up a new trajectory file with current information
				ProcessTrajFile(strTrajState2,State,pos,CurRPY,cur_time);
				EndEff_p_org = pos;
				EndEff_r_org = CurRPY;
			}

			// Primitive Ikin controller: Use a preplanned trajectory to rotate wrist until further contact
			//ret=ControlCompositions(m_path,bodyPtr,JointAngleUpdate,CurrAngles,approach, IKinComposition,n,DesForce,n6,ErrorNorm1,ErrorNorm2,pos,rot,cur_time,Jacobian,PseudoJacobian);

			// Or, use a ForceIKinController
			// Set a DesForce that pushes down and against the wall of the camera model
#ifdef SIMULATION
			/*------------------------ WORLD COORDINATES -------------------------------*/
			// +X: Downwards
			// +Y: Moves right
			// +Z: Moves forward (and a bit left)
			DesForce(UP_AXIS) 	=  1.000*VERTICAL_FORCE;
			//DesForce(SIDE_AXIS) = -1.000*HORIZONTAL_FORCE;
			DesForce(FWD_AXIS) 	=  -16.000*TRANSVERSE_FORCE;
			DesMoment(1) 		=  3.0*ROTATIONAL_FORCE;
#else
			DesForce(UP_AXIS) 	= 1.375*VERTICAL_FORCE;
			//DesForce(SIDE_AXIS) = HORIZONTAL_FORCE;
			DesForce(FWD_AXIS) 	= -20*TRANSVERSE_FORCE;
			DesMoment(1) 		= 2.100*ROTATIONAL_FORCE;
#endif

			ret=ControlCompositions(m_path,bodyPtr,JointAngleUpdate,CurrAngles,approach,ForceMomentComposition,DesForce,DesMoment,n6,ErrorNorm1,ErrorNorm2,pos,rot,cur_time,Jacobian,PseudoJacobian);

			// Check for state termination. Also free allocated resources.
			StateSwitcher(approach,State,ErrorNorm1,ErrorNorm2,pos,rot,CurrAngles,avgSig,cur_time);
		}
		break;


		/*-------------------------------------------------- Compliant Insertion Controller --------------------------------------------------*/
		case hsaInsertion:
		{
			// Initialize
			if(ctrlInitFlag)
			{
#ifdef DEBUG_PLUGIN3
				std::cerr << "State 3 in AssemblyStrategy::StateMachine" << std::endl;
#endif	 

				nextState	 = false;
				ctrlInitFlag = false;
			}
			/*------------------------ WORLD COORDINATES -------------------------------*/
			// Set a DesForce that pushes down stronger and still rotates.
			// +X: Downwards
			// +Y: Moves right
			// +Z: Moves forward (and a bit left)
#ifdef SIMULATION
			DesForce(UP_AXIS) 	=   1.300*VERTICAL_FORCE;
			DesForce(SIDE_AXIS) =   2.000*HORIZONTAL_FORCE;
			DesForce(FWD_AXIS) 	= -13.000*TRANSVERSE_FORCE;			// DesForce(FWD_AXIS)  =-13.0*TRANSVERSE_FORCE*( pow(CurrAngles(4),2)/pow(1.5708,2) ); 					// backwards pushing force that counters the forward motion cause by the forward rotation (jacobian effect)
			DesMoment(1) 		=   3.750*ROTATIONAL_FORCE;
#else
			DesForce(UP_AXIS) = VERTICAL_FORCE;
			// DesForce(SIDE_AXIS) = HORIZONTAL_FORCE;
			DesForce(FWD_AXIS) = -20*TRANSVERSE_FORCE;
			DesMoment(1) = 4*ROTATIONAL_FORCE;
#endif

			ret=ControlCompositions(m_path,bodyPtr,JointAngleUpdate,CurrAngles,approach,ForceMomentComposition,DesForce,DesMoment,n6,ErrorNorm1,ErrorNorm2,pos,rot,cur_time,Jacobian,PseudoJacobian);

			// Check for state termination. Also free allocated resources.
			StateSwitcher(approach,State,ErrorNorm1,ErrorNorm2,pos,rot,CurrAngles,avgSig,cur_time);
		}
		break;

		/*-------------------------------------------------- SubInsertionStage --------------------------------------------------*/
		// Introduced in 2013Aug to deal with lack of empalement at end. Contains the same information that was formally used in the hsaMating state.
		case hsaSubInsertion: // Only push down
		{
			// Initialize
			if(ctrlInitFlag)
			{
#ifdef DEBUG_PLUGIN3
				std::cerr << "State 3b: in AssemblyStrategy::StateMachine" << std::endl;
#endif	 

				nextState	 = false;
				ctrlInitFlag = false;
				//SA_S4_Height = pos(2);
			}

			// Set a DesForce that pushes down and against the wall of the camera model
			//--------------------------------------------------------------------------
			// Linux
			//DesForce(UP_AXIS)   =  1.25*VERTICAL_FORCE;	// In world coordinates
			//DesForce(FWD_AXIS)  = -6.0*TRANSVERSE_FORCE*((pos(2)-0.0728)/(SA_S4_Height-0.0728)); // same as above but weaker backwards pushing. the push decreases as the rotation decreases and the snaps push in.
			//DesForce(SIDE_AXIS) = HORIZONTAL_FORCE/10.0;

			// QNX
			//DesForce(UP_AXIS)   =  VERTICAL_FORCE;   // In world coordinates
			//DesForce(FWD_AXIS)  =  TRANSVERSE_FORCE; //

#ifdef SIMULATION
			DesForce(UP_AXIS)		=   1.5000*VERTICAL_FORCE;
			//DesForce(SIDE_AXIS) 	=   0.000*HORIZONTAL_FORCE;
			//DesForce(FWD_AXIS) 	= -13.000*TRANSVERSE_FORCE;			// DesForce(FWD_AXIS)  =-13.0*TRANSVERSE_FORCE*( pow(CurrAngles(4),2)/pow(1.5708,2) ); 					// backwards pushing force that counters the forward motion cause by the forward rotation (jacobian effect)

#else
			DesForce(UP_AXIS) = 3*VERTICAL_FORCE;
			DesForce(FWD_AXIS) = -20*TRANSVERSE_FORCE;
#endif

			//	    DesForce(SIDE_AXIS) = HORIZONTAL_FORCE;
			//--------------------------------------------------------------------------
			ret=ControlCompositions(m_path,bodyPtr,JointAngleUpdate,CurrAngles,approach,MomentForceComposition,DesMoment,DesForce,n6,ErrorNorm1,ErrorNorm2,pos,rot,cur_time,Jacobian,PseudoJacobian);

			// Check for Approach termination condition
			ret=StateSwitcher(approach,State,ErrorNorm1,ErrorNorm2,pos,rot,CurrAngles,avgSig,cur_time);

			break;

		}

		case hsaMating:
				{
					// 2013Aug. When mating takes place under current simulation circumstances, there is penetration from the male snap into the walls of the female snap.
					// So even when no force is applied, the simulation produces a jittery effect that is seen as very high noise and large force jumps in the plots.
					// This exponential decay for the force values registered by the FT sensor are an attempt to reproduce the decay that would take place upon proper mating.
					avgSig = avgSig * exp(-cur_time/2.5);
				}
		break;
		// case hsaFinsih
		}
	}
	else
		return -1;

	/***************************************************************** Write data to file ***********************************************************************************/
	// if state 1 or 2, return the endeffector (handPos) position, if state 3 or 4 return handpos<-wrist2endeffector

	if(DB_WRITE)
	{
		if(approach==SideApproach || approach==FailureCharacerization)
		{
			vector3 handRPY;
			vector3 handPos;
			vector3 tempForce;
			vector3 tempMoment;
			vector3 temp1,temp2;
			dvector6 transformedForce;

			/***************************** Force/Moment ********************************/
			// Copy forces in world coordinates to local variable
			for(int i=0;i<3;i++)
			{
				tempForce(i) = avgSig(i);
				tempMoment(i) = avgSig(i+3);
			}

			// Transform to local coordinates
			temp1 = rot*tempForce;
			temp2 = rot*tempMoment;

			// Copy to 6 dimensional vector
			for(int i=0;i<3;i++)
			{
				transformedForce(i) = temp1(i);
				transformedForce(i+3) = temp2(i);
			}

			/*****************************  Position transformations *************************/
			//	  	  if(State==1)
			//	    {
			for(int i=0;i<3; i++)
				handPos(i) = pos(i);
			handRPY = rpyFromRot(rot);
			//	    }
			/*	  	      else
	    	{
			  handPos = m_path->joint(5)->p;

			  //handRPY = rpyFromRot(m_path->joint(5)->attitude()); // Used in linux simulations
			  handRPY = rpyFromRot(m_path->joint(5)->segmentAttitude());
			  }*/

			WriteFiles(cur_time, 				// Current time
					   CurrAngles, 				// Current robot joint angles
					   JointAngleUpdate, 		// Changes in Joint Angles in the last iteration
					   handPos,					// Current end-effector Cartesian position
					   handRPY,					// Current end-effector RPY
					   transformedForce);		// Current end-effector force and moments.
		}

		// Timing
		if(DB_TIME)
		{
			// Get end time
			gettimeofday(&endTime,NULL);

			// Compute duration
			duration = (double)(endTime.tv_sec - startTime.tv_sec) * 1000.0; 	// Get from sec to msec
			duration += (double)(endTime.tv_usec - startTime.tv_usec) / 1000.0; 	// From usec to msec

			// Print out the duration of the function
			#ifdef DEBUG_PLUGIN3
				std::cerr << "Duration of AssemblyStrategy::StateMachine() is: " << duration << "ms." << std::endl;
			#endif
		}

		#ifdef DEBUG_PLUGIN3
			std::cerr << "Exiting AssemblyStrategy::StateMachine" << std::endl;
		#endif
	}
	return ret;
}

/************************************************* Control Basis Methods *********************************************************************/
// Drive the kind of Ctrl Strategy that you want to accomplish.
// A control policy is selected according to the ctrlComp type. One can choose from:
// 	- IKinComposition 		 - inverse kinematics controlled composition
//	- ForceComposition 		 - Use jacobian transpose control to drive force control
// 	- MomentForceComposition - Compound controller that optimizes moment controller over force control
// All desired forces and moments are first given in world coordinates and later changed to local wrist coordinates
//
// Inputs:
// 		JointPathPtr - object that contains information from shoulder to wrist. Can call kinematic routines.
// 		BodyPtr		 - object that contains information about the whole body of HIRO
// 		CtrlStrategy - Select between StraightLineApproach/PivotApproach and other future methods
// 		ctrlComp	 - the type of control composition to executed
// 		DesData1/2	 - ControlBasis operates on the basis of two maximum desired quantities at a time
// 		DesIkin  	 - Used in the IkinComposition
// 		ErrorNorm1/2 - returned by ControlBasis class for primtive/compound compositions respectively
// 		pos			 - incoming is the base-to-end effector position. when updated by the inverse kinematics it shifts to represent base-to-wrist position
//		rot 		 - rotation matrix with similar properties as pos
//		cur_time	 - approximate value of control sampling time
//
// Outputs:
// 	The JointAngleUpdate's are passed by reference as well as the updated CurrAngles.
//
// Notes:
// More compositions were originally developed in my original project working with the PA10 and using OpenHRP-3.1.0 and OpenHRP-3.1.2-Release.
// You can find more compositions there under the sample/controller/PA10Controller2a folder, they need to be slightly adapted to work in this code per the code below.
// Juan Rojas. April 2012.
/********************************************************************************************************************************************/
int AssemblyStrategy::ControlCompositions(JointPathPtr 	m_path,								// Contains kinematics calls for arm 6 DoF
					  BodyPtr 		bodyPtr,							// Contains angles/path for 15 DoF
					  dvector6& 	JointAngleUpdate,					// Joint angle update as a result of des angles - actual angles
					  dvector6& 	CurrAngles,							// Current joint angles
					  CtrlStrategy 	strat,								// Strategy type: pivot approach or straight line
					  ctrlComp 		type,								// Type of controller
					  vector3& 		DesData1,							// Dominant controller reference data
					  vector3& 		DesData2,							// Subordinate controller reference data
					  dvector6& 	DesIKin, 							// Optional arguments for subordinate controllers if they exist.
					  double& 		ErrorNorm1,							// Error norm for dominant controller
					  double& 		ErrorNorm2,							// Error norm for subordinate controller
					  vector3& 		pos,								// Endeffector position
					  matrix33&		rot,								// Endeffector rotation matrix
					  double 		cur_time,							// 6 DoF Jaocobian to the wrist
					  dmatrix 		Jacobian,
					  dmatrix 		PseudoJacobian)
{
  // Compute Timing
  timeval startTime, endTime;
  double duration = -1.0;
	
  gettimeofday(&startTime,NULL);
	
  // Local variables
  matrix33 attitude(0);
  Vector3 temp(0,0,0),     CurXYZ(0,0,0),         CurRPY(0,0,0),
    CurForce(0,0,0), DesWristForce(0,0,0),  CurWristForce(0,0,0),
    CurMoment(0,0,0),DesWristMoment(0,0,0), CurWristMoment(0,0,0);

  // Output
  JointAngleUpdate.resize(ARM_DOF);

  // Initialize
  attitude 	= rot;			// Holds the rotation transformation from base to wrist
  dmatrix Jac = Jacobian;

  switch(type)
    {
      /**************************************************** Primitive Ikin Controller -Linear Descent ********************************************/
    case IKinComposition:
      {
#ifdef DEBUG_PLUGIN3 
	std::cerr << "AssemblyStrategy::ControlCompositions - IKinComposition Entering." << std::endl;
#endif				
				
	// Local Params
	bool ret = false;
	IKinTestFlag = 1;																// Used to activate the computation of Inverse Kinematics

	// 1) Computed desired Cartesian point for wrist
	moveRobot(cur_time);															// Call moveRobot to generate new desired wrist positions and wrist rotations.
#ifdef DEBUG_PLUGIN3
	std::cerr << "AssemblyStrategy::moveRobot. Desired wrist positions are: " << EndEff_p(0) << "\t" << EndEff_p(1) << "\t" << EndEff_p(2) << "\t" << EndEff_r(0) << "\t" << EndEff_r(1) << "\t" << EndEff_r(2) << std::endl;
#endif	

	// 2) Compute the desired joint angle info through inverse Kinematics
	//    Convert the desired RPY into desired rotation
	matrix33 DesRot = rotFromRpy(wrist_r);
#ifdef USE_OPENRAVE_IK
	ret = IK_arm(m_path, bodyPtr->link(0), bodyPtr->link(1), wrist_p, DesRot);
#else
	ret=m_path->calcInverseKinematics(wrist_p,DesRot);							// Returns angles with indeces 0-5.
#endif
	if(!ret)
	  {
	    // Print result and Do nothing
		#ifdef DEBUG_PLUGIN3
			std::cerr << "\nAssemblyStraegy::The inverse kinematics calculation failed!!\n";
		#endif
	    return -1;
	  }

	// 3) Find the angle update. Done this way to be able to project null space projections with other combinations.
	for(int i=0;i<ARM_DOF;i++)
	  {
	    JointAngleUpdate(i) = m_path->joint(i)->q - CurrAngles(i);					// Subtract desired angles from actual.
	    CurrAngles(i) 		= m_path->joint(i)->q;									// Consider indeces 0-5
	  }

	// 4) Free allocated resources
	FreeResources(type);

	// 5) Reset the IkinFlag so that if other controllers call ComputeKinematics no Inverse Kinematics are called there.
	IKinTestFlag = 0;
			
	#ifdef DEBUG_PLUGIN3
		std::cerr << "AssemblyStrategy::ControlCompositions - IKinComposition Exiting." << std::endl;
	#endif
      }
      break;

      /**************************************************** Compound Force-Ikin Controller ********************************************/
      // DO NOT USE, UNSTABLE. THE PROJECTION IN THIS SCENARIO DOES NOT WORK.
      // IKIN-FORCE SLIGHTLY BETTER BUT NOT USABLE.
    case ForceIKinComposition:
	{
		// Local Params
		bool ret = false;
		IKinTestFlag = 1;												// Used to activate the computation of inverse kinematics
		dvector6 tempCurrAngles;
		dvector6 AngleUpdate2;
		dvector6 CompoundAngleUpdate;
		c1 = new ControlBasis();

		// A1) Get Desired and current Data
		// Retrieve current force data and desired force data
		if(DEBUG_AS)
			for(int i=0; i<3; i++) CurForce(i) = 0;  					// keep CurForce as zero to avoid noisy signals.
		else
			for(int i=0; i<3; i++)
			{
				CurForce(i)      	= avgSig(i);
				tempCurrAngles(i) 	= CurrAngles(i);
				tempCurrAngles(i+3) = CurrAngles(i+3);
				AngleUpdate2(i) 	= 0.0;
			 	AngleUpdate2(i+3) 	= 0.0;
			}

		// A2) Rotate frame of reference from the base to the wrist for the forces.
		DesWristForce = attitude*DesData1;							// Desired force
		CurWristForce = attitude*CurForce;							// Current Force

		// A4) Copy to class
		for(int i=0; i<3; i++)
		{
			c1->DesData(i) = DesWristForce(i);
			c1->CurData(i) = CurWristForce(i);
		}

		// For IKin compound controllers set NumCtrls=2, but call ComputePrimitiveController() on Force or Moment, and then separately call NullSpaceProjection and UpdateJointAngles
		// A5) Call Primitive controller
		NumCtlrs = 2;		// This number is necessary to prohibit the function from adding JointAngleUpdate to CurrAngles. We care about JointAngleUpdate
		c1->ComputePrimitiveController(JointAngleUpdate,NumCtlrs,c1->ForceCtrl,c1->DesData,c1->CurData,CurrAngles,Jac,1,ErrorNorm1);

		/**IKin Segment**/
		// B1) Computed desired Cartesian point
		moveRobot(cur_time);											// Call moveRobot to generate new desired wrist positions and wrist rotations.

		// B2) Compute the desired joint angle info through inverse kinematics
		//    Convert the desired RPY into desired rotation
		matrix33 DesRot = rotFromRpy(wrist_r);
		#ifdef USE_OPENRAVE_IK
			ret = IK_arm(m_path, bodyPtr->link(0), bodyPtr->link(1), wrist_p, DesRot);
		#else
			ret=m_path->calcInverseKinematics(wrist_p,DesRot);                                                      // Returns angles with indeces 0-5.
		#endif

			if(!ret)
		{
			// Print exception results and Do nothing
			cerr << "\n!!The inverse kinematics calculation failed!!\n";
			cerr << "Cur_time at IL failure: " << cur_time << std::endl;
			cerr << "Wrist: " << wrist_p(0) << " " << wrist_p(1) << " " << wrist_p(2) << " " << wrist_r(0) << " " << wrist_r(1) << " " << wrist_r(2) << std::endl;
			exit(1);
		}

		// B3) Find the angle update. Done this way to be able to project null space projections with other combinations.
		for(int i=0;i<ARM_DOF;i++)
		{
			AngleUpdate2(i) = m_path->joint(i)->q - CurrAngles(i);					// Desired Joint Angles - Current Angles
			CurrAngles(i) 	= m_path->joint(i)->q;									// Consider indeces 0-5
		}

		// C1) Now that we have two separate joint angle updates we need to project the vector update of the subordinate onto the nullspace of the dominant vector update
		c1->NullSpaceProjection(/*out*/CompoundAngleUpdate, /*in-dominant*/JointAngleUpdate,/*in-subordinate*/AngleUpdate2);

		// C2) Add the compound angle update to the current joint angles
		c1->UpdateJointAngles(CurrAngles,CompoundAngleUpdate);

		// C3) Save updated desired joint angles to the private members of the Joint class to be used next in the computation of the torque control law
		for(int i=0;i<ARM_DOF;i++) m_path->joint(i)->q=CurrAngles(i);

		// D1) Free allocated resources
		FreeResources(type);

		// D3) Reset the IkinFlag so that if other controllers call ComputeKinematics no inv. kins are called there
		IKinTestFlag = 0;
	}
	break;


      /************************************************** Primitive Force Controller ***************************************************/
    case ForceComposition:
      {
	// 1) Local Parameters
	NumCtlrs = 1;
	c1 = new ControlBasis();

	// Get Desired and current Data
	// 3a) Retrieve current force data and desired force data
	if(DEBUG_AS)
	  for(int i=0; i<3; i++) CurForce(i) = 0;  					// keep CurForce as zero to avoid noisy signals.
	else
	  for(int i=0; i<3; i++)
	    {
	      CurForce(i)      = avgSig(i);
	    }

	// 4) Rotate frame of reference from the base to the wrist for the forces.
	DesWristForce = attitude*DesData1;							// Desired force
	CurWristForce = attitude*CurForce;							// Current Force
#ifdef DEBUG_PLUGIN3
	std::cerr << "\nThe desired force in local coordinates:\t" << DesWristForce(0) << "\t" << DesWristForce(1) << "\t" << DesWristForce(2);
#endif
	// 5) Copy to class
	for(int i=0; i<3; i++)
	  {
	    c1->DesData(i) = DesWristForce(i);
	    c1->CurData(i) = CurWristForce(i);
	  }

	// 6) Call Primitive controller
	c1->ComputePrimitiveController(JointAngleUpdate,NumCtlrs,c1->ForceCtrl,c1->DesData,c1->CurData,CurrAngles,Jac,ErrorFlag,ErrorNorm1);
	ErrorFlag = c1->ErrorFlag;																											// Used to determine state transition

	// 7) Save updated desired joint angles to the private members of the Joint class to be used next in the computation of the torque control law
	for(int i=0;i<ARM_DOF;i++)
	  bodyPtr->joint(zerothJoint+i)->q=CurrAngles(i);

	// 8) Update position and rpy
	m_path->calcForwardKinematics();

	// Store updates in pos and rot
	pos = m_path->joint(5)->p;
	//rot = m_path->joint(5)->attitude(); // Used in Linux simulation
	rot = m_path->joint(5)->segmentAttitude();

	// 9) Free allocated resources
	FreeResources(type);
      }
      break;

      /************************************************** Compound Moment-Force Controller ***************************************************/
    case MomentForceComposition:
      {
	// 1) Local Params
	NumCtlrs = 2;

	c1 = new ControlBasis(momentGainFactor);							// Allocate memory for two primitive controllers. Argument is a moment gain factor
	c2 = new ControlBasis();											// Objects are deallocated in FreeResources()

	// Get Desired and current Data
	// 3a) Retrieve current force data and desired force data
	if(DEBUG_AS)
	  {
	    for(int i=0; i<3; i++) CurForce(i) = 0; 				   		// Zeroed out during tests for clear numbers
	  }

	else
	  {
	    for(int i=0; i<3; i++) CurForce(i)  = avgSig(i);				// Zeroed out during tests for clear numbers
	    for(int i=0; i<3; i++) CurMoment(i) = avgSig(i+3); 				// Zeroed out during tests for clear numbers
	  }

	// 4) Rotate desired and current data from the world coordinate frame of reference to the wrist frame of reference.

	DesWristMoment = attitude*DesData1;											// Moment
	CurWristMoment = attitude*CurMoment;

	DesWristForce  = attitude*DesData2;											// Force
	CurWristForce  = attitude*CurForce;
	
	#if WRITELOG
		for(int i=0;i<3;i++)
		{
			ostr_des << DesWristForce(i) << "\t";
			ostr_cur << CurWristForce(i) << "\t";
		}
		for(int i=0;i<3;i++)
		{
			ostr_des << DesWristMoment(i) << "\t";
			ostr_cur << CurWristMoment(i) << "\t";
		}

		ostr_des << std::endl;
		ostr_cur << std::endl;
	#endif

	#ifdef DEBUG_PLUGIN3
		std::cerr << "\nThe desired force in local coords:\t" << DesWristForce(0) << "\t" << DesWristForce(1) << "\t" << DesWristForce(2)  << "\t" << DesWristMoment(0) << "\t" << DesWristMoment(1) << "\t" << DesWristMoment(2);
	#endif
	// 5) Copy values to class objects
	for(int i=3; i<6; i++)												// Moment
	  {
	    c1->DesData(i) = DesWristMoment(i-3);
	    c1->CurData(i) = CurWristMoment(i-3);
	  }
	for(int i=0; i<3; i++)												// Force
	  {
	    c2->DesData(i) = DesWristForce(i);
	    c2->CurData(i) = CurWristForce(i);
	  }

	// 6) Call compound controller where c1 is the dominant controller and c2 is the subordinate controller
	c1->ComputeCompoundController(JointAngleUpdate,CurrAngles, NumCtlrs,
				      	  	  	  c1->MomentCtrl, c1->DesData, c1->CurData,
				      	  	  	  c2->ForceCtrl,c2->DesData, c2->CurData,
				      	  	  	  Jac, ErrorNorm2, ErrorNorm1);
	// 7) Copy error flagstate
	ErrorFlag = c1->ErrorFlag;

	// 8) Save updated desired joint angles to the private members of the Joint class to be used next in the computation of the torque control law
	for(int i=0;i<ARM_DOF;i++)
	  bodyPtr->joint(zerothJoint+i)->q=CurrAngles(i);

	// 9) Update position and rpy
	m_path->calcForwardKinematics();

	// Store updates in pos and rot
		pos = m_path->joint(5)->p;
	rot = m_path->joint(5)->segmentAttitude();

	// 10) Free allocated resources
	FreeResources(type);

      }
      break;

      /************************************************** Compound Force-Moment Controller ***************************************************/
    case ForceMomentComposition:
      {
	// 1) Local Params
	NumCtlrs = 2;

	c1 = new ControlBasis();											// Allocate memory for two primitive controllers. Argument is a moment gain factor
	c2 = new ControlBasis(momentGainFactor);							// Objects are deallocated in FreeResources()

	// Get Desired and current Data
	// 3a) Retrieve current force data and desired force data
	if(DEBUG_AS)
	  {
	    for(int i=0; i<3; i++) CurForce(i) = 0; 				   		// Zeroed out during tests for clear numbers
	  }

	else
	  {
	    for(int i=0; i<3; i++) CurForce(i)  = avgSig(i);				// Zeroed out during tests for clear numbers
	    for(int i=0; i<3; i++) CurMoment(i) = avgSig(i+3); 				// Zeroed out during tests for clear numbers
	  }

	
	// 4) Rotate desired and current data from the world coordinate frame of reference to the wrist frame of reference.

	DesWristForce  = attitude*DesData1;											// Force
	CurWristForce  = attitude*CurForce;

	DesWristMoment = attitude*DesData2;											// Moment
	CurWristMoment = attitude*CurMoment;

#if WRITELOG

	for(int i=0;i<3;i++){
	  ostr_des << DesWristForce(i) << "\t";
	  ostr_cur << CurWristForce(i) << "\t";
	}
	for(int i=0;i<3;i++){
	  ostr_des << DesWristMoment(i) << "\t";
	  ostr_cur << CurWristMoment(i) << "\t";
	}
	
	ostr_des << std::endl;
	ostr_cur << std::endl;
	
#endif

#ifdef DEBUG_PLUGIN3
	std::cerr << "\nThe desired force in local coords:\t" << DesWristMoment(0) << "\t" << DesWristMoment(1) << "\t" << DesWristMoment(2)  << "\t" << DesWristForce(0) << "\t" << DesWristForce(1) << "\t" << DesWristForce(2);
#endif
	// 5) Copy values to class objects
	for(int i=0; i<3; i++)												// Moment
	  {
	    c1->DesData(i) = DesWristForce(i);
	    c1->CurData(i) = CurWristForce(i);
	  }
	for(int i=3; i<6; i++)												// Force
	  {
	    c2->DesData(i) = DesWristMoment(i-3);
	    c2->CurData(i) = CurWristMoment(i-3);
	  }

	// 6) Call compound controller where c1 is the dominant controller and c2 is the subordinate controller
	c1->ComputeCompoundController(JointAngleUpdate,CurrAngles, NumCtlrs,
				      c1->ForceCtrl, c1->DesData, c1->CurData,
				      c2->MomentCtrl,c2->DesData, c2->CurData,
				      Jac, ErrorNorm2, ErrorNorm1);
	// 7) Copy error flag state
	ErrorFlag = c1->ErrorFlag;

	// 8) Save updated desired joint angles to the private members of the Joint class to be used next in the computation of the torque control law
	for(int i=0;i<ARM_DOF;i++)
	  bodyPtr->joint(zerothJoint+i)->q=CurrAngles(i);

	// 9) Update position and rpy
	m_path->calcForwardKinematics();

	// Store updates in pos and rot
	pos = m_path->joint(5)->p;
	// rot = m_path->joint(5)->attitude(); // Used in Linux for simulation
	rot = m_path->joint(5)->segmentAttitude();

	// 10) Free allocated resources
	FreeResources(type);

      }
      break;

    default:
      {
	// Exit strategy
	State=-1;
	return -1;
      }
      break;
    } // end switch
	
  // Timing
  // Get end time
  if(DB_TIME)
    {
      gettimeofday(&endTime,NULL);
	
      // Compute duration
      duration  = (double)(endTime.tv_sec - startTime.tv_sec)   * 1000.0; 	// Get from sec to msec
      duration += (double)(endTime.tv_usec - startTime.tv_usec) / 1000.0; 	// From usec to msec
	
      // Print out the duration of the function
      std::cerr << "Duration of AssemblyStrategy::ControlComposition() is: " << duration << "ms." << std::endl;	
    }
  return 0;
}

// Switches states based on transition conditions
int AssemblyStrategy::StateSwitcher(enum 		CtrlStrategy approach,
										int& 		State,
										double 		ErrorNorm1,
										double 		ErrorNorm2,
										vector3 	pos,
										matrix33 	rot,
										dvector6 	CurJointAngles,
										dvector6 	currForces,
										double 		cur_time)
{
  //        std::cerr << "pos:" << pos(2) << std::endl;
	if(approach==PivotApproach)
	{
		// Local Variables
		Vector3 position(0), attitude(0);
		position = pos;
		attitude = rpyFromRot(rot);

		// Motion params
		//int cPt = 0;
		//float afterContactDip 	= 0.004;
		//float sinkStep 			= afterContactDip;

		// Transition States
		switch(State)
		{
			// Transition from State 1 to State 2: linear descent to rotating motion
			// Large contact force in the x-direction (could also measure along all other axis)
			case paApproach2Rotation:
			{
				// Graph shows that contacts typically manage to exceed Threshold 9 N. Measure at 75% end of trajectory
				if( cur_time>(ex_time[0]*0.80) )
					if(avgSig(0)>9)				// Lateral contact for x-axis
					{
							NextStateActions(cur_time,hsaHIROTransitionExepction);
					}
			}
			break;

			// Transition from State 2 to 3: rotation to Alignment
			case paRotation2IAlignment:
			{
				// At the end of the rotation, the clearest impacts is discerned by My. Threshold 5 N-m.
				if( cur_time>(ex_time[1]*0.90) )
					if(avgSig(4)>0.60) // upon first contact it surpasses 0.5 but then descends for some time until another interior contact raies it past 0.75
						{
						NextStateActions(cur_time,hsaHIROTransitionExepction);
						}
			}
			break;

			// Transition from State 3 to 4: from alignment to compliant insertion
			case paAlignment2Insertion:
			{
				// Transition Condition: if it is aligned
				if(CurJointAngles(4)<0.1745) // if the wrist pitch joint angle is less than 10 degrees
				{
					NextStateActions(cur_time,hsaHIROTransitionExepction);
				}
			}
			break;

			// Transition from final insertion to stop/desnapping.
			case paInsertion2Mating:
			{

				// Finish when there is no more forward motion, measured at a given step size.
				float zPosDifference = 0;

				// Record the z-position
				state3_zPos = pos(2);

				// Record the difference
				zPosDifference = state3_zPos - state3_zPrevPos;

				// Assign current value to previous value
				state3_zPrevPos = state3_zPos;

				// Conditions: straight up attitude, no motion, and low moment.
				if(CurJointAngles(4)>1.57 && CurJointAngles(4)<1.5718)
					if(abs(zPosDifference)<=0.000001)		// in testing i realized that for each step the motion may be much smaller than 0.0001 giving a fall sense of finish.
						NextStateActions(cur_time,hsaHIROTransitionExepction);			// it could be done through a time-window
			}
			break;

			default:
				break;
		}
	}
	/*--------------------------------------------------------------------------------- SIDE APPROACH------------------------------------------------------------------------------------------------------------------------*/
	else if(approach==SideApproach || approach==FailureCharacerization)
	{
		#ifdef DEBUG_PLUGIN3
		  std::cerr << "State:::" << State  << std::endl;
		  std::cerr << "pos:" 	  << pos(2) << std::endl;
		#endif
		  // Local Variables
		  Vector3 position(0), attitude(0);
		  position = pos;
		  attitude = rpyFromRot(rot);

		  // Transition States
		  switch(State)
		  {
		  /*------------------------------------------------------------ Approach2Rotation Transition ------------------------------------------------------------------------------------------------------------------------*/
		  	  // Large contact force in the x-direction in local coordinates
			  case hsaApproach2Rotation:
			  {
				  // Graph shows that contacts typically manage to exceed Threshold 9 N. Measure at 75% end of trajectory
				  float endApproachTime=ex_time[1];						// Check the pivotApproachState1.dat carefully. If 1 line choose ex_time[0], if there are two lines choose ex_time[0]
				  if( cur_time>(endApproachTime*0.80) )					// Want to make sure we are near the region of contact before we start measuring.
					  if(avgSig(Fx)>HSA_App2Rot_Fx)						// Vertical Contact Force along X-Direction in local coordinates. // Lateral contact for x-axis
					  {
							  NextStateActions(cur_time,hsaHIROTransitionExepction);
					  }
			  }
			  break;

			  /*------------------------------------------------------------ Rotation2Insertion Transition ------------------------------------------------------------------------------------------------------------------------*/
			  case hsaRotation2Insertion:
			  {
				  // At the end of the rotation, check either wrist joint angle or My moment with threshold ~5 N-m.
				  float endApproachTime=ex_time[1];
				  #ifdef SIMULATION
					  if( cur_time > endApproachTime ) 					// Time-based Condition: Must be at least greater than the end of ApproachTime. Should be near rotation.
						  if(CurJointAngles(4)<0.44550) //5093)			// Joint-based Angle Condition: Can be coupled with the pitch angle in pivotApproachState1.dat. Notice that a flat horizontal maleCam corresponds to a pitch angle of -1.57 but a joint angle 4 of 0
							  //if(avgSig(My)>HSA_Rot2Ins_My) 			// Force-based Condition: Upon first contact it surpasses 0.5 but then descends for some time until another interior contact raies it past 0.75
						  {
							  NextStateActions(cur_time,hsaHIROTransitionExepction);
						  }
				  #else
					  if(CurJointAngles(4)<0.1919)
					  {
						  NextStateActions(cur_time,hsaHIROTransitionExepction);
					  }
				  #endif
			  }
			  break;

			  /*------------------------------------------------------------ Insertion2Mating Transition ------------------------------------------------------------------------------------------------------------------------*/
			  // Transition Insertion2InsertionPart2
			  /*-----Transition Insertion2Mating-----*/
			  case hsaInsertion2InsPartB:
			  {
				#ifdef SIMULATION
				  //if(CurJointAngles(My)<0.116877)			//for simulation
				  //if(CurJointAngles(4)<0.097688)
				  //if(CurJointAngles(4)<-0.264) 		// if the wrist pitch joint angle is less than 14 degrees
				  //if(CurJointAngles(4)<-0.104717333)
				  //if(CurJointAngles(4)<0.104717333) 	// if the wrist pitch joint angle is less than 6 degrees
				  if( attitude(1) < -1.55200 || CurJointAngles(My) < 0.3850 )			//For 2013July-Rojas, This is about 20 degrees.This angle is not very precise because it changes depending on the pose of the elbow. If the male part is closer to the robot, the more bending there will be. This is in local coordinates, we need the angles in world coordinates.
					  {
					  	  hsaHIROTransitionExepction = Ins2InsSubPart; 						// Used to tell NextStateActions not to insert an entry in the time transition vector.
						  NextStateActions(cur_time,hsaHIROTransitionExepction);
						  hsaHIROTransitionExepction = normal;
					  }
				#else
				  if(CurJointAngles(4)<0.1395)	//if(CurJointAngles(4)<0.1919)
				  {
					  NextStateActions(cur_time,hsaHIROTransitionExepction);
				  }
				#endif
			  }
			  break;

			  /*------------------------------------------------------------ Insertion2b2Mating Transition ------------------------------------------------------------------------------------------------------------------------*/
			  case hsaInsPartB2Mating:
			  {
				  // If the height of the wrist is less than 0.7290 we are finished
				  // if(pos(2)<0.07290)
				  // if(pos(2)<0.533) 	// for simulation
				  // if(pos(2) < 0.575)  // This number is relative and can change depending on where to put the object of interest
				  // 2013Aug. New parameters.
				  // if(attitude(1) < -1.5400 || CurJointAngles(My) < 0.3700)
				  // if( (attitude(1) < -1.5330 && currForces(4) > 1.2) || (CurJointAngles(My) < 0.3870 && currForces(4) > 1.2) )	Used with 2nd Order Filter of Cutoff Freq 0.1
				  if( (attitude(1) < -1.5330 && currForces(4) > 0.9) || (CurJointAngles(My) < 0.3870 && currForces(4) > 0.9) )		// Used with 2nd Order Filter of Cutoff Freq 0.095
				  {
					  NextStateActions(cur_time,	 			hsaHIROTransitionExepction);		// Enter a time entry for the mating state
					  hsaHIROTransitionExepction = DoNotIncreaseStateNum;
					  NextStateActions(cur_time+mating2EndTime,	hsaHIROTransitionExepction);		// Enter a hard-coded time entry for the end of the experiment one second later
					  hsaHIROTransitionExepction = normal;
					  return PA_FINISH;
				  }
			  }
			  break;

			  // default case
			  default:
				  break;

		  } // End Switch
    }		// End if==PivotApproach

  return 0;
}

/*----------------------------------------------------------------------------------------------------*/
// Increase the value of the state variable by one and reset nextState and ctrlInitFlags.
// Except for listed exceptions.
void AssemblyStrategy::NextStateActions(double cur_time, int hsaHIROTransitionExepction)
{
	nextState		= true;
	ctrlInitFlag	= true;

	// 1) If flag is normal, then increase state and write output to file.
	if(hsaHIROTransitionExepction==normal)
	{
		State++;

		// Enter the time at which the state changed. Start with State 0 moving at the home position, State 1 moving towards part...
		ostr_state << cur_time << std::endl;
	}

	// 2) If flag is Ins2InsSubPart, then increase state but do not write output to file.
	else if(hsaHIROTransitionExepction==Ins2InsSubPart)
		State++;

	//
	else if(hsaHIROTransitionExepction==DoNotIncreaseStateNum)
	{
		// Enter the time at which the state changed. Start with State 0 moving at the home position, State 1 moving towards part...
		ostr_state << cur_time << std::endl;
	}
}

//**********************************************************************************************************************
// moveRobot()
// This function uses the motion.dat file to create sub-waypoints for the motion trajectory. Output points are saved
// into the classes private member variables wrist_p and wrist_r. These in turns are used when calling AssemblyStrategy::
// ControlCompositions.IkinCompositions.
//
// An Inverse Kinematics Function is called (OpenRAVE or OpenHRP's IK lib) with wrist_p and wrist_r as desired quantities.
// There can be multiple way-points in the motion.dat file.
// i is used an index to indicated if we are still in the trajectory from:
// 		a) Origin to waypoint 1 (i=0),
// 		b) waypoint 1 to 2 (i<T), and
//		c) waypoint 2 to 3, etc.
// Note that the function coswt is used as a scaling function between sub-waypoints,
// The scaling function gradually changes from 0 to 1 as the steps in the simulation increase for each waypoint.
//**********************************************************************************************************************/
bool AssemblyStrategy::moveRobot(double cur_time)
{
  // Local variables and flags
  bool 	ret			= true;									// Return flag
  int  	i 			= 0;									// Counter
  int  	T 			= ex_time.size();						// Read the number of waypoints contained in motion.dat
  double 	m_pi 	= 3.141592;								// Pi
  double 	coswt	= 0.0;									// Scaling function between waypoints

  // Trajectory-stage Divider
  // If the time-stamp (from way-point file) is less than the accumulated time of our simulation, increase counter i.
  for(int j=0; j<T; j++)
  {
	  // Compare cur_time, which is the time recorded in code (every cycle of onExecute updates by 0.001) with the waypoint time slots (i.e. 4 secs, 7 secs, 10 secs).
	  if(ex_time[j] < cur_time)
		  i++;						// Increments when new time stage arrives but it is never equal to the last one.
	  // So, I can be 0,1, but it is not set to two.
  }

  // Trajectory-stages with/without noise
  // Create vector for current position/rotation of the wrist. Can also be computed for the gripper if we used the array: hand[2]

  //-------------------------------------------------------------------------------------------------------------------------------------------
  // Stage 1: first way point time greater than current time
  //-------------------------------------------------------------------------------------------------------------------------------------------
  if(i==0)
    {
      // Scaling function
      // Design a simple scaling function over time from 0secs to 1sec.
      // cur_time/ex_time is a ration that goes from 0 to 1, for each way point segment.
      coswt = 0.5*(1.0 - cos(m_pi*(cur_time/ex_time[i])) ); // coswt = cur_time/ex_time[i]; 	//

      // Compute the new END_EFFECTOR position.
      // Original_Position + (New_Position - Original_Position)*scaling function
      EndEff_p =  EndEff_p_org(0) + (x_pos[i]-EndEff_p_org(0)) * coswt,						// x_pos comes from the waypoints in setRobot()
    		  	  EndEff_p_org(1) + (y_pos[i]-EndEff_p_org(1)) * coswt,
    		  	  EndEff_p_org(2) + (z_pos[i]-EndEff_p_org(2)) * coswt;

      EndEff_r =  EndEff_r_org(0) + ( roll_angle[i]-EndEff_r_org(0)) * coswt,
    		  	  EndEff_r_org(1) + (pitch_angle[i]-EndEff_r_org(1)) * coswt,
				  EndEff_r_org(2) + (  yaw_angle[i]-EndEff_r_org(2)) * coswt;

      //		hand[0] = 	lhand_org + (l_hand[i] - lhand_org) * coswt;
      //		hand[1] = 	rhand_org + (r_hand[i] - rhand_org) * coswt;
    }
  //-------------------------------------------------------------------------------------------------------------------------------------------
  // Stage 2: second Waypoint
  //-------------------------------------------------------------------------------------------------------------------------------------------
  else if(i<T)
  {
	  coswt = coswt = 0.5*(1.0 - cos(m_pi*(cur_time-ex_time[i-1])/(ex_time[i]-ex_time[i-1])) ); 	// (cur_time-ex_time[i-1])/(ex_time[i]-ex_time[i-1]); // position + (current desired position-previous position)*scaling function.
	  EndEff_p = x_pos[i-1] + (x_pos[i]+divPoint(0)-x_pos[i-1]) * coswt,							// xpos is a 3x1. it stores data for a given waypoint step, 0, 1, or 2.
			  	 y_pos[i-1] + (y_pos[i]+divPoint(1)-y_pos[i-1]) * coswt,
			  	 z_pos[i-1] + (z_pos[i]+divPoint(2)-z_pos[i-1]) * coswt;

	  EndEff_r = roll_angle[i-1]  + (  roll_angle[i]+divPoint(3)-roll_angle[i-1])  * coswt,
			  	 pitch_angle[i-1] + ( pitch_angle[i]+divPoint(4)-pitch_angle[i-1]) * coswt,
			  	 yaw_angle[i-1]   + (   yaw_angle[i]+divPoint(5)-yaw_angle[i-1])   * coswt;
	  //		 hand[0] = 	l_hand[i-1] + (l_hand[i] - l_hand[i-1]) * coswt;
	  //		 hand[1] = 	r_hand[i-1] + (r_hand[i] - r_hand[i-1]) * coswt;
  }
  //-------------------------------------------------------------------------------------------------------------------------------------------
  // Stage 3: third Waypoint: assign the previous position, which is at the desired and final waypoint.
  //---------------	----------------------------------------------------------------------------------------------------------------------------
  else
    {
      EndEff_p = x_pos[i-1]			+divPoint(0), 		// The divPoint array was introduced to perform error characterization of failure case scenarios.
    		  	 y_pos[i-1]			+divPoint(1),
    		  	 z_pos[i-1]			+divPoint(2);
      EndEff_r = roll_angle[i-1]	+divPoint(3),
    		     pitch_angle[i-1]	+divPoint(4),
    		     yaw_angle[i-1]		+divPoint(5);
      //		hand[0] = 	l_hand[i-1];
      //		hand[1] = 	r_hand[i-1];
      ret = false;
    }

  // Transform End-Effector Cartesian Coordinates into Wrist Coordinates:
  // Here we will convert the end-effector points into wrist points and pass them to the IKs. We do this because we could not achieve accurate positions including end-effector transofrmations in the IKs.
  // Transform these points taking into account the end-effector to produce a new wrist position/orientation.
  EndEff2WristTrans(EndEff_p,EndEff_r,wrist_p, wrist_r);

  // Print to cerr
  //cerr << wrist_p(0) << " " << wrist_p(1) << " " << wrist_p(2) << " " << wrist_r(0) << "" << wrist_r(1) << " " << wrist_r(2) << std::endl;

  return ret;
}

/******************************************** Other Functions ********************************************************/
// manipulationTest()
// Designed to test the performance of force and moment controllers individually for each axis: +/-X, +/-Y, +/-Z
// Originally wanted to do a time based test, but I need to exit this class to go back to forceSensorPlugin_impl to move
// HIRO. So instead I must test by doing a particular motion x number of times and then switch.
/******************************************** Other Functions ********************************************************/
int ::AssemblyStrategy::manipulationTest(TestAxis		axis,
					 bool&			completionFlag,
					 JointPathPtr 	m_path,
					 BodyPtr 		bodyPtr,
					 dvector6& 		JointAngleUpdate,
					 dvector6& 		CurrAngles,
					 vector3& 		DesData,
					 vector3& 		DesData2,
					 dvector6& 		DesIkin,
					 vector3 		pos,
					 matrix33 		rot,
					 double 		cur_time,
					 dmatrix 		Jacobian,
					 dmatrix 		PseudoJacobian)
{
  // Local variables
  //	clock_t startTime;
  //	clock_t endTime;
  //	clock_t clockTicksTaken;

  ::AssemblyStrategy::ctrlComp type;
  //double timeInSeconds = 0.0,
  double ErrorNorm1 = 0.0, ErrorNorm2 = 0.0;

  // Constants
  double DesForce = 10;	// Newtons

  // Test force
  vector3 ForceData, MomentData;
  vector3 nill;

  // Initialize all force values
  for(int i=0;i<3;i++)
    {
      ForceData(i) =0;
      MomentData(i)=0;
      avgSig(i)    =0;	// Including the sensed force
      avgSig(i+3)  =0;
    }

  /*type = IKinComposition;
  ControlCompositions(m_path,bodyPtr,
		      JointAngleUpdate,CurrAngles,
		      PivotApproach,type,
		      MomentData,ForceData,DesIkin,
		      ErrorNorm1,ErrorNorm2,
		      pos,rot,cur_time,
		      Jacobian,PseudoJacobian);*/

  /*********************************** Define what axis you want to test *******************************/
  /********************** +Fx ***************************/
  if(axis == ::AssemblyStrategy::posFx)
    {
      ForceData(0)=DesForce;
      type = ForceComposition;
      std::cerr <<"\n\nTesting Pos Fx: " << testCounter << "\n";
    }

  /********************** -Fx ***************************/
  else if(axis == ::AssemblyStrategy::negFx)
    {
      ForceData(0)=-DesForce;
      type = ForceComposition;
      std::cerr <<"\n\nTesting Neg Fx: " << testCounter << "\n";
    }

  /********************** Fy ***************************/
  else if(axis == ::AssemblyStrategy::posFy)
    {
      ForceData(1)=DesForce;
      type = ForceComposition;
      std::cerr <<"\n\nTesting Pos Fy: " << testCounter << "\n";
    }

  /********************** -Fy ***************************/
  else if(axis == ::AssemblyStrategy::negFy)
    {
      ForceData(1)=-DesForce;
      type = ForceComposition;
      std::cerr <<"\n\nTesting Neg Fy: " << testCounter << "\n";
    }

  /********************** Fz ***************************/
  else if(axis == ::AssemblyStrategy::posFz)
    {
      ForceData(2)=DesForce;
      type = ForceComposition;
      std::cerr <<"\n\nTesting Pos Fz: " << testCounter << "\n";
    }

  /********************** -Fz ***************************/
  else if(axis == ::AssemblyStrategy::negFz)
    {
      ForceData(2)=-DesForce;
      type = ForceComposition;
      std::cerr <<"\n\nTesting Neg Fz: " << testCounter << "\n";
    }

  /********************** Mx ***************************/
  else if(axis == ::AssemblyStrategy::posMx)
    {
      MomentData(0)=DesForce;
      type = MomentForceComposition;
      std::cerr <<"\n\nTesting Pos Mx: " << testCounter << "\n";
    }

  /********************** -Mx ***************************/
  else if(axis == ::AssemblyStrategy::negMx)
    {
      MomentData(0)=-DesForce;
      type = MomentForceComposition;
      std::cerr <<"\n\nTesting Neg Mx: " << testCounter << "\n";
    }

  /********************** My ***************************/
  else if(axis == ::AssemblyStrategy::posMy)
    {
      MomentData(1)=DesForce;
      type = MomentForceComposition;
      std::cerr <<"\n\nTesting Pos My: " << testCounter << "\n";
    }

  /********************** -My ***************************/
  else if(axis == ::AssemblyStrategy::negMy)
    {
      MomentData(1)=-DesForce;
      type = MomentForceComposition;
      std::cerr <<"\n\nTesting Neg My: " << testCounter << "\n";
    }

  /********************** Mz ***************************/
  else if(axis == ::AssemblyStrategy::posMz)
    {
      MomentData(2)=DesForce;
      type = MomentForceComposition;
      std::cerr <<"\n\nTesting Pos Mz: " << testCounter << "\n";
    }

  /********************** -Mz ***************************/
  else if(axis == ::AssemblyStrategy::negMz)
    {
      MomentData(2)=-DesForce;
      type = MomentForceComposition;
      std::cerr <<"\n\nTesting Neg Mz: " << testCounter << "\n";
    }


  /********************** ALL ***************************/
  // Composition switching mechanism
  else if(axis == ::AssemblyStrategy::all)
    {
      if(compositionTypeTest==0)
	{
	  // Initialize
	  for(int i=0;i<3;i++)
	    {
	      ForceData(i) =0;
	      MomentData(i)=0;
	    }
	  ForceData(0)=DesForce;
	  type = ForceComposition;

	  if(initialFlag)
	    {
	      std::cerr <<"\nStarting the Force Composition Test!!\n";
	      initialFlag = false;
	    }
	}
      else if(compositionTypeTest==1)
	{
	  // Initialize
	  for(int i=0;i<3;i++)
	    {
	      ForceData(i) =0;
	      MomentData(i)=0;
	    }
	  MomentData(0)=DesForce/2.0;
	  type = MomentForceComposition;

	  if(initialFlag)
	    {
	      std::cerr <<"\nStarting the Moment Composition Test!!\n";
	      initialFlag = false;
	    }
	}
      else
	{
	  std::cerr << "\nThe AssemblyStrategy::manipulationTest() has been completed!!\n";
	  std::cerr << "Next iteration will restart the test if called.\n";
	  compositionTypeTest = -1;

	  return 1;
	}

      // Data switching mechanism
      if(DesForceSwitch==0) 									// X
	{
	  if(compositionTypeTest==0)
	    std::cerr <<"\n\nTesting Pos Fx: " << testCounter << "\n";
	  else
	    std::cerr <<"\n\nTesting Pos Mx: " << testCounter << "\n";
	  //sleep(1);
	}

      else if(DesForceSwitch==1)
	{
	  ForceData(0) = -1*DesForce;

	  if(compositionTypeTest==0)
	    std::cerr <<"\n\nTesting Neg Fx: " << testCounter << "\n";
	  else
	    std::cerr <<"\n\nTesting Neg Mx: " << testCounter << "\n";
	}

      else if(DesForceSwitch==2) 								// Y
	{
	  ForceData(0) = 0;
	  ForceData(1) = DesForce;

	  if(compositionTypeTest==0)
	    std::cerr <<"\n\nTesting Pos Fy: " << testCounter << "\n";
	  else
	    std::cerr <<"\n\nTesting Pos My: " << testCounter << "\n";
	}

      else if(DesForceSwitch==3)
	{
	  ForceData(1) = -1*DesForce;

	  if(compositionTypeTest==0)
	    std::cerr <<"\n\nTesting Neg Fy: " << testCounter << "\n";
	  else
	    std::cerr <<"\n\nTesting Neg My: " << testCounter << "\n";
	}

      else if(DesForceSwitch==4)									// Z
	{
	  ForceData(1) = 0;
	  ForceData(2) = DesForce;

	  if(compositionTypeTest==0)
	    std::cerr <<"\n\nTesting Pos Fz: " << testCounter << "\n";
	  else
	    std::cerr <<"\n\nTesting Pos Mz: " << testCounter << "\n";
	}

      else if(DesForceSwitch==5)
	{
	  ForceData(2) = -1*DesForce;
	  if(compositionTypeTest==0)
	    std::cerr <<"\n\nTesting Neg Fz: " << testCounter << "\n";
	  else
	    std::cerr <<"\n\nTesting Neg Mz: " << testCounter << "\n";
	}

      else
	ForceData(0) = ForceData(1) = ForceData(2) = 0.0;
    }

  // Do this
  //while(timeInSeconds<TIME_THRESHOLD)
  if(testCounter<500)
    {
      if(type == ForceComposition)
	{
	  ControlCompositions(m_path,bodyPtr,
			      JointAngleUpdate,CurrAngles,
			      PivotApproach,type,
			      ForceData,MomentData,DesIkin,
			      ErrorNorm1,ErrorNorm2,
			      pos,rot,cur_time,
			      Jacobian,PseudoJacobian);
	}
      else if(type == MomentForceComposition)
	{
	  ControlCompositions(m_path,bodyPtr,
			      JointAngleUpdate,CurrAngles,
			      PivotApproach,type,
			      MomentData,ForceData,DesIkin,
			      ErrorNorm1,ErrorNorm2,
			      pos,rot,cur_time,
			      Jacobian,PseudoJacobian);
	}


      // Increase counter
      testCounter++;

      // Perform time calculations //endTime = clock();clockTicksTaken = endTime - startTime;timeInSeconds = clockTicksTaken / (double) CLOCKS_PER_SEC;if(timeInSeconds==0.0) std::cerr << "AssemblyStrategy::ManipulationTest.clock() is not working properly";
    }
  else
    {
      // Reset the iteration test counter
      testCounter=0;
      completionFlag = true;

      // After reaching the last element
      if(DesForceSwitch==5)
	{
	  compositionTypeTest++;
	  DesForceSwitch = 0;
	  initialFlag = true;
	}
      else
	DesForceSwitch++;			// Switch desired force element
    }

  // Print joint angle information in degrees to cerr
  std::cerr <<  setprecision(3) << "\nThe update joint angles are: \t" << JointAngleUpdate(0)*RAD2DEG << "\t" << JointAngleUpdate(1)*RAD2DEG << "\t" << JointAngleUpdate(2)*RAD2DEG  << "\t" << JointAngleUpdate(3)*RAD2DEG  << "\t" << JointAngleUpdate(4)*RAD2DEG  << "\t" << JointAngleUpdate(5)*RAD2DEG;
  std::cerr <<  setprecision(3) << "\nThe current angles are: \t"      << CurrAngles(0)*RAD2DEG 		<< "\t" << CurrAngles(1)*RAD2DEG 	   << "\t" << CurrAngles(2)*RAD2DEG 		<< "\t" << CurrAngles(3)*RAD2DEG 		<< "\t" << CurrAngles(4)*RAD2DEG 		<< "\t" << CurrAngles(5)*RAD2DEG 		<< "\n";
  return 1;
}


/******************************************** Other Functions ********************************************************/
// Free allocated pointers
/******************************************** Other Functions ********************************************************/
void AssemblyStrategy::FreeResources(ctrlComp type)
{
  // Free allocated resources
  if(type==ForceComposition)
    delete c1;
  if(type==MomentForceComposition)
    delete c2;
  //	if(c3!=NULL)
  //		delete c3;
}

/******************************************************************************************************************/
// During our efforts to do a conventional transformation to from the wrist to the end-effector,
// for a reason that we could not understand the simulation was not moving accurately. So we decided instead
// to take the desired points from the motion file and then SUBTRACT the end-effector transform from those and
// then pass them to the Inverse Kinematics as wrist points.
// transWrist2CamXEff is set at the constructor
/******************************************************************************************************************/
int AssemblyStrategy::EndEff2WristTrans(/*in*/Vector3 EndEff_p, /*in*/ Vector3 EndEff_r, /*out*/Vector3& WristPos, /*out*/ Vector3& WristRot)
{
  // Local variable to avoid aliasing
  Vector3 tempPos(0,0,0);

  if(PA10)
    {
      // Transformation Variables form wrist to End-Effector using force sensor and male camera in right-handed world coordinate system: (+Z:Up,+Y:Right,+X:Forward)
      const double X_w2xeff =  0.0000;
      const double Y_w2xeff = -0.0340;
      const double Z_w2xeff =  0.1890;
      transWrist2CamXEff = X_w2xeff, Y_w2xeff, Z_w2xeff;
    }

  //	if(HIRO) // Right Arm  using force sensor and male camera in right-handed world coordinate system: (+Z:Up,+Y:Right,+X:Forward)
  //	{
  //		// Transformation Variables form wrist to End-Effector (at the home position??, should it be with the arm straight up. )
  //		const double X_w2xeff = -0.127;
  //		const double Y_w2xeff = -0.03;
  //		const double Z_w2xeff = -0.005;
  //		transWrist2CamXEff = X_w2xeff, Y_w2xeff, Z_w2xeff;
  //	}

  // Copy the wrist position
  tempPos = EndEff_p;
  WristRot = EndEff_r;

  // Compute the New wrist position and rotation
  // NewWristRot = wrist_r; // No change for this endeffector
  WristPos = tempPos - rotFromRpy(WristRot)*transWrist2CamXEff;

  return 1;
}

/********************************************************************************
 * wrist2EndEffXform()
 * Takes the base2wrist position and rotation and transforms them to base2endeff
 * position and orientation
 * transWrist2CamXEff is set at the constructor
 *********************************************************************************/
int AssemblyStrategy::wrist2EndEffTrans(/*in,out*/vector3& WristPos, /*in,out*/vector3& WristRot)
{
  // Copy the wrist position
  vector3 tempPos = WristPos;

  if(PA10)
    {
      // Transformation Variables form wrist to End-Effector
      const double X_w2xeff =  0.0000;
      const double Y_w2xeff = -0.0340;
      const double Z_w2xeff =  0.1890;
      transWrist2CamXEff = X_w2xeff, Y_w2xeff, Z_w2xeff;
    }

  WristPos = tempPos + rotFromRpy(WristRot)*transWrist2CamXEff;

  return 1;
}

/****************************************** File Methods *************************************/

/********************************************************************************************/
// Open files from which to read robot data
/********************************************************************************************/
void AssemblyStrategy::OpenFiles()
{

	#ifdef DEBUG_PLUGIN3
	  std::cerr << "\nAssemblyStrategy::OpenFiles - Entering" << std::endl;
	#endif

  /*************************************** Robot Joint Angles ***************************************/
  ostr_angles.open(strAngles);			// "Angles.dat");
  if (!ostr_angles.is_open())
    std::cerr << strAngles << " was not opened." << std::endl;

  /*************************************** EndEffector Cart Positions (World Coordinates) ***************************************/
  ostr_cartPos.open(strCartPos); 		// "CartPos.dat");
  //	#endif
  if (!ostr_cartPos.is_open())
    std::cerr << strCartPos << " was not  opened." << std::endl;

  /*************************************** State Time Data ***************************************/
  ostr_state.open(strState);			// "State.dat"); //
  if (!ostr_state.is_open())
    std::cerr << strState << " was not opened." << std::endl;

  // Insert a value of 0 as the starting time of the state vector
  ostr_state << "0.0" << endl;;

  /*************************************** Joint Torque Data in World Coordinates ***************************************/
  ostr_Forces.open(strForces);			// "Torques.dat");
  if (!ostr_Forces.is_open())
    std::cerr << strForces << " was not opened." << std::endl;

  /*************************************** Desired Joint Torque Data in Local Coordinates ***************************************/
  ostr_des.open("TorquesLocal.dat");
  if(!ostr_des.is_open())
    std::cerr << "TorquesLocal.dat was not opened." << std::endl;

  /*************************************** Actual Joint Torque Data in World Coordinates ***************************************/
  ostr_cur.open("AnglesLocal.dat");
  if(!ostr_cur.is_open())
    std::cerr << "AnglesLocal.dat was not opened." << std::endl;
  
#ifdef DEBUG_PLUGIN3
  std::cerr << "\nAssemblyStrategy::OpenFiles - Exiting" << std::endl;	
#endif
}

/********************************************************************************************/
// Closes the files for writing
/********************************************************************************************/
void AssemblyStrategy::CloseFiles()
{
  // Robot Joint Angles
  if(ostr_angles.is_open())
    ostr_angles.close();
  ostr_angles.clear();

  // Robot End-Effecter Cartesian Positions
  if(ostr_cartPos.is_open())
    ostr_cartPos.close();
  ostr_cartPos.clear();

  // Robot Wrist Forces
  if(ostr_Forces.is_open())
    ostr_Forces.close();
  ostr_Forces.clear();

  // Robot State
  if(ostr_state.is_open())
    ostr_state.close();
  ostr_state.clear();

  if(ostr_des.is_open())
    ostr_des.close();
  ostr_des.clear();

  if(ostr_cur.is_open())
    ostr_cur.close();
  ostr_cur.clear();

}

/*********************************************************************************************************************************************************/
// WriteFiles
// Writes to file the following pieces of data:
// 1) State
// 2) Reference angles
// 3) Joint Updates
// 4) Cartesian Position
// 5) CurrentForces
/*********************************************************************************************************************************************************/
int AssemblyStrategy::WriteFiles(double cur_time, dvector6 CurrAngles, dvector6 JointAngleUpdate, vector3 CurrPos, vector3 CurrRPY, dvector6 CurrForces)
{

  /*************************** Write DesiredAngles file ****************************************/
  ostr_angles << cur_time << "\t"; 
  for(int i=0; i<6; i++)
    ostr_angles << CurrAngles(i) << " \t"; // CurrAngles at this stage have been updated by the State Machine and reflect the desired angles.
  ostr_angles << std::endl;

  // Print to screen
  if(DB_PRINT)
    {
      std::cerr << "\nThe current state is: " << State;
      std::cerr << "\nThe time and reference angles are: \t" << cur_time << "\t\t" << CurrAngles(0)*RAD2DEG << "\t" << CurrAngles(1)*RAD2DEG << "\t" << CurrAngles(2)*RAD2DEG << "\t" << CurrAngles(3)*RAD2DEG << "\t" << CurrAngles(4)*RAD2DEG << "\t" << CurrAngles(5)*RAD2DEG;
      std::cerr << "\nThe time and Joint   Updates  are: \t" << cur_time << "\t\t" << JointAngleUpdate(0)*RAD2DEG << "\t" << JointAngleUpdate(1)*RAD2DEG << "\t" << JointAngleUpdate(2)*RAD2DEG << "\t" << JointAngleUpdate(3)*RAD2DEG << "\t" << JointAngleUpdate(4)*RAD2DEG << "\t" << JointAngleUpdate(5)*RAD2DEG;
    }
	
  /*************************** Write Desired Cartesian Positions file in two steps. pos/rpy ****************************************/
  ostr_cartPos << cur_time << "\t";
  for(int i=0; i<3; i++)
    ostr_cartPos << CurrPos(i) << " \t";
  for(int i=0; i<3; i++)
    ostr_cartPos << CurrRPY(i) << " \t";
  ostr_cartPos << std::endl;

  // Print to screen
  if(DB_PRINT)
    std::cerr << "\nThe time and Cartesian position is: \t" << cur_time << "\t\t" << CurrPos(0) << "\t" << CurrPos(1) << "\t" << CurrPos(2) << "\t" << CurrRPY(0) << "\t" << CurrRPY(1) << "\t" << CurrRPY(2);

  /*************************** Write to force file ****************************************/
  ostr_Forces << cur_time << "\t";
  for(int i=0; i<6; i++)
    ostr_Forces << CurrForces(i) << " \t";
  ostr_Forces << std::endl;

  // Print to screen
  if(DB_PRINT)
    std::cerr << "\nThe time and current forces are: \t" << cur_time << "\t\t" << CurrForces(0) << "\t" << CurrForces(1) << "\t" << CurrForces(2) << "\t" << CurrForces(3) << "\t" << CurrForces(4) << "\t" << CurrForces(5) << "\n\n";

  return 1;
}

//************************************************************************************************************
// ProcessTrajFile
// Has different functionality depending on whether the value of State is 1 or 2.
// If State is 1, we extract 1 time element, and 6 position elements for each line that exists in the file.
// 
// If the State is 2, we write 2 new lines that describe the trajectory for that file. 
//
// Inputs:
// path 	- desired trajectory file to read from. Used for state 1.
// State 	- int describing what state of the pivot approach we are in. Could be 1 or 2. 
// pos	 	- current position
// rpy 		- current rpy
// cur_time - current time. 
//
// Outputs:
// return 1 if successful.
//************************************************************************************************************
int AssemblyStrategy::ProcessTrajFile(char path[STR_LEN], int State, vector3 pos, vector3 rpy, double cur_time)
{

	#ifdef DEBUG_PLUGIN3
		std::cerr << "\nAssemblyStrategy::ProcessTrajFile - Entering" << std::endl;
	#endif

	// If we enter state 2: Used in PivotApproach with PA10 but not in SideApproach with HIRO
	if(State==2)
	{
		#ifdef DEBUG_PLUGIN3
				std::cerr << "AssemblyStrategy::ProcessTrajFile - Entering State 2" << std::endl;
		#endif

		// 1) Open the trajectory file where we will place time and position data for the second state
		ostr_TrajState2.open("./data/PivotApproach/pivotApproachState2.dat");
		if (!ostr_TrajState2.is_open())
			std::cerr << path << " not opened" << std::endl;

		// 2) Insert current data in the first line and the desired data in the second line
		float traj2_goaltime = cur_time + ROTATION_TIME_SLOW;	// Goal time is one second later

		// Insert a Start and End position for State 2.
		// The first line is the current position
		// The second line keeps the same pose, but sets the goal orientation to be parallel to the ground
		ostr_TrajState2 << cur_time 		<< "\t" << pos(0) << "\t" << pos(1) << "\t" << pos(2) << "\t" << rpy(0) << "\t" << rpy(1)  << "\t" << rpy(2) << "\n";
		ostr_TrajState2 << traj2_goaltime 	<< "\t" << pos(0) << "\t" << pos(1) << "\t" << pos(2) << "\t" << 0.00   << "\t" << -1.5708 << "\t" << -1.5708;

		// 3) Close file
		if(ostr_TrajState2.is_open())
			ostr_TrajState2.close();
		ostr_TrajState2.clear();

		// Clear data
		ex_time.clear();
		x_pos.clear();
		y_pos.clear();
		z_pos.clear();
		roll_angle.clear();
		pitch_angle.clear();
		yaw_angle.clear();
	}

	/*--------------------------------------------------------------------- State 1 ---------------------------------------------------------------------------*/
	// Open an input stream
	#ifdef DEBUG_PLUGIN3
		cerr << "AssemblyStrategy::processTrajFile() - state 1 processing" << std::endl;
	#endif

	//ifstream fp;
	struct stat st;
	if(stat(path, &st)==0)
	{
		#ifdef DEBUG_PLUGIN3
				cerr << "AssemblyStrategy::processTrajFile() - the file exists!!" << std::endl;
		#endif
	}

	// Use input file stream to open the pivotApproachState1.dat file
	ifstr_pivApproachState1.open(path);

	// Save data stored in motion.dat into <vector> of doubles for t,xyz,rpy
	double x;
	if( ifstr_pivApproachState1.is_open() )
	{
#ifdef DEBUG_PLUGIN3
		cerr << "The waypoint file was opened successfully." << std::endl;
#endif

		while( !ifstr_pivApproachState1.eof() )
		{
			// Read data into x, and then push it into the relevant vector.
			ifstr_pivApproachState1 >> x;
			if(ifstr_pivApproachState1.eof())
				break;
			ex_time.push_back(x);								// Waypoint time

			#ifdef DEBUG_PLUGIN3
				std::cerr << "Time: " << x << std::endl;
			#endif

			ifstr_pivApproachState1 >> x;  x_pos.push_back(x);								// Waypoint x-pos
			#ifdef DEBUG_PLUGIN3
				std::cerr << "x-pos: " << x << std::endl;
			#endif

			ifstr_pivApproachState1 >> x;  y_pos.push_back(x);								// Waypoint y-pos
			#ifdef DEBUG_PLUGIN3
				std::cerr << "y-pos: " << x << std::endl;
			#endif

			ifstr_pivApproachState1 >> x;  z_pos.push_back(x);								// Waypoint z-pos
			#ifdef DEBUG_PLUGIN3
				std::cerr << "z-pos: " << x << std::endl;
			#endif

			ifstr_pivApproachState1 >> x;  roll_angle.push_back(x);							// Waypoint roll angle
			#ifdef DEBUG_PLUGIN3
				std::cerr << "Roll " << x << std::endl;
			#endif

			ifstr_pivApproachState1 >> x;  pitch_angle.push_back(x);						// Waypoint pitch angle
			#ifdef DEBUG_PLUGIN3
				std::cerr << "Pitch " << x << std::endl;
			#endif

			ifstr_pivApproachState1 >> x;  yaw_angle.push_back(x);							// Waypoint yaw angle
			#ifdef DEBUG_PLUGIN3
				std::cerr << "Yaw " << x << std::endl;
			#endif
		}
	}
	else
		std::cerr << "The file was not opened successfully." << std::endl;

	#ifdef DEBUG_PLUGIN3
		std::cerr << "AssemblyStrategy::ProcessTrajFile - Exiting" << std::endl;
	#endif

	return 1;
}

bool AssemblyStrategy::IK_arm(JointPathPtr arm_path, Link *base, Link *waist, const vector3& p, const matrix33& R){

  double m_pi = 3.141592;
  double q_old[6];
  for(int i=0; i<6; i++)
    q_old[i] = arm_path->joint(i)->q;

  int n = arm_path->numJoints();

  vector3  bp;
  matrix33 bR;

  bp = trans(base->attitude())*(p - base->p);
  bR = trans(base->attitude())*R;

  void *ik_handle;
  bool (*ik_)(const IKReal*, const IKReal*, const IKReal*,std::vector<IKSolution>&);

  if(arm_path->joint(n-1)->name=="RARM_JOINT5"){
    ik_handle = dlopen("OpenRAVE/ikfast.HIRO_RARM.so", RTLD_LAZY);
    //waist->q = atan2(bp(1), bp(0)) + asin(0.145/sqrt(bp(0)*bp(0)+bp(1)*bp(1)));                                                                       
  }
  else if(arm_path->joint(n-1)->name=="LARM_JOINT5"){
    ik_handle = dlopen("OpenRAVE/ikfast.HIRO_LARM.so", RTLD_LAZY);
    //waist->q = atan2(bp(1), bp(0)) - asin(0.145/sqrt(bp(0)*bp(0)+bp(1)*bp(1)));                                                                       
  }

  if(!ik_handle) cerr << "loading error" << std::endl;

  matrix33 Rz = rotFromRpy(0,0,waist->q);

  vector3  p0;
  matrix33 R0;

  p0 = trans(Rz)*bp;
  R0 = trans(Rz)*bR;

  IKReal p_[3],R_[9];
  for(int i=0; i<3; i++){
    p_[i] = p0(i);
    for(int j=0; j<3; j++)
      R_[3*i+j] = R0(i,j);
  }

  ik_ = (bool (*)(const IKReal*, const IKReal*, const IKReal*, std::vector<IKSolution>&))dlsym(ik_handle, "ik");

  bool solved = true;

  vector<IKSolution> vsolutions;
  if(! ik_(p_, R_, NULL, vsolutions) ) solved = false;

  /*
    cerr << vsolutions.size() << std::endl;
    for(size_t i=0; i<vsolutions.size(); i++)
    cerr << "#" << i << " " << vsolutions[i].basesol[i].foffset << std::endl;
  */

  if(solved){
    vector<IKReal> sol(6);
    bool ret = false;
    for(std::size_t i = 0; i < vsolutions.size(); ++i){

      vector<IKReal> vsolfree(vsolutions[i].GetFree().size());
      vsolutions[i].GetSolution(&sol[0],vsolfree.size()>0?&vsolfree[0]:NULL);

      for( std::size_t j = 0; j < sol.size(); ++j)
	arm_path->joint(j)->q = sol[j];
      if(checkArmLimit(arm_path)){
	ret = true;
	break;
      }
    }

    if(!ret) return false;
  }

  while(arm_path->joint(5)->q < arm_path->joint(5)->llimit ) arm_path->joint(5)->q += 2*m_pi;
  while(arm_path->joint(5)->q > arm_path->joint(5)->ulimit ) arm_path->joint(5)->q -= 2*m_pi;

  while( arm_path->joint(5)->q - q_old[5] >  m_pi && arm_path->joint(5)->q-2*m_pi > arm_path->joint(5)->llimit)   arm_path->joint(5)->q -= 2*m_pi;
  while( arm_path->joint(5)->q - q_old[5] < -m_pi && arm_path->joint(5)->q+2*m_pi < arm_path->joint(5)->ulimit)   arm_path->joint(5)->q += 2*m_pi;

  if(!checkArmLimit(arm_path))
    return false;

  arm_path->calcForwardKinematics();

  return true;
}

bool  AssemblyStrategy::checkArmLimit(JointPathPtr arm_path)
{
  for(int i=0; i<arm_path->numJoints(); i++){

    if(arm_path->joint(i)->q < arm_path->joint(i)->llimit) return false;
    if(arm_path->joint(i)->q > arm_path->joint(i)->ulimit) return false;
  }
  return true;

}
