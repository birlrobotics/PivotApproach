/*
 * AssemblyStrategy.h
 *
 *  Created on: Mar 19, 2012
 *      Author: juan
 *
 *      This class was derived from the one found in /home/juan/openhrp/OpenHRP-3.1.0-Release/sample/controller/PA10Controller2a
 *      This class has abstracted all the code necessary to run the strategy based on the Control Basis approach.
 *      This class currently has not introduced the Straight Line Approach code.
 *
 */
#ifndef ASSEMBLYSTRATEGY_H_
#define ASSEMBLYSTRATEGY_H_
//---------------------------------------------------------------------------------------------------------------------------
#include "hrpModelHeaders.h"			// Dynamics Simulator Folder
//---------------------------------------------------------------------------------------------------------------------------
// TVMet Files
#include <tvmet/Matrix.h>
#include <tvmet/Vector.h>
#include <tvmet/VectorFunctions.h>
//---------------------------------------------------------------------------------------------------------------------------
// Boost Files
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
//---------------------------------------------------------------------------------------------------------------------------
// Supporting Libraries: ControlBasis, FilterTools, and OpenRave.
#include "ControlBasis.h"
#include "FilterTools.h"
#include "OpenRAVE/OpenRAVE_IK.h"
//---------------------------------------------------------------------------------------------------------------------------
// STL Libraries.
#include <iostream>
#include <math.h>
#include <fstream>
#include <sstream>
#include <stdio.h>			// string operations
#include <stdlib.h>		// posix
#include <string.h>
#include <string>
#include <time.h>
#include <sys/time.h>
//#include <sys/syspage.h>
#include <vector>
#include <dlfcn.h>
//---------------------------------------------------------------------------------------------------------------------------
// Namespaces
using namespace std;
using namespace OpenHRP;
using namespace tvmet;
using OpenHRP::BodyPtr;
//---------------------------------------------------------------------------------------------------------------------------
// Typedef's
typedef tvmet::Vector<double, 6> vector6;
//---------------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------------
// GLOBAL VARIABLES
//---------------------------------------------------------------------------------------------------------------------------
// String length
#define STR_LEN			    256
//---------------------------------------------------------------------------------------------------------------------------
// Degrees of Freedom for body and arm
#define ARM_DOF               		6			// Right and left arm only have 6 DOF.
#define ROBOT_DOF            		15			// HIRO has 15 DOF. 3 in torso/head and six in each arm.
//---------------------------------------------------------------------------------------------------------------------------
// Filter parameters
#define HISTBUFF_LENGTH 	 		12 			// This value determines the size of the possible number of samples to be included to perform a moving average for data signals.
//---------------------------------------------------------------------------------------------------------------------------
// FORCE CONSTANTS FOR CONTROL POLICY
#define CONST_FORCE_STATE3     		5.0			// All forces need to be in world coordinates. Used in Hiro Side Approach (HSA) execution.
#define VERTICAL_FORCE	      		10.0		// HSA: Used in state 3 and 4 of pivot approach/stage 3 of side approach. pos value pushes down. 10kg Force pointing downwards
#define TRANSVERSE_FORCE      		0.25		// HSA: Moves out of the screen. Towards the desired wall of the mold in the side approach
#define HORIZONTAL_FORCE      		0.30		// HSA: There is a natural push to the left by the robot and gravity. This compensates for that during the rotation.
//---------------------------------------------------------------------------------------------------------------------------
// ROTATIONAL Time and Force Variables
#define ROTATIONAL_FORCE     		20.0		// Used in state 3 of side approach to close camera mold with a snap. 20N-m
#define ROTATION_TIME_SLOW    		3.0			// Slow time for testing and faster time for optimized trajectory
//---------------------------------------------------------------------------------------------------------------------------
// CONTACT Transitional Parameters
#define HSA_App2Rot_Fx				9.0			// HSA: Transition condition between Approach and Rotation stages. Used in Fx = 9N
#define HSA_Rot2Ins_My				0.8			// HSA: Transition condition between Approach and Rotation stages. Used in Fx = 9N

//---------------------------------------------------------------------------------------------------------------------------
// Math variables
#define PI 		      				3.1416
#define RAD2DEG            			180.0/PI
//---------------------------------------------------------------------------------------------------------------------------

// Class Forwarding
class ControlBasis;							// Need this declaration in order for the AssemblyStrategy class to know the type of the Control Basis Class

class AssemblyStrategy {
 public:
  AssemblyStrategy();
  AssemblyStrategy(int NUM_q0, vector3 base2endEffectorPos, matrix33 base2endEffectorRot, vector3 ePh, double momentGainFac);
  virtual ~AssemblyStrategy();

  /************************************* Snap Assembly and Control Basis Members *************************************/

  /***************************************** Enumerations ********************************************************/
  // Control Paradigm
  enum ControlParadigm
  {
    motionData,
    controlBasis
  };

  // Controller Type Enumeration: What type of basis control are you working with?
  enum ControllerType
  {
    Null,
    PositionCtrl,
    ForceCtrl,
    MomentCtrl,
  };

  // Control Strategy: What kind of approach will you adopt to complete the snap assembly
  enum CtrlStrategy
  {
    ManipulationTest,
    StraightLineApproach,
    PivotApproach,
    SideApproach
  };

  enum TestAxis
  {
    posFx, negFx,
    posFy, negFy,
    posFz, negFz,
    posMx, negMx,
    posMy, negMy,
    posMz, negMz,
    all,none
  };

  // Different kind of control compositions
  enum ctrlComp
  {
    // Jacobian Position
    PositionComposition = 1,
    PositionOrientationComposition,
    PoseComposition,
    // Inv Kin
    IKinComposition,
    IKinForceComposition,
    IKinMomentComposition,
    // Force
    ForceComposition,
    ForceIKinComposition,
    ForceMomentComposition,
    FMPComposition,
    // Moment
    MomentComposition,
    MomentIKinComposition,
    MomentForceComposition,
    MomentForceIKinComposition,
    MFPComposition,
  };

  // Enumerate the six force-moment axes for easy indexing.
  enum forceAxes
  {
	  Fx, Fy, Fz, Mx, My, Mz,
  };

  // Transitions for the SideApproach using HIRO. To be used in the switch-case statements.
  enum HSATransitions
  {
	  Approach2Rotation = 1,
	  Rotation2Insertion,
	  Insertion2Mating,
  };

  // *** Strategy ***/
  int		approachType;				// Save the kind of approach.
  bool 		approachFlag;				// Used to switch values
  bool 		ctrlInitFlag;				// Ussed to determine if its our first time in a state

  // Pivot Approach
  bool 		noContact;					// Flag become true when after a contact, the private member m_ContactPt return to the zero value.

  // Manipulation Test
  int testCounter;
  int compositionTypeTest;
  int DesForceSwitch;
  bool initialFlag;
  bool completionFlag;

  // Transtion
  double  	signChanger;				// Changes the sign of data after a certain amount of time.
  bool 		switchFlag;					// Flag to be triggered to change signs
  bool 		nextState;					// Used to tell the state machine when to move to the next state
  int 		State;						// What state are we in
  float 		transitionTime;					// Time between a two state stransition
  bool 		transitionTimebool;				// Flag for transition time
  double		state3_zPos;					// Used to identify the position in the z-direction of the wrist
  double		state3_zPrevPos;				// Keeps the previous record
  double 		SA_S4_Height;

  // Control Basis Objects
  ControlBasis* c1;
  ControlBasis* c2;
  ControlBasis* c3;

  double momentGainFactor;

  // Control Paradigm
  int CONTROL_PARADIGM;

  // Controller
  int 	NumCtlrs;						// How many primitive controllers are being compounded in the CtrlBasis framework?
  bool 	ErrorFlag;						// If error is low, set to true.

  // Data vectors
  dvector6 DesIKin;						// Final Desired Position
  dvector6 CurCartPos;						// 6 element vector to hold xyz rpy
  dvector6 DesCartPos;
  dvector6 contactPos;						// Position versions for strategies that use inverse kinematics.
  dvector6 CurJointAngles;					// Save current joint angles
  dvector6 JointAngleUpdate;					// It's the output of the control basis computation (single or primitive controllers) used to update the joint angle position

  // Filter Objects
  FilterTools* ft;

  // Kinematics
  int IKinTestFlag;						// Used to indicate IKin to control basis
  int zerothJoint;						// Zero index
  vector3 transWrist2CamXEff;					// Translation from wrist to end-effector holding camera. TCP is on edge of camera
  vector3 ContactWristAngle;
  dmatrix Jacobian;

  // Motion
  vector3 		wrist_p, EndEff_p, EndEff_p_org;	// Original and updated position angles for the wrist
  vector3 		wrist_r, EndEff_r, EndEff_r_org;	// Original and updated rotational angles
  vector<double> 	ex_time, x_pos, y_pos, z_pos;		// Save robot position in vectors
  vector<double>	roll_angle, pitch_angle, yaw_angle;		// Save robot orientation in vectors
  double angle_o[7]; 						// rhand_org, lhand_org, hando[2],

  // Files
  ifstream ifstr_pivApproachState1;																			// Input Streams
  ofstream ostr_TrajState2, ostr_cartPos, ostr_angles, ostr_state, ostr_Forces, ostr_des, ostr_cur; 		// Output Streams
  char strTrajState1[STR_LEN], strTrajState2[STR_LEN], strCartPos[STR_LEN], strAngles[STR_LEN], strState[STR_LEN], strForces[STR_LEN];

  // Imported values
  double 		cur_time;												// Save current tiem
  dvector6 		avgSig;													// contains filtered/moving average result signal
  /**************************************************************************** Methods **********************************************************************************/

  int StateMachine(TestAxis 		axis,																		// Used exclusively for AssemblyStrategy::manipulatorTest
		   CtrlStrategy 	approach,																	// Straight Line Approach or Pivot Approach
		   JointPathPtr 	m_path,																		// Kinematics
		   BodyPtr 		bodyPtr,																	// Kinematics
		   double 		cur_time,																	// Simulation time
		   vector3& 		pos,																		// TCP position
		   matrix33& 		rot,																		// TCP rotation
		   dvector6 		currForces,																	//
		   dvector6& 		JointAngleUpdate,
		   dvector6& 		CurrAngles,
		   dmatrix 		Jacobian,
		   dmatrix 		PseudoJacobian);

  int StateSwitcher(CtrlStrategy 	approach,
		    int& 			State,
		    double 		ErrorNorm1,
		    double 		ErrorNorm2,
		    vector3 		pos,
		    matrix33 		rot,
		    dvector6 		CurJointAngles,
		    dvector6		currForces,
		    double 		cur_time);																			// Switches the states of the state machine according to approach.

  void NextStateActions(double cur_time);																				// Fixed number of operations to be done between switching of states

  // Call any combination of legal control compositions (upto three for now), and pass desired data.
  int ControlCompositions(JointPathPtr 	m_path,
			  BodyPtr 		bodyPtr,
			  dvector6& 		JointAngleUpdate,
			  dvector6& 		CurrAngles,
			  CtrlStrategy 	strat,
			  ctrlComp 		type,
			  vector3& 		DesData1,
			  vector3& 		DesData2,
			  dvector6& 		DesIKin, 															// Optional arguments for subordinate controllers if they exist.
			  double& 		ErrorNorm1,
			  double& 		ErrorNorm2,
			  vector3& 		pos,
			  matrix33& 		rot,
			  double 			cur_time,
			  dmatrix 		Jacobian,
			  dmatrix 		PseudoJacobian);

  // Motion
  bool moveRobot(double cur_time);
  int manipulationTest(TestAxis 		axis,
		       bool&			completionFlag,
		       JointPathPtr 	m_path,
		       BodyPtr 		bodyPtr,
		       dvector6& 		JointAngleUpdate,
		       dvector6& 		CurrAngles,
		       vector3& 		DesData1,
		       vector3& 		DesData2,
		       dvector6& 		DesIKin, 															// Optional arguments for subordinate controllers if they exist.
		       vector3 		pos,
		       matrix33 		rot,
		       double 		cur_time,
		       dmatrix 		Jacobian,
		       dmatrix 		PseudoJacobian);

  // Other
  int  Initialize(char TrajState1[STR_LEN], char TrajState2[STR_LEN], char Angles[STR_LEN], char Position[STR_LEN], char State[STR_LEN], char Forces[STR_LEN], vector3 pos, matrix33 rot, double CurAngles[15]);
  int  EndEff2WristTrans(/*in*/ Vector3 EndEff_p, /*in*/ Vector3 EndEff_r, /*out*/ Vector3& WristPos, /*out*/ Vector3& WristRot);
  int  wrist2EndEffTrans(/*in,out*/vector3& WristPos, /*in,out*/vector3& WristRot);
  void FreeResources(ctrlComp type);

  // Files
  void OpenFiles();
  void CloseFiles();
  int  WriteFiles(double cur_time, dvector6 CurrAngles, dvector6 JointAngleUpdate, vector3 CurrPos, vector3 CurrRPY, dvector6 CurrForces);
  int  ProcessTrajFile(char path[STR_LEN], int State, vector3 pos, vector3 rpy, double cur_time);
  bool checkArmLimit(JointPathPtr arm_path);
  bool IK_arm(JointPathPtr arm_path, Link *base, Link *waist, const vector3& p, const matrix33& R);

};

#endif /* ASSEMBLYSTRATEGY_H_ */
