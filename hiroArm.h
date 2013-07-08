/**
 * @file   hiroArm.h
 * @brief  arm class for HIRO
 * @author Natsuki Yamanobe
 * @date   2011/02/16
 * @note
 */

#ifndef HIRO_ARM_H
#define HIRO_ARM_H

// Includes
#include <cstddef>
#include <cstdlib>
#include <cmath>
#include <cerrno>
//#include <sys/syspage.h>
#include <sys/time.h>
#include <fstream>
#include <iostream>
#include <string>
#include <string.h>
#include <stdio.h>
#include "tvmet3d.h"
//#--------------------------------------------------------------------------------------------------------------------#--------------------------------------------------------------------------------------------------------------------
// OpenHRP
// In Linux
//#include "/home/juan/openhrp/OpenHRP3.0-HIRO/DynamicsSimulator/server/Link.h"			
//#include "/home/juan/openhrp/OpenHRP3.0-HIRO/DynamicsSimulator.orig/server/LinkPath.h"		// Used to compute inverse kinematics and jacobian
//#include "/home/juan/openhrp/OpenHRP3.0-HIRO/DynamicsSimulator/server/Body.h"				// Used to retrieve the position/joint angles/rot matrix of the kinematic body

// In QNX
#include "bodyinfo.h"
#include "hrpModelHeaders.h"			// DynamicSimulator/server
//#include "/home/grxuser/src/OpenHRP-3.0/DynamicsSimulator/server/quaternion.h"   				// DynamicSimulator/server

//#include "/home/grxuser/src/OpenHRP-3.0/DynamicsSimulator/server/Link.h"					// Needed for full definitino of the LinkPath Class
//#include "/home/grxuser/src/OpenHRP-3.0/DynamicsSimulator/server/LinkTraverse.h"				// Used to compute inverse kinematics and jacobian. Need this file when running the QNX version
//#include "/home/grxuser/src/OpenHRP-3.0/DynamicsSimulator/server/Body.h"					// Used to retrieve the position/joint angles/rot matrix of the kinematic body

// TVMet Files
#include <tvmet/Matrix.h>
#include <tvmet/Vector.h>
#include <tvmet/VectorFunctions.h>

// Boost Files
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>

// tvmet
using namespace tvmet;
typedef tvmet::Vector<double, 6> vector6;
typedef tvmet::Vector<double, 3> Vector3;

typedef boost::numeric::ublas::bounded_vector<double, 7> dvector7;				 // Define a 7x1 vector used for joint angles of a 7 DoF robot
typedef boost::numeric::ublas::bounded_vector<double, 9> dvector9;
typedef boost::numeric::ublas::bounded_matrix<double, 7, 6, boost::numeric::ublas::column_major> dmatrix76;
typedef boost::numeric::ublas::bounded_matrix<double, 7, 7, boost::numeric::ublas::column_major> dmatrix77;


//#--------------------------------------------------------------------------------------------------------------------#--------------------------------------------------------------------------------------------------------------------



// Force library
//#include "nittaFS.h"
extern "C" {
#include "ifs_com.h"
}

// Pivot Approach / Control Basis Framework
#include "AssemblyStrategy.h"			     							// Pivot Approach Strategy. Control Basis is embeded in the approach.
extern "C"
{
#include <stdio.h>
}

/********************************** Namespaces *******************************************/
using OpenHRP::matrix33;
//using OpenHRP::vector3;
using OpenHRP::dvector6;
using OpenHRP::dvector3;
using OpenHRP::BodyPtr;
using OpenHRP::Link;
using OpenHRP::JointPathPtr;
using OpenHRP::DblArray9;
using OpenHRP::DblArray3;
/********************************* DEFINES *********************************************/
#define ARM_DOF   6
#define STR_LEN 256

//------------------------------------------------------------------------------------------------------------------------
class hiroArm
{
  friend class forceSensorPlugin_impl;

 public:

  /******************************* ENUMERATIONS **************************************/
  enum interpolation
  {
    CONSTANT_VELOCITY,
    QUINTIC_FUNCTION,
    // append new method hereinafter
    END_OF_INTERPOLATION
  };

  /****************************** PATH GENERATION PARAMETERS ***************************/
  struct path_params
  {
    interpolation method;						// Interpolation method. See enumeration above.
    double        	max_jvel;				// Joint velocity limit
    dvector6 		q_start;				// Starting joint angle configuration
    dvector6 		q_goal;					// Ending joint angle configuration
    dvector6 		distance;				// Distance measure
    double 		duration;				// Desired Duration
    dvector6 		jvel;					// Joint velocity
    //dvector6 duration;
    //double max_duration;
    //std::vector<object_state> route
  };

  /*****************************  Control Mode **************************************/
  enum cmode
  {
    NOCONTROL,
    VEL_CONTROL,
    IMP_CONTROL
  };

  enum
  {
    X,
    Y,
    Z
  };

  /********************************************** METHODS *********************************************/

  hiroArm(std::string  name_in, 							// String variable indicating right or left arm
	  BodyPtr      body_in, 					// Body (link objects) Pointer
	  unsigned int num_q0_in, 					// Number of Starting joint
	  double 		 period_in, 				// Sampling Time
	  float 		 ang_limits_in[6][5], 			// Angle Limits
	  vector3 	 ePh_in, 					// Tranlation from wrist to end effector
	  matrix33 	 eRh_in, 					// Rotation   from wrist to end effector
	  vector3 	 hPfs_in);					//

  virtual ~hiroArm();

  void init();
  int init(vector3 pos, matrix33 rot, double CurAngles[15]);	// Initialize
  virtual void savedata();									// Save Data

  // Data retrieval methods
  std::string get_name();

  // Cartesian position and orientation and Current Joint Angles
  void update_currposdata();							// Update the base2endeffector position
  void wrist2EndEffXform(/*in*/vector3 rPe, /*in*/matrix33 rRe, /*out*/vector3& rPh, /*out*/matrix33& rRh);
  void EndEff2wristXform(/*in*/vector3 rPh, /*in*/matrix33 rRh, /*out*/vector3& rPe);

  // Joint Position
  dvector6 get_qref();							// Get joint angle references
  dvector6 get_qcur();							// Get current joint angles

  // Forces
  //void set_FSptr(nittaFS *fs_in, int NO_fs_in);	                       // Set force sensor pointer from nittaFs clas
  virtual void update_currforcedata();
  virtual bool get_forces(vector3 &rFh_gc_out, 				                                // Force
			  vector3 &rMh_gc_out);		                                                                // Moment

  // Hand position
  void get_curr_handpos(vector3 &rPh_out, 					                // Get current base2endeffector position
			matrix33 &rRh_out);		                                                                // Rotation

  void get_ref_handpos(vector3 &rPh_ref_out, 					                // Get desired base2endeffector position
		       matrix33 &rRh_ref_out);		                                                                // Rotation

  void set_ref_handpos(vector3 rPh_ref_in, 					                // Set desired base2endeffector position
		       matrix33 rRh_ref_in);		                                                                // Rotation

  // Time
  unsigned long get_Iteration();
  void set_Iteration();

  // Control Methods
  //void set_sampling_period(double period_in);
  bool velocity_control(vector3 rdP, vector3 rW, bool f_new);
  bool impedance_control();
#ifdef IMPEDANCE
  bool impedance_control2();
#endif

  // Gravity Compensation
  void reset_gravity_comp();
  int gravity_comp();
  int moveto_q_goal(dvector6 q_goal_in);

  // Pivot Approach and Control Bais Approach
  int PivotApproach(double 	cur_time,
		    vector3 	pos,
		    matrix33 	rot,
		    dvector6 	currForces,
		    dvector6& JointAngleUpdate,
		    dvector6& CurrAngles);
  // Reset the original EndEff pos and rpy
  void set_OrgPosRot(vector3& pos, vector3& RPY);

  // Files
  char TrajState1[STR_LEN];
  char TrajState2[STR_LEN];
  char Angles[STR_LEN];
  char CartPos[STR_LEN];
  char Forces[STR_LEN];
  char State[STR_LEN];
  char manipTest[STR_LEN];

#ifdef SIMULATION
  double raw_forces[6];
#endif

  /*********************************************************** MEMBERS ******************************************************/
 protected:

  // Asembly Strategy - Pivot Approach object
  AssemblyStrategy* PA;						// Pivot Approach strategy pointer

  //static const int ARM_DOF = 6;
  cmode controlmode;						// Control Mode Flag

  std::string name;						// String name for right or left arm
  FILE *fp1,*fp2, *fp3;						// File streams

  BodyPtr      body;						// Body (link objects) pointer
  JointPathPtr m_path;						// Path variable of OpenHRP model
  //	OpenHRP::JointPath* path;

  // Angles
  dvector6 	 q;						// Joint angles(radians)
  float 		 ang_limits[ARM_DOF][5];		               	// Angle limits
  unsigned int NUM_q0;						// Number of start joint of arm

  // Force
  //nittaFS 	*fs;						// Force Sensor Pointer
  int 		NO_fs;						// number of force sensor (0 or 1)
  double 	DEL_T;						// sampling period
  double 	ikGains[4], ikRot[4], ikTrans[4];		// Gains

  // Forces
  vector3 fsF_raw, fsM_raw;				        // Raw force and moment values of FS
  vector3 rFfs_gc, rMfs_gc;				        // gravity compensated force-moment value of FS in robot coord for the arm
  vector3 rFh_gc, rMh_gc;						// gravity compensated force-moment value of FS in robot coord for the hand
  //vector3 fsF_offset, fsM_offset;		                // Initial force-monment offset of FS
  //vector3 fsF, fsM;						// Initialized force-moment value.

  /************************************  Position Parameters ******************************************************************/
  // r stands for the base of the robot
  // e stands for wrist
  // h stands for endeffector
  // fs stands for force sensor

  /*********** Arm's Current State ***************/
  vector3  ePh;								// Translation from wrist to end effector
  matrix33 eRh;								// Rotation from wrist to end effector
  vector3  hPfs;								// Translation from end effector to force sensor

  vector3  rPh;								// Translation from base to end effector
  matrix33 rRh;								// Rotation from base to end effector

  vector3  rPe;								// Translation from base to wrist
  matrix33 rRe;								// Rotation from base to wrist

  /*********** Arm's Reference State ***************/
  vector3  rPh_ref;							// Desired position (Translation from base to end effector)
  matrix33 rRh_ref;							// Desired Rotation (Rotation from base to end effector)

  vector3  rPe_ref;							// Desired position (Translation from base to wrist)
  matrix33 rRe_ref;							// Desired Rotation (Rotation from base to wrist)

  dvector6 q_ref;								// Desired joint angle(rad)

  // Arm motion
  bool f_moving, f_reached;					// Motion flag variables: is moving? has reached goal?
  unsigned long m_time;						// amount of time that motion has taken place
  path_params p_param;						// structure containing info about the motion composed of: interpolation method, Max. Angular Joint Vel, starting goal joint position, ending goal joint position, Distance, and duration.

  const double MAX_JVEL;					// Maximum limit for joint velocity, i.e. 0.5 (rad/s)??
  const double TIME_LIMIT;					// Time limit
  const double MIN_PERIOD;					// Min Period
  const double TOLERANCE;					// Tolerance

  // Impedance control
  matrix33 Mt, Mr, Dt, Dr;					//
  vector3 rXd,rdXd,rddXd;						// Desired pos, dpos, ddpos
  vector3 rRd,rdRd,rddRd;						// Desired rot, drot, ddrot

  // Gravity compensation
  bool		 f_gc, f_gc_init;				// f_gc: ; f_gc_init: have we initialize grav. comp. variables?
  int 		 step_gc, step_fs;				// step_gc: has gravity compensation been executed
  double 		 mass;
  const int 	 max_step_fs, wait_step;			// Max = 500, Wait=200
  const double GACC;						// Gravity acceleration (9.8 m/s^2)

  vector3  rG_vec;						// Unit gravity vector in global frame (robot frame)
  vector3  fsPgc;							// Centroid in hand frame
  vector3  fsF_offset, fsM_offset;				// Force offset in force sensor frame
  vector3  fsF_tmp[5], fsM_tmp[5];
  dvector6 q_gc_ref[5];
  matrix33 fsRr_gc[5];

  // Others
  vector3  rPh_initial;
  matrix33 rRh_initial;
  vector3  fsF_initial, fsM_initial;

  //#ifdef IMPEDANCE
  std::ifstream fin_lpos;						// left position data
  std::ifstream fin_rpos;						// right position data
  std::ifstream fin_lfc;						// left force data
  std::ifstream fin_rfc;						// right force data
  //#endif

  // Flags
  bool initFlag;

  /************************************************** Functions **********************************************/
  // Motion
  bool is_moving();
  bool is_reached();
  bool is_activated();

  // Path Functions
  bool init_path_params(dvector6 q_start_in, dvector6 q_goal_in);
  bool set_path_params(path_params param_in);
  bool set_path_method(interpolation method_in);
  bool set_path_max_jvel(double max_jvel_in);
  bool calc_duration_jvel(dvector6 distance_in, double max_jvel_in);
  bool calc_qref(vector3 rPh_ref_in, matrix33 rRh_ref_in, dvector6 &q_out);
  bool moveto(dvector6 q_start, dvector6 q_goal);

  // Gravity Compensation
  bool calc_gravity_param();
  bool calc_gravity_param_shimizu();

  // Force Computations
  bool get_raw_forces(double f_out[6]);
  bool calc_gc_forces(vector3 &rFfs_gc_out, vector3 &rMfs_gc_out);
  void calc_gc_forces_at_hand(vector3 &rFh_out, vector3 &rMh_out);
  virtual matrix33 calc_hRfs(dvector6 q_in);

  // Mathematical
  void SkewToVector(matrix33 skew, vector3 &vec);
  matrix33 get_rot33(int dir, double rad);
  void calc_rPe_rRe(vector3 rPh_in, matrix33 rRh_in, vector3 &rPe_out, matrix33 &rRe_out); //base2wrist position and rotation matrices
};

/*********************************************************************** ArmMas classes. Inherits from hiroArm ******************************************************************************************/
// Master Class. Associated with the Left Arm.
class hiroArmMas : public hiroArm
{
 public:
  hiroArmMas(std::string name_in, BodyPtr body_in, unsigned int num_q0_in, double period_in, float ang_limits_in[6][5], vector3 ePh_in, matrix33 eRh_in, vector3 hPfs_in);
  //~ void savedata();

 protected:
  matrix33 calc_hRfs(dvector6 q_in);


};

// Slave Class. Associated with the Right Arm. Follows the master arm.
class hiroArmSla : public hiroArm
{
 public:
  hiroArmSla(std::string name_in, BodyPtr body_in, unsigned int num_q0_in, double period_in, float ang_limits_in[6][5], vector3 ePh_in, matrix33 eRh_in, vector3 hPfs_in, matrix33 hRfs_in);
  //~ void update_currforcedata();
  //~ bool get_forces(vector3 &rF_gc_out, vector3 &rM_gc_out);
  //~ void savedata();

 protected:
  matrix33 hRfs;
  //~ vector3 rFh_gc, rMh_gc;

  matrix33 calc_hRfs(dvector6 q_in);
  //~ void calc_forces_at_hand(vector3 &rFh_out, vector3 &rMh_out);
};

#endif
