// - Delete "setDirectTeachingAxis(::CORBA::ULong axisflag)". (09.05.13)
// - Delete "setInertiaDamping(::CORBA::ULong mx,::CORBA::ULong dx)", (09.05.13)
// - Add    "setRPWbySensor(::CORBA::ULong r,::CORBA::ULong p, ::CORBA::ULong w)", (09.05.13)
// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
// vi: ai : ts=4
#ifndef FORCE_SENSOR_PLUGIN_IMPL_H
#define FORCE_SENSOR_PLUGIN_IMPL_H
//----------------------------------------------------------------------------------------------------------------------------------
#include <cstddef>
#include <cstdlib>
#include <cmath>
#include <cerrno>
#include <fstream>
#include <string>
//----------------------------------------------------------------------------------------------------------------------------------
extern "C" {
#include <pthread.h>
}
#include "Plugin_impl.h"
#include "forceSensorPlugin.h"
//---------------------------------------------------------------------------------------------------------------------------------- 
// Force
//#include "nittaFS.h" 
extern "C" {
#include "ifs_com.h"    // Driver
#include "ifs.h"
}
//---------------------------------------------------------------------------------------------------------------------------------- 
#include "hiroArm.h"
//---------------------------------------------------------------------------------------------------------------------------------- 
//#include "hiroArmSub.h"
//#include "quaternionUtil.h"	        // TODO bad includes code - change
//----------------------------------------------------------------------------------------------------------------------------------
// Namespaces
using OpenHRP::dquaternion;
using OpenHRP::matrix33;
using OpenHRP::BodyPtr;
using OpenHRP::Link;
using OpenHRP::JointPathPtr;
using OpenHRP::DblArray9;
using OpenHRP::DblArray3;
//----------------------------------------------------------------------------------------------------------------------------------
// DEFS
#define ARM_DOF 6
//----------------------------------------------------------------------------------------------------------------------------------
// Globals
static const double	deg2radC  	= M_PI/180.0;
static const double	rad2degC	= 180.0/M_PI;
//----------------------------------------------------------------------------------------------------------------------------------

// Class
class forceSensorPlugin_impl : public plugin,
  virtual public POA_forceSensorPlugin,
  virtual public PortableServer::RefCountServantBase
{
 public:
  void test();

 private:
  double DT; // [sec]
  // hardware limits
  // 0: CCW, 1: CW, 2: V, 3: Acc, 4: Dcc
  //~ bool ikRet, ikRetL, ikRetR;
  //~ double ikGains[4], ikRot[4], ikTrans[4];
  float ang_limit[DOF][5];
  BodyPtr body;
		
  //-----------------------------------------------------------------------------------------------------------------------------------------------
  // n-yamanobe
  //-----------------------------------------------------------------------------------------------------------------------------------------------
  // Enumerations
  // Axis Selection
  enum 		// Used to indiate in which direction rotatations should take place.
  { X, Y,Z };

  // Control Methods for Hiro
  enum
  {
    NotControlled,
    GravityCompensation,
    ResetGravityCompensation,
    BirateralControl,
    DirectTeaching,
    ReadyPosition,
    PivotApproach
#ifdef IMPEDANCE
    , ImpedanceControl
#endif
  };

  // ControlFlags. Use enumerations to define the kind of Control Method to use.
  int controlmode_r, controlmode_nr, controlmode_pre;

  // External Classes                     	//nittaFS    *fs;					// Force Sensor Class
  //ifs        *fs;
  hiroArmMas *lArm;				// Left Master Arm Class
  hiroArmSla *rArm;				// Right Slave Arm Class

  // Gravity Compensation
  bool f_gravity_comp[2];

  // Position and Rotation variables
  vector3  Pb_mas,   Pb_sla;
  matrix33 Rb_mas,   Rb_sla;
  vector3  dP_sum,   dRPY_sum;
  vector3  GainP[3], GainR[3];

  // Force and Moment Variables
  vector3 F_ave[2], 	  M_ave[2];
  vector3 F_sd[2],	  M_sd[2];
  vector3 F_err_max[2], M_err_max[2];
  double F_zero_lim[2], M_zero_lim[2];

  // Digital State and HW Switches
  unsigned long DioState;
  ifstream ifs;
  ofstream ofs;
  dvector6 logdata[10000][2];
  int      num_logs;

  // Streams
  FILE *fv_L, *fv_R; 				// Left and Right arm velocities
  FILE *fp, *fv;

  // Flags
  int initControl; // used to look at the first iteration of control
  int testFlag;
  int num_test;

  // Pivot Approach
  double cur_time;
  vector3 CurXYZ, L_CurXYZ;
  matrix33 CurRot, L_CurRot;
  dvector6 CurrentForces, L_CurrentForces;
  dvector6 CurrentAngles, JointAngleUpdate, L_CurrentAngles, L_JointAngleUpdate;

  // Flags
  bool initFlag;

  // Log
  ofstream ostr_rstate, ostr_astate, ostr_force, ostr_worldforce;
  /********************************************************** Methods ************************************************************/
  void 	 init(void);
  void 	 readInitialFile(const char *filename);
  void 	 readGainFile(const char *filename);
  matrix33 get_rot33(int dir, double rad );

 public:
  forceSensorPlugin_impl(istringstream &strm);
  //forceSensorPlugin_impl();
  ~forceSensorPlugin_impl();

  /**************************************** CORBA Interface ************************************************************/
  bool setup(RobotState *rs,RobotState *mc);
  void control(RobotState *rs,RobotState *mc);
  bool cleanup(RobotState *rs,RobotState *mc);
};
#endif // FORCE_SENSOR_PLUGIN_IMPL_H
