/* ControlBasis.h
 *  Created on: Dec 1, 2011
 *      Author: juan */

#ifndef ControlBasis_H_
#define ControlBasis_H_

#include "hrpModelHeaders.h"	// Contains all the header files otherwise found in DynamicSimulator/server

// TVMet Files
#include <tvmet/Matrix.h>
#include <tvmet/Vector.h>
#include <tvmet/VectorFunctions.h>

// Boost files
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>

// Standard libraries
#include <vector>
#include <sstream>


// Namespaces
using namespace OpenHRP;
using namespace tvmet;
using namespace boost;
using namespace std;

// TypeDefs

// Debugging flags
// #define DEBUG 1			// Turn on if you want to disable debug printouts and maximize efficiency in boost libraries.

// Ublas
// Bounded 6 element or 7 element vectors and matrices
typedef boost::numeric::ublas::bounded_vector<double, 7> dvector7;								// Define a 7x1 vector used for joint angles of a 7 DoF robot
typedef boost::numeric::ublas::bounded_vector<double, 9> dvector9;
typedef boost::numeric::ublas::bounded_matrix<double, 7, 6, boost::numeric::ublas::column_major> dmatrix76;
typedef boost::numeric::ublas::bounded_matrix<double, 7, 7, boost::numeric::ublas::column_major> dmatrix77;

// tvmet
typedef tvmet::Vector<double, 3> Vector3;
#define vector3 Vector3

//class PA10Controller2a;
class ControlBasis {
 public:
  ControlBasis();
  ControlBasis(double factor);
  virtual ~ControlBasis();

 public:

  /*************************************************************** Enumerations ***************************************************************/
  // Controller Type Enumeration: What type of basis control are you working with?
  enum ControllerType {
    Null,
    PositionCtrl,
    PoseCtrl,
    ForceCtrl,
    MomentCtrl,
  };

  enum NumController {
    TWO=2,
    THREE_A,
    THREE_B,
  };

  /******************************** Control Strategy **************************************/
  // Control Strategy: What kind of approach will you adopt to complete the snap assembly
  enum CtrlStrategy {
    StraightLineApproach,
    PivotApproach
  };

  /*************************************************************** Data ***************************************************************/
  int 	flag;						// Used to tell the state machine when to move to the next state

  // Desired Data and Current Data
  dvector6 DesData;					// Desired position or force/moment data for a ctrl object
  dvector6 CurData;					// Current position or force/moment data for a ctrl object

  // Joint Angles
  dvector7 CurJointAngles;			// Current joint angles for PA10 with 7 DoF
  dvector7 PrevJointAngles;			// Previous robot's joint angles
  dvector7 JointAngleUpdate;			// The differential update for the joint angles

  dvector6 CurJointAngles6;			// Current joint angles for PA10 with 6 DoF
  dvector6 PrevJointAngles6;			// Previous robot's joint angles
  dvector6 JointAngleUpdate6;			// The differential update for the joint angles

  // Error
  dvector6 ErrorOutput;				// Error Vector
  double   ErrorNorm;					// Sum of the squared values of the error vector elements
  bool	 ErrorFlag;					// Indicates low error

  // Gains
  dvector6 PositionGain;				// Control Basis Gains
  dvector6 PoseGain;
  dvector6 ForceGain;
  dvector6 MomentGain;

  /*************************************Filter Members *****************************************/
  float 	weight;						// Weighting measure used for averaging filter
  bool 	updateFlag;					// Becomes false after the first time it is used

  // Filter
  bool 	dataHistFlag;				// Flag to initialize dataHist
  bool 	ctrlInitFlag;				// Flag to initialize ctrl primitives
  boost::numeric::ublas::bounded_matrix<double, 8, 3, boost::numeric::ublas::column_major>	 dataHist;					// Holds filter/general data history in a buffer. Matrix dims given in MovingAverage function.

  vector3 stateVar;
  /*************************************************************** Methods ***************************************************************/

  // Generate a vector update based on a dominant controller and a subordinate controller
  int ComputeCompoundController(	/*out*/  	dvector7& 			JointAngleUpdate,
					/*in*/  	dvector7& 			CurJointAngles,
					/*in*/		int 				NumCtlrs,
					/*in*/		ControllerType 		type1,
					/*in*/		dvector6&			DesData1,
					/*in*/		dvector6&			CurData1,
					/*in*/		ControllerType 		type2,
					/*in*/		dvector6&			DesData2,
					/*in*/		dvector6&			CurData2,
					/*in*/		dmatrix&			Jacobian,
					/*out*/		double&				ErrorNorm1,
					/*out*/		double&				ErrorNorm2);

  // Same as above for 6 DOF robot. Overload.
  int ComputeCompoundController(	/*out*/  	dvector6& 			JointAngleUpdate,
					/*in*/  	dvector6& 			CurJointAngles,
					/*in*/		int 				NumCtlrs,
					/*in*/		ControllerType 		type1,
					/*in*/		dvector6&			DesData1,
					/*in*/		dvector6&			CurData1,
					/*in*/		ControllerType 		type2,
					/*in*/		dvector6&			DesData2,
					/*in*/		dvector6&			CurData2,
					/*in*/		dmatrix&			Jacobian,
					/*out*/		double&				ErrorNorm1,
					/*out*/		double&				ErrorNorm2);

  /*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
  // Generate a vector update based on a single primitive controller of types position/force/moment
  int ComputePrimitiveController(/*out*/  dvector7& 			JointAngleUpdate,
				 /*in*/ 	int 				NumCtlrs,
				 /*in*/ 	ControllerType 		type,
				 /*in*/ 	dvector6& 			DesData,
				 /*in*/	dvector6&			CurData,
				 /*in,out*/dvector7& 			CurJointAngles,
				 /*in*/	dmatrix&			Jacobian,
				 /*in*/	int					ErrorFlag,
				 /*out*/	double&				ErrorNorm);

  // Same as above for 6 DOF robot. Overload.
  int ComputePrimitiveController(/*out*/  dvector6& 			JointAngleUpdate,
				 /*in*/ 	int 				NumCtlrs,
				 /*in*/ 	ControllerType 		type,
				 /*in*/ 	dvector6& 			DesData,
				 /*in*/	dvector6&			CurData,
				 /*in,out*/dvector6& 			CurJointAngles,
				 /*in*/	dmatrix&			Jacobian,
				 /*in*/	int					ErrorFlag,
				 /*out*/	double&				ErrorNorm);

  /*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
  // Find difference between desired/actual data and calls Jacobian Product
  int ComputeError(/*in*/dvector6& DesData, /*in*/dvector6& CurData, /*out*/dvector6& ErrorOutput, /*out*/double& ErrorNorm, /*in*/int flag);

  // Selects appropriate Jacobian product methods based on controller type
  int JacobianProduct(/*in*/	dmatrix& 		Jacobian,
		      /*in*/	ControllerType 	type,
		      /*in*/	dvector6& 		ErrorOutput,
		      /*out*/ dvector7& 		AngleUpdate);

  //  Same as above for 6 DOF robot. Overload.
  int JacobianProduct(/*in*/	dmatrix& 		Jacobian,
		      /*in*/	ControllerType 	type,
		      /*in*/	dvector6& 		ErrorOutput,
		      /*out*/ dvector6& 		AngleUpdate);

  /*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
  // Used in compound controller to project a vector update of subordinate controller unto the null space of the dominant vector update
  int NullSpaceProjection(/*out*/         dvector7& JointAngleUpdate,
			  /*in*/ const    dvector7& AngleUpdate1,
			  /*in*/ const    dvector7& AngleUpdate2);

  // Same as above for 6 DOF robot. Overload.
  int NullSpaceProjection(/*out*/         dvector6& JointAngleUpdate,
			  /*in*/ const    dvector6& AngleUpdate1,
			  /*in*/ const    dvector6& AngleUpdate2);

  /*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
  // Adds joint angle update to existing current angles
  int UpdateJointAngles(	/*in*/ dvector7& CurJointAngles,
				/*out*/ dvector7& JointAngleUpdate);

  // Same as above for 6 DOF robot. Overload.
  int UpdateJointAngles(	/*in*/ dvector6& CurJointAngles,
				/*out*/ dvector6& JointAngleUpdate);

  /*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
  // Sets gains for position, force, and moment control primitives
  void SetGains(double factor);
};
#endif /* ControlBasis_H_ */
