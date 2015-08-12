/*
 * ControlBasis.cpp
 *
 *  Created on: Dec 1, 2011
 *  Author: juan
 *
 *  In this program two math libraries are used: Tvmet and uBlass.
 *  The first one is used for 3D vectors and matrices. The second one is used for 6/7 element vectors and matrices.
 *
 *  Aliasing: tvmet cannot alias. It needs temp variables. uBlass can alias. And if no aliasing is used,
 *  the computation can be made more efficient by including noalias(var) on the left hand side variable.
 *
 *  Design:
 *  This class was designed such that each primitive controller in the control basis can be instantiated as an object.
 */
//#include "../PA10Controller2a/PA10Controller2a.h"
#include "ControlBasis.h"
// In Linux
//#include <cmath>										// used to check infinity or NaN.

// In QNX
#include <math.h>

// CONSTANTS
#define MAX_ERROR 5000

/** Design parameters **/								// The default position/pose jacobian control computation is the pseudoinverse.
#define PSEUDO_JAC_HRP			0						// If you want to use the pseudojacobian computed by the HRP classes.
#define POSITION_JAC_TRANS_CTRL 0 						// If want to use Jacobian Transpose Control instead of the pseudoinverse. Pseudoinverse works better for position control.
#define POSE_JAC_TRANS_CTRL     0
#define ERROR_TEST         		0 						// If want to test controller with fixed error

// Debugging
#define DEBUG 1 										// Print cerr statements
/************************************************ CONSTRUCTOR ************************************************/
ControlBasis::ControlBasis()
{
  // Initialize private members
  flag = 0;

  // Set Control Gains
  SetGains(1.0);

  // Initialize vectors
  for(int i=0;i<6;i++)
    {
      DesData(i) 		= (0);
      CurData(i) 		= (0);
      ErrorOutput(i) 	= (0);
    }

  for(int i=0;i<7;i++)
    {
      CurJointAngles(i) 	= (0);
      JointAngleUpdate(i) = (0);
    }

  // Other var's init
  ErrorNorm 		= 0.0;
  ErrorFlag		= false;

  /************************** Filter ***********************/
  // moving average
  weight 			= 1; //0.5;
  updateFlag 		= true;
  dataHistFlag	= true;
  ctrlInitFlag	= true;

  // low pass
  stateVar = 0,0,0;
}

// Overloaded constructor that changes the gains
ControlBasis::ControlBasis(double factor)
{
  // Initialize private members
  flag = 0;

  // Set Control Gains
  SetGains(factor);

  // Initialize vectors
  for(int i=0;i<6;i++)
    {
      DesData(i) 		= (0);
      CurData(i) 		= (0);
      ErrorOutput(i) 	= (0);
    }

  for(int i=0;i<7;i++)
    {
      CurJointAngles(i) 	= (0);
      JointAngleUpdate(i) 	= (0);
    }

  // Other var's init
  ErrorNorm 		= 0.0;
  ErrorFlag		= false;

  /************************** Filter ***********************/
  // moving average
  weight 			= 1; //0.5;
  updateFlag 		= true;
  dataHistFlag	= true;
  ctrlInitFlag	= true;

  // low pass
  stateVar = 0,0,0;
}

/************************************************ DESTRUCTOR ************************************************/
ControlBasis::~ControlBasis()
{
  for(int i=0;i<6;i++)
    {
      DesData(i) 		= (0);
      CurData(i) 		= (0);
      ErrorOutput(i) 	= (0);
    }

  for(int i=0;i<7;i++)
    {
      CurJointAngles(i) 	= (0);
      JointAngleUpdate(i) = (0);
    }

  for(int i=0;i<6;i++)
    {
      CurJointAngles6(i) 	= (0);
      JointAngleUpdate6(i) = (0);
    }

  // Other var's init
  flag			= 0;
  ErrorNorm 		= 0.0;
  ErrorFlag		= false;
}

/************************************************ DETERMINANT ************************************************/
//int determinant_sign(const hrp::ublas::permutation_matrix<std::size_t>& pm) //<std::size_t>
//{
//    int pm_sign=1;
//    std::size_t size = pm.size();
//    for (std::size_t i = 0; i < size; ++i)
//        if (i != pm(i))
//            pm_sign *= -1.0; // swap_rows would swap a pair of rows here, so we change sign
//    return pm_sign;
//}
////
//double determinant( hrp::ublas::matrix<double>& m )
//{
//    hrp::ublas::permutation_matrix<std::size_t> pm(m.size1());	// </std><std::size_t>
//    double det = 1.0;
//    if( hrp::ublas::lu_factorize(m,pm) )
//    {
//        det = 0.0;
//    }
//
//    else
//    {
//        for(int i = 0; i < m.size1(); i++)
//            det *= m(i,i); // multiply by elements on diagonal
//
//        det = det * determinant_sign( pm );
//    }
//    return det;
//}

/********************************************* COMPOUND CONTROLLER**********************************************
 ** Takes two controllers, the first one is a dominant controller and the second one is a subordinate controller.
 ** The joint angle update of the second one is projected onto the nullspace of the dominant controller ensuring
 ** that the goal of the dominant controller is reached while optimizing the goal of the second controller.
 **
 ** The projection is executed through means of the Moore-Penrose pseduoinverse.
 ***************************************************************************************************************/
// DesData values are dvectors that need to be resized based on type of the controller
int ControlBasis::ComputeCompoundController(/*out*/  	dvector7& 			JointAngleUpdate,
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
					    /*out*/		double&				ErrorNorm2)
{
  if(NumCtlrs==TWO)
    {
      // Check to ensure that all incoming signals are valid
      if(&DesData1==NULL || &DesData2==NULL || &CurData1==NULL || &CurData2==NULL)
	return -1;

      // Local variable declaration
      dvector7 AngleUpdate1;
      dvector7 AngleUpdate2;

      // Initialization
      for(int i=0;i<7;i++)
	{
	  AngleUpdate1(i) 	= 0;
	  AngleUpdate2(i) 	= 0;
	}

      // 1a. Compute the joint angle update for the subordinate primitive controller
      ComputePrimitiveController(AngleUpdate2, NumCtlrs, type2, DesData2, CurData2, CurJointAngles, Jacobian, 1, ErrorNorm1);
      // 1b. Compute the joint angle update for the dominant primitive controller
      ComputePrimitiveController(AngleUpdate1, NumCtlrs, type1, DesData1, CurData1, CurJointAngles, Jacobian, 2, ErrorNorm2);

      // 2. Project the subordinate controller's update unto the left null space of the
      // dominant controller to produce an optimized joint angle update
      NullSpaceProjection(JointAngleUpdate,AngleUpdate1,AngleUpdate2);

      // 3. Add the joint angle update to the current angular joint position of the robot
      UpdateJointAngles(CurJointAngles, JointAngleUpdate);

      return 0;
    }

  if(NumCtlrs==THREE_A)
    {
      // Check to ensure that all incoming signals are valid
      if(&DesData1==NULL || &DesData2==NULL || &CurData1==NULL || &CurData2==NULL)
	return -1;

      // Local variable declaration
      dvector7 AngleUpdate1;
      dvector7 AngleUpdate2;

      // Initialization
      for(int i=0;i<7;i++)
	{
	  AngleUpdate1(i) 	= 0;
	  AngleUpdate2(i) 	= 0;
	}

      // 1a. Compute the joint angle update for the subordinate primitive controller
      ComputePrimitiveController(AngleUpdate2, NumCtlrs, type2, DesData2, CurData2, CurJointAngles, Jacobian, 1, ErrorNorm1);
      // 1b. Compute the joint angle update for the dominant primitive controller
      ComputePrimitiveController(AngleUpdate1, NumCtlrs, type1, DesData1, CurData1, CurJointAngles, Jacobian, 2, ErrorNorm2);

      // 2. Project the subordinate controller's update unto the left null space of the
      // dominant controller to produce an optimized joint angle update
      NullSpaceProjection(JointAngleUpdate,AngleUpdate1,AngleUpdate2);


      return 0;
    }

  // In this case,
  else if(NumCtlrs==THREE_B)
    {
      // Check to ensure that all incoming signals are valid
      if(&DesData1==NULL || &CurData1==NULL)
	return -1;

      // Local variable declaration
      dvector7 AngleUpdate1;
      dvector7 AngleUpdate2;

      // Initialization. Copy previous Joint Angle Update as AngleUpdate2.
      for(int i=0;i<7;i++)
	{
	  AngleUpdate1(i) 	= 0;
	  AngleUpdate2(i) 	= JointAngleUpdate(i);
	}


      // 1a. Compute the joint angle update for the third and most dominant primitive controller
      ComputePrimitiveController(AngleUpdate1, NumCtlrs, type1, DesData1, CurData1, CurJointAngles, Jacobian, 3, ErrorNorm1);

      // 2. Project the joint angle update produced by the 2nd/3rd primitive controllers unto the nullspace of dominant controller
      NullSpaceProjection(JointAngleUpdate,AngleUpdate1,AngleUpdate2);

      // 3. Add the joint angle update to the current angular joint position of the robot
      UpdateJointAngles(CurJointAngles, JointAngleUpdate);

      return 0;
    }

  else
    return -1;
}

/********************************************* COMPOUND CONTROLLER**********************************************
 ** Same as above but for 6DOF robot
 ***************************************************************************************************************/
// DesData values are dvectors that need to be resized based on type of the controller
int ControlBasis::ComputeCompoundController(/*out*/  	dvector6& 			JointAngleUpdate,
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
					    /*out*/		double&				ErrorNorm2)
{
  if(NumCtlrs==TWO)
    {
      // Check to ensure that all incoming signals are valid
      if(&DesData1==NULL || &DesData2==NULL || &CurData1==NULL || &CurData2==NULL)
	return -1;

      // Local variable declaration
      dvector6 AngleUpdate1;
      dvector6 AngleUpdate2;

      // Initialization
      for(int i=0;i<6;i++)
	{
	  AngleUpdate1(i) 	= 0;
	  AngleUpdate2(i) 	= 0;
	}

#ifdef DEBUG_PLUGIN3
      // Print data
      std::cerr<< "ComputeCompoundController. Dominant desired data is:\t" 		<< DesData1 << std::endl;
      std::cerr<< "ComputeCompoundController. Subordinate desired data is:\t" 	<< DesData2 << std::endl;
#endif
      
      // 1a. Compute the joint angle update for the subordinate primitive controller
      ComputePrimitiveController(AngleUpdate2, NumCtlrs, type2, DesData2, CurData2, CurJointAngles, Jacobian, 1, ErrorNorm2);
      if(DEBUG) std::cerr << "Subordinate Controller AngleUpdate2: " << AngleUpdate2 << std::endl;

      // 1b. Compute the joint angle update for the dominant primitive controller
      ComputePrimitiveController(AngleUpdate1, NumCtlrs, type1, DesData1, CurData1, CurJointAngles, Jacobian, 2, ErrorNorm1);
      if(DEBUG) std::cerr << "Dominant Controller AngleUpdate1:" << AngleUpdate1 << std::endl;

      // 2. Project the subordinate controller's update unto the left null space of the
      // dominant controller to produce an optimized joint angle update
      NullSpaceProjection(JointAngleUpdate,AngleUpdate1,AngleUpdate2);
      if(DEBUG) std::cerr << "Compound Joint Angle Update:\t" << JointAngleUpdate << std::endl;

      // 3. Add the joint angle update to the current angular joint position of the robot
      UpdateJointAngles(CurJointAngles, JointAngleUpdate);

      return 0;
    }

  if(NumCtlrs==THREE_A)
    {
      // Check to ensure that all incoming signals are valid
      if(&DesData1==NULL || &DesData2==NULL || &CurData1==NULL || &CurData2==NULL)
	return -1;

      // Local variable declaration
      dvector6 AngleUpdate1;
      dvector6 AngleUpdate2;

      // Initialization
      for(int i=0;i<6;i++)
	{
	  AngleUpdate1(i) 	= 0;
	  AngleUpdate2(i) 	= 0;
	}

      // 1a. Compute the joint angle update for the subordinate primitive controller
      ComputePrimitiveController(AngleUpdate2, NumCtlrs, type2, DesData2, CurData2, CurJointAngles, Jacobian, 1, ErrorNorm2);
      // 1b. Compute the joint angle update for the dominant primitive controller
      ComputePrimitiveController(AngleUpdate1, NumCtlrs, type1, DesData1, CurData1, CurJointAngles, Jacobian, 2, ErrorNorm1);

      // 2. Project the subordinate controller's update unto the left null space of the
      // dominant controller to produce an optimized joint angle update
      NullSpaceProjection(JointAngleUpdate,AngleUpdate1,AngleUpdate2);


      return 0;
    }

  // In this case,
  else if(NumCtlrs==THREE_B)
    {
      // Check to ensure that all incoming signals are valid
      if(&DesData1==NULL || &CurData1==NULL)
	return -1;

      // Local variable declaration
      dvector6 AngleUpdate1;
      dvector6 AngleUpdate2;

      // Initialization. Copy previous Joint Angle Update as AngleUpdate2.
      for(int i=0;i<6;i++)
	{
	  AngleUpdate1(i) 	= 0;
	  AngleUpdate2(i) 	= JointAngleUpdate(i);
	}


      // 1a. Compute the joint angle update for the third and most dominant primitive controller
      ComputePrimitiveController(AngleUpdate1, NumCtlrs, type1, DesData1, CurData1, CurJointAngles, Jacobian, 3, ErrorNorm1);

      // 2. Project the joint angle update produced by the 2nd/3rd primitive controllers unto the nullspace of dominant controller
      NullSpaceProjection(JointAngleUpdate,AngleUpdate1,AngleUpdate2);

      // 3. Add the joint angle update to the current angular joint position of the robot
      UpdateJointAngles(CurJointAngles, JointAngleUpdate);

      return 0;
    }

  else
    return -1;
}

/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

// Retrieves info from pa10
// if only one controller adds joint angle update to current joint angles
int ControlBasis::ComputePrimitiveController(	/*out*/ 	dvector7& 			JointAngleUpdate,
						/*in*/ 		int 				NumCtlrs,
						/*in*/ 		ControllerType 		type,
						/*in*/ 		dvector6& 			DesData,
						/*in*/		dvector6&			CurData,
						/*in,out*/	dvector7& 			CurJointAngles,
						/*in*/		dmatrix&			Jacobian,
						/*in*/		int					ErrorFlag,
						/*out*/		double&				ErrorNorm)
{
  // Check to ensure that all incoming signals are valid
  if(&DesData==NULL || &CurData==NULL)
    return -1;

  // Local Variable declaration and initialization
  dvector6 temp; for(int i=0;i<6;i++) temp(i)=0;

  // 1. Identify type to select the current data.
  // Position Data
  if(type==PositionCtrl)
    for(int i=0;i<3;i++) temp(i) = CurData(i);

  // Pose Data
  else if(type==PoseCtrl)
    for(int i=3;i<6;i++) temp(i) = CurData(i);

  // Force Data
  else if(type==ForceCtrl)
    for(int i=0;i<3;i++) temp(i) = CurData(i);

  // Moment Data
  else if(type==MomentCtrl)
    for(int i=3;i<6;i++) temp(i) = CurData(i);

  else
    return -1;

  // 2. Compute the error between the desired data and actual data
  ComputeError(DesData, temp, ErrorOutput, ErrorNorm, ErrorFlag);

  // 3. Multiply by Gain and Jacobian and place output on AngleUpdate
  JacobianProduct(Jacobian, type, ErrorOutput, JointAngleUpdate);

  // 4. For single controller, add to current angles
  if(NumCtlrs==1)
    // Add the joint angle update to the current angular joint position of the robot
    UpdateJointAngles(CurJointAngles, JointAngleUpdate);

  return 0;
}

// Same as above but for a 6 DOF robot.
int ControlBasis::ComputePrimitiveController(	/*out*/ 	dvector6& 			JointAngleUpdate,
						/*in*/ 		int 				NumCtlrs,
						/*in*/ 		ControllerType 		type,
						/*in*/ 		dvector6& 			DesData,
						/*in*/		dvector6&			CurData,
						/*in,out*/	dvector6& 			CurJointAngles,
						/*in*/		dmatrix&			Jacobian,
						/*in*/		int					ErrorFlag,
						/*out*/		double&				ErrorNorm)
{
  // Check to ensure that all incoming signals are valid
  if(&DesData==NULL || &CurData==NULL)
    return -1;

  // Local Variable declaration and initialization
  dvector6 temp; for(int i=0;i<6;i++) temp(i)=0;

  // 1. Identify type to select the current data.
  // Position Data
  if(type==PositionCtrl)
    for(int i=0;i<3;i++) temp(i) = CurData(i);

  // Pose Data
  else if(type==PoseCtrl)
    for(int i=3;i<6;i++) temp(i) = CurData(i);

  // Force Data
  else if(type==ForceCtrl)
    for(int i=0;i<3;i++) temp(i) = CurData(i);

  // Moment Data
  else if(type==MomentCtrl)
    for(int i=3;i<6;i++) temp(i) = CurData(i);

  else
    return -1;

  // 2. Compute the error between the desired data and actual data
  ComputeError(DesData, temp, ErrorOutput, ErrorNorm, ErrorFlag);

  // 3. Multiply by Gain and Jacobian and place output on AngleUpdate
  JacobianProduct(Jacobian, type, ErrorOutput, JointAngleUpdate);

  // 4. For single controller, add to current angles
  if(NumCtlrs==1)
    // Add the joint angle update to the current angular joint position of the robot
    UpdateJointAngles(CurJointAngles, JointAngleUpdate);

  return 0;
}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

int ControlBasis::ComputeError(/*in*/dvector6& DesData, /*in*/dvector6& CurData, /*out*/dvector6& ErrorOutput, /*out*/double& ErrorNorm, /*in*/int flag)
{
  // 1. Check dimensionality of desired and actual data to ensure they are of same dimension
  if( DesData.size() != CurData.size() )
    return -1;

  // 2. Compute the error: Desired - Actual
  //    Then get the NEGATIVE OF THE ERROR. This will help us move in the right direction
  if(ERROR_TEST)
    {
      float CONST_ERROR_VALUE = -0.4;				// When desiring to fix the error value and see how the jacobian responds.
      ErrorOutput(5) = -CONST_ERROR_VALUE;
    }
  else
    for(int i=0;i<6;i++)
      {
	ErrorOutput(i) = DesData(i) - CurData(i);
	ErrorOutput(i) = -1.0 * ErrorOutput(i);

	// Check magnitude of signal. If too large, act.
	if(ErrorOutput(i)>MAX_ERROR)
	  {
	    ErrorOutput(i) = MAX_ERROR;
	    std::cerr << "Warning: ErrorOutput(). Maximum limit for joint " << i << " has been reached!" << std::endl;
	  }
	if(ErrorOutput(i)<-MAX_ERROR)
	  {
	    ErrorOutput(i) = -MAX_ERROR;
	    std::cerr << "Warning: ErrorOutput(). Maximum limit for joint " << i << " has been reached!" << std::endl;
	  }
      }

#ifdef DEBUG_PLUGIN3
  // Print the error to the terminal
  if(DEBUG)
    {
      std::cerr << "\n/----------------------------------------------------------------------------/\n"
	"Error " << flag << " is: " 	<< ErrorOutput(0) << " " << ErrorOutput(1) << " " << ErrorOutput(2) << " "
		<< ErrorOutput(3) << " " << ErrorOutput(4) << " " << ErrorOutput(5);
    }
#endif

  // 3. Compute the root mean square
  for(int i=0;i<6;i++) ErrorNorm += pow(ErrorOutput(i),2);
  ErrorNorm = sqrt(ErrorNorm)/3;

  // Printer error to terminal
  //if(DEBUG)
#ifdef DEBUG_PLUGIN3
  std::cerr << "\nThe error norm # " << flag << " is: " << ErrorNorm <<
    "\n/----------------------------------------------------------------------------/\n";
#endif


  // 4. If low error set Errorflag to true. Used in position control in order to receive the next desired coordinate.
  if(ErrorNorm < 0.01)
    ErrorFlag = true;

  return 0;
}

/*********************************************** JacobianProduct ***********************************************************
 ** The Jacobian product can be computed using the pseudoinverse/transpose in the case of position control
 ** and transpose in case of force/moment.
 ** The transpose is an approximation that can work if scaled appropriately.
 ** The latter is more stable that the pseudoinverse approach which struggles near singularities.
 **
 ** For a 6x7 non-square matrix:
 ** For position control:
 ** del_x = Jacobian * del_q
 ** del_q = pseudoInverse(Jacobian) * del_x
 **
 ** For force control:
 ** del_t = Jacobian'*del_F
 **
 ** Note: Matrices use column-major indexing.
 *************************************************************************************************************************************/
int ControlBasis::JacobianProduct(/*in*/	dmatrix& 		Jacobian,
				  /*in*/	ControllerType 	type,
				  /*in*/	dvector6& 		ErrorOutput,
				  /*out*/ 	dvector7& 		AngleUpdate)
{
  // Local Variables
  dvector6 temp; for(int i=0;i<6;i++) temp(i)=0;

  /******************** Jacobian Pseudoinverse: J* = J'(JJ')^-1 *************************************
   // The Jacobian transform can be used if the product is multiplied by a small scalar value:
   // del_q = inv(Jac)*del_x but one can also do del_q = alpha*trans(Jac)*del_x, with small alpha.
   // For details of the process see a great description at: Intro to Inv. Kin.'s with Jac. Transpose
   // Pseudoinverse and Damped Least Squared Methods by Samuel R. Buss. Oct 7, 2009. ****************/
  dmatrix76 transJac 			= trans(Jacobian);
  dmatrix66 op; noalias(op) 	= prod(Jacobian,transJac);

  // Check for singularities, by making sure the determinant is not zero.
  // (not yet implemented) if det(zero) curAngles = prevAngles??

  // 1) Identify controller type to select the appropriate gains
  if(type==PositionCtrl)
    {
      if(PSEUDO_JAC_HRP)	// HRP has computed the pseudojac. Passed into this function as Jacobian.
	{
	  for(int i=0;i<6;i++) temp(i)=PositionGain(i)*ErrorOutput(i);
	  noalias(AngleUpdate) = prod(Jacobian,temp);
	}
      else if(POSITION_JAC_TRANS_CTRL) /*** Jacobian transpose control ****/
	{
	  // Derive: del_q = alpha*J'*del_x
	  // Alpha Computation:
	  // 		alpha = dot(e,JJ'e)/dot(JJ'e)
	  double alpha, num, den;
	  dvector b(6);

	  // Compute in steps:
	  noalias(b) = prod(op,ErrorOutput);
	  num= ublas::inner_prod(ErrorOutput,b);
	  den= ublas::inner_prod(b,b);

	  // Use alpha and jacobian transpose to compute angle update:
	  alpha = num/den;
	  noalias(AngleUpdate) = alpha*prod(transJac,ErrorOutput);
	}

      else /*** Jacobian pseudoinverse control ****/
	{
	  // The next two equations have been placed here and not before for computational efficiency.
	  dmatrix66 inv_op 						= inverse(op);
	  dmatrix76 pseudoJac; noalias(pseudoJac)	= prod(transJac, inv_op);

	  for(int i=0;i<6;i++) temp(i)=PositionGain(i)*ErrorOutput(i);
	  noalias(AngleUpdate) = prod(pseudoJac,temp);
	}
    }

  else if(type==PoseCtrl)
    {
      if(PSEUDO_JAC_HRP)	// HRP has computed the pseudojac. Passed into this function as Jacobian.
	{
	  for(int i=0;i<6;i++) temp(i)=PoseGain(i)*ErrorOutput(i);
	  noalias(AngleUpdate) = prod(Jacobian,temp);
	}
      else if(POSE_JAC_TRANS_CTRL) /*** Jacobian transpose control ****/
	{
	  // Derive: del_q = alpha*J'*del_x
	  // Alpha Computation:
	  // 		alpha = dot(e,JJ'e)/dot(JJ'e)
	  double alpha, num, den;
	  dvector b(6);

	  // Compute in steps:
	  noalias(b) = prod(op,ErrorOutput);
	  num= ublas::inner_prod(ErrorOutput,b);
	  den= ublas::inner_prod(b,b);

	  // Use alpha and jacobian transpose to compute angle update:
	  alpha = 0.0002*(num/den);
	  noalias(AngleUpdate) = alpha*prod(transJac,ErrorOutput);
	}

      else /*** Jacobian pseudoinverse control ****/
	{
	  // The next two equations have been placed here and not before for cmputational efficiency.
	  dmatrix66 inv_op 						= inverse(op);
	  dmatrix76 pseudoJac; noalias(pseudoJac)	= prod(transJac, inv_op);

	  for(int i=0;i<6;i++) temp(i)=PoseGain(i)*ErrorOutput(i);
	  noalias(AngleUpdate) = prod(pseudoJac,temp);
	}
    }

  else if(type==ForceCtrl)
    {
      for(int i=0;i<6;i++) temp(i)=ForceGain(i)*ErrorOutput(i);

      // Transpose the Jacobian used for force or moment control
      noalias(AngleUpdate) = prod(transJac,temp);

      // Zero out wrist updates
      for(int i=3;i<6;i++) AngleUpdate(i)=0.0;

    }

  else if(type==MomentCtrl)
    {
      for(int i=0;i<6;i++) temp(i)=MomentGain(i)*ErrorOutput(i);

      // Transpose the Jacobian used for force or moment control
      noalias(AngleUpdate) = prod(transJac,temp);

      // Zero out arm updates
      for(int i=0;i<3;i++) AngleUpdate(i)=0.0;
    }

  else
    return -1;

  return 0;
}

/*********************************************** JacobianProduct ***********************************************************
 ** Same as above for 6 DOF robot
 *************************************************************************************************************************************/
int ControlBasis::JacobianProduct(/*in*/	dmatrix& 		Jacobian,
				  /*in*/	ControllerType 	type,
				  /*in*/	dvector6& 		ErrorOutput,
				  /*out*/ 	dvector6& 		AngleUpdate)
{
  // Local Variables
  dvector6 temp; for(int i=0;i<6;i++) temp(i)=0;

  /******************** Jacobian Pseudoinverse: J* = J'(JJ')^-1 *************************************
   // The Jacobian transform can be used if the product is multiplied by a small scalar value:
   // del_q = inv(Jac)*del_x but one can also do del_q = alpha*trans(Jac)*del_x, with small alpha.
   // For details of the process see a great description at: Intro to Inv. Kin.'s with Jac. Transpose
   // Pseudoinverse and Damped Least Squared Methods by Samuel R. Buss. Oct 7, 2009. ****************/
  dmatrix66 transJac 			= trans(Jacobian);
  dmatrix66 op; noalias(op) 	= prod(Jacobian,transJac);

  // Check for singularities, by making sure the determinant is not zero.
  // (not yet implemented) if det(zero) curAngles = prevAngles??

  // 1) Identify controller type to select the appropriate gains
  if(type==PositionCtrl)
    {
      if(PSEUDO_JAC_HRP)	// HRP has computed the pseudojac. Passed into this function as Jacobian.
	{
	  for(int i=0;i<6;i++) temp(i)=PositionGain(i)*ErrorOutput(i);
	  noalias(AngleUpdate) = prod(Jacobian,temp);
	}
      else if(POSITION_JAC_TRANS_CTRL) /*** Jacobian transpose control ****/
	{
	  // Derive: del_q = alpha*J'*del_x
	  // Alpha Computation:
	  // 		alpha = dot(e,JJ'e)/dot(JJ'e)
	  double alpha, num, den;
	  dvector b(6);

	  // Compute in steps:
	  noalias(b) = prod(op,ErrorOutput);
	  num= ublas::inner_prod(ErrorOutput,b);
	  den= ublas::inner_prod(b,b);

	  // Use alpha and jacobian transpose to compute angle update:
	  alpha = num/den;
	  noalias(AngleUpdate) = alpha*prod(transJac,ErrorOutput);
	}

      else /*** Jacobian pseudoinverse control ****/
	{
	  // The next two equations have been placed here and not before for computational efficiency.
	  dmatrix66 inv_op 						= inverse(op);
	  dmatrix76 pseudoJac; noalias(pseudoJac)	= prod(transJac, inv_op);

	  for(int i=0;i<6;i++) temp(i)=PositionGain(i)*ErrorOutput(i);
	  noalias(AngleUpdate) = prod(pseudoJac,temp);
	}
    }

  else if(type==PoseCtrl)
    {
      if(PSEUDO_JAC_HRP)	// HRP has computed the pseudojac. Passed into this function as Jacobian.
	{
	  for(int i=0;i<6;i++) temp(i)=PoseGain(i)*ErrorOutput(i);
	  noalias(AngleUpdate) = prod(Jacobian,temp);
	}
      else if(POSE_JAC_TRANS_CTRL) /*** Jacobian transpose control ****/
	{
	  // Derive: del_q = alpha*J'*del_x
	  // Alpha Computation:
	  // 		alpha = dot(e,JJ'e)/dot(JJ'e)
	  double alpha, num, den;
	  dvector b(6);

	  // Compute in steps:
	  noalias(b) = prod(op,ErrorOutput);
	  num= ublas::inner_prod(ErrorOutput,b);
	  den= ublas::inner_prod(b,b);

	  // Use alpha and jacobian transpose to compute angle update:
	  alpha = 0.0002*(num/den);
	  noalias(AngleUpdate) = alpha*prod(transJac,ErrorOutput);
	}

      else /*** Jacobian pseudoinverse control ****/
	{
	  // The next two equations have been placed here and not before for cmputational efficiency.
	  dmatrix66 inv_op 						= inverse(op);
	  dmatrix76 pseudoJac; noalias(pseudoJac)	= prod(transJac, inv_op);

	  for(int i=0;i<6;i++) temp(i)=PoseGain(i)*ErrorOutput(i);
	  noalias(AngleUpdate) = prod(pseudoJac,temp);
	}
    }

  else if(type==ForceCtrl)
    {
      for(int i=0;i<6;i++) temp(i)=ForceGain(i)*ErrorOutput(i);

      // Transpose the Jacobian used for force or moment control
      noalias(AngleUpdate) = prod(transJac,temp);

      // Zero out wrist updates
      for(int i=3;i<6;i++) AngleUpdate(i)=0.0;

    }

  else if(type==MomentCtrl)
    {
      for(int i=0;i<6;i++) temp(i)=MomentGain(i)*ErrorOutput(i);

      // Transpose the Jacobian used for force or moment control
      noalias(AngleUpdate) = prod(transJac,temp);


      // Zero out arm updates
      for(int i=0;i<3;i++) AngleUpdate(i)=0.0;
    }

  else
    return -1;

  return 0;
}


/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

/*******************************************************************************************************
 ** NullSpaceProjection()
 ** This function projects the output of the subordinate
 ** controller to the left null space of the dominant controller.
 ** It is important to note that the dominant output is the update
 ** produced by that controller but is not the updated position.
 ** Ie the update might be 0.4 degrees and the position might be 90.4 degrees.
 **
 ** We multiply AngleUpdate2 * null_space_matrix and
 ** place the result in NullSpaceProjMat
 **
 ** This controller consider a 6D/7D space, given that the input
 ** and output vectors will always be the change in joint
 ** coordinates, AngleUpdate2(q1,q2,q3,q4,q5,q6,q7).
 **
 ** From Platt's work, the null space operator, N, is defined as the moore-penrose generalized pseudoinverse:
 **
 ** N = I - [x_out * inv(x_out' * x_out) * x_out']: that is, the identity - *outer product / inner product).
 ** From linear algebra the fraction of outer product/inner product is a projection unto the null space of the column space.
 ** When we subtract the identity from it, we are projecting on the perpendicular space, the left null space.
 **
 ** N = I - [ 1/(q1^2 + q2^2 + q3^2 + q4^2 + q5^2 + q6) * |q1^2 q1q2 q13 ... q1q6 |
 **														|q2q1 q2^2 ...     q2q6 |
 **														|q6q1  ... ...      q6^2|
 **
 ** Notice that the identity matrix is subtracted from
 ** the outer product normalized by the dot product of x_out.
 **
 ** Variables:
 **
 ** SPECIAL CASE: there may be a time where the dominant controller is all zeros
 ** If this is the case, we need to code the value of denominator to 1, otherwise
 ** there will be a division by zero.
 ********************************************************************************************************/
int ControlBasis::NullSpaceProjection(	/*out*/			dvector7& 			JointAngleUpdate,
					/*in*/         	const dvector7& 	AngleUpdate1,//
					/*in*/         	const dvector7& 	AngleUpdate2)//
{
  // Local variables
  double innerProduct = 1.0;
  dvector7 leftNullSpace;
  dmatrix77 NullSpaceProjMat, outerProduct, norm_outerProduct;
  ublas::identity_matrix<double> Identity(7);

  // Initialization
  for(int i=0;i<7;i++)
    {
      for(int j=0;j<7;j++)
	{
	  outerProduct(i,j)		= 0.0;
	  norm_outerProduct(i,j)	= 0.0;
	  NullSpaceProjMat(i,j)	= 0.0;
	}
      leftNullSpace(i) = 0.0;
    }

  // 1. Computer the inner product.
  innerProduct = inner_prod(AngleUpdate1,AngleUpdate1);
  // In cases where there is no contact, the inner product would be zero, so here we deal with that exception to make it one in value.
  if(innerProduct == 0.0)
    innerProduct=1.0;

  // 2. Compute the outer product.
  outerProduct = outer_prod(AngleUpdate1,AngleUpdate1);

  // 3. Normalize the outer product
  norm_outerProduct = outerProduct/innerProduct;

  // 4. Compute the left null space projection matrix by subtracting the identity from the nullspace projection matrix
  NullSpaceProjMat = Identity - norm_outerProduct;

  // 5. Compute the nullspace projection of the subordinate controller unto the dominant controller
  leftNullSpace = prod(NullSpaceProjMat,AngleUpdate2);

  // 6. Add the left nullspace projection to the dominant vector update
  JointAngleUpdate = AngleUpdate1 + leftNullSpace;

  // check for nan/inf values
  for(int i=0;i<7;i++)
    if(isnan(JointAngleUpdate(i)))
      {
	std::cerr << "JointAngleUpdate produced an: isnan. " << std::endl;
	return -1;
      }

  return 0;
}

/********************************************************************************************************
 * Same as above but for 6 DOF robot.
 ********************************************************************************************************/
int ControlBasis::NullSpaceProjection(	/*out*/			dvector6& 			JointAngleUpdate,
					/*in*/         	const dvector6& 	AngleUpdate1,
					/*in*/         	const dvector6& 	AngleUpdate2)
{
  // Local variables
  double innerProduct = 1.0;
  dvector6 leftNullSpace;
  dmatrix66 NullSpaceProjMat, outerProduct, norm_outerProduct;
  ublas::identity_matrix<double> Identity(6);

  // Initialization
  for(int i=0;i<6;i++)
    {
      for(int j=0;j<6;j++)
	{
	  outerProduct(i,j)		= 0.0;
	  norm_outerProduct(i,j)	= 0.0;
	  NullSpaceProjMat(i,j)	= 0.0;
	}
      leftNullSpace(i) = 0.0;
    }

  // 1. Computer the inner product.
  innerProduct = inner_prod(AngleUpdate1,AngleUpdate1);
  // In cases where there is no contact, the inner product would be zero, so here we deal with that exception to make it one in value.
  if(innerProduct == 0.0)
    innerProduct=1.0;

  // 2. Compute the outer product.
  outerProduct = outer_prod(AngleUpdate1,AngleUpdate1);

  // 3. Normalize the outer product
  norm_outerProduct = outerProduct/innerProduct;

  // 4. Compute the left null space projection matrix by subtracting the identity from the nullspace projection matrix
  NullSpaceProjMat = Identity - norm_outerProduct;

  // 5. Compute the nullspace projection of the subordinate controller unto the dominant controller
  leftNullSpace = prod(NullSpaceProjMat,AngleUpdate2);

  // 6. Add the left nullspace projection to the dominant vector update
  JointAngleUpdate = AngleUpdate1 + leftNullSpace;

  // check for nan/inf values
  for(int i=0;i<6;i++)
    if(isnan(JointAngleUpdate(i)))
      {
	std::cerr << "JointAngleUpdate produced an: isnan. " << std::endl;
	return -1;
      }

  return 0;
}
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

/********************************************************************************************************
 ** Adds the update joint angles to the current joint angles while using an averaging filter.
 ********************************************************************************************************/
int ControlBasis::UpdateJointAngles(/*in*/ dvector7& CurJointAngles,
				    /*out*/ dvector7& JointAngleUpdate)
{
  // Add the joint update to the current angles
  for(int i=0;i<7;i++) CurJointAngles(i)=CurJointAngles(i)+JointAngleUpdate(i);

  // If first time, set PrevJointAngles to CurJointAngles
  if(this->updateFlag==true)
    {
      for(int i=0;i<7;i++)
	{
	  PrevJointAngles(i) = CurJointAngles(i);
	}

      // Once read change the flag
      this->updateFlag = false;
    }

  // Average current and previous signals (1st order low-pass filter).
  for(int i=0;i<7;i++) CurJointAngles(i) = weight*CurJointAngles(i) + (1-weight)*PrevJointAngles(i);

  // Set CurAngles to PrevAngles
  for(int j=0;j<7;j++) PrevJointAngles(j) = CurJointAngles(j);

  return 0;
}

/********************************************************************************************************
 * Same as above but for 6 DOF robot.
 ********************************************************************************************************/
int ControlBasis::UpdateJointAngles(/*in*/ dvector6& CurJointAngles,
				    /*out*/ dvector6& JointAngleUpdate)
{
  // Add the joint update to the current angles
  for(int i=0;i<6;i++) CurJointAngles(i)=CurJointAngles(i)+JointAngleUpdate(i);

  // If first time, set PrevJointAngles to CurJointAngles
  if(this->updateFlag==true)
    {
      for(int i=0;i<6;i++)
	{
	  PrevJointAngles6(i) = CurJointAngles(i);
	}

      // Once read change the flag
      this->updateFlag = false;
    }

  // Average current and previous signals (1st order low-pass filter).
  for(int i=0;i<6;i++) CurJointAngles(i) = weight*CurJointAngles(i) + (1-weight)*PrevJointAngles6(i);

  // Set CurAngles to PrevAngles
  for(int j=0;j<6;j++) PrevJointAngles6(j) = CurJointAngles(j);

  return 0;
}

/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

/********************************************************************************************************
 **	Sets gains for position, force, and moment control primitives
 ********************************************************************************************************/
void ControlBasis::SetGains(double factor)
{
  // Position
  double Px = 0.000100; 		// This gain gives a steady motion for J0. Faster motions can be achieved up to x10.
  double Py = 0.000010;		// Smallest possible gain to achieve motion without failing.
  double Pz = 0.000220;		// Smallest possible gain to achieve motion without failing.

  // Pose
  double Pr = 0.000075;// 0.002;//0.02;

  // Force
  double F  = 0.0000500; // 0.00025; // The former value worked in independent experimentation. But this actual value works for the camera.

  // Moment
  double M  = 0.0000035*factor; // 0.00075; // The former value worked in independent experimentation. But this actual value works for the camera.

  PositionGain(0) = Px;		PoseGain(0) = 0;			ForceGain(0) = F; 			MomentGain(0) = 0.0;
  PositionGain(1) = Py;		PoseGain(1) = 0;			ForceGain(1) = F; 			MomentGain(1) = 0.0;
  PositionGain(2) = Pz;		PoseGain(2) = 0;			ForceGain(2) = F; 			MomentGain(2) = 0.0;
  PositionGain(3) = 0;		PoseGain(3) = -Pr;			ForceGain(3) = 0.0;			MomentGain(3) = M;
  PositionGain(4) = 0;		PoseGain(4) = 0.00003;		ForceGain(4) = 0.0;			MomentGain(4) = M;
  PositionGain(5) = 0;		PoseGain(5) = 0.00008;		ForceGain(5) = 0.0;			MomentGain(5) = M;
}
