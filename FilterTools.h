/*
 * FilterTools.h
 *
 *  Created on: Mar 19, 2012
 *      Author: juan
 */

#ifndef FILTERTOOLS_H_
#define FILTERTOOLS_H_

// OpenHRP Classes
// Linux
//#include "/home/juan/openhrp/OpenHRP3.0-HIRO/DynamicsSimulator/server/Link.h"			// Needed for full definitino of the LinkPath Clas
//#include "/home/juan/openhrp/OpenHRP3.0-HIRO/DynamicsSimulator/server/LinkPath.h"		// Used to compute inverse kinematics and jacobian
//#include "/home/juan/openhrp/OpenHRP3.0-HIRO/DynamicsSimulator/server/Body.h"			// Used to retrieve the position/joint angles/rot matrix of the kinematic body


#include "/home/grxuser/src/OpenHRP3.0/DynamicsSimulator/server/hrpModelHeaders.h"	// DynamicSimulator/server

// QNX
//#include "../../../DynamicsSimulator/server/LinkPath.h"

/*
#include "/home/hrpuser/src/OpenHRP3/DynamicsSimulator/server/Link.h"
#include "/home/hrpuser/src/OpenHRP3/DynamicsSimulator/server/Body.h"
*/
// TVMet Files
#include <tvmet/Matrix.h>
#include <tvmet/Vector.h>
#include <tvmet/VectorFunctions.h>

// Boost files
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>


// Local include files
#include <vector>
#define HISTBUFF_LENGTH 	12

using namespace OpenHRP;
using namespace tvmet;
using namespace std;

class FilterTools {
public:
	FilterTools();
	virtual ~FilterTools();

	/************************** Members ******************************/
	bool 		dataHistFlag;				// Flag to initialize dataHist
	bool 		ctrlInitFlag;				// Flag to initialize ctrl primitives
	dvector6	stateVar6;					// State variable for each of the elements of the FxyzTxyz used in a low-pass filter
	dvector6 	avgSig;						// contains filtered/moving average result signal
	ublas::bounded_matrix<double, HISTBUFF_LENGTH, 6, ublas::column_major> dataHist;					// Holds filter/general data history in a buffer. Matrix dims must agree with



	/************************** Methods ******************************/
	// Low Pass Filter Declaration
	int LowPassFilter(double in[6], double out[6]);

	// Moving Average Filter
	int MovingAverage(/*in,out*/dvector6& CurData,/*in*/int bufferLength);
};

#endif /* FILTERTOOLS_H_ */
