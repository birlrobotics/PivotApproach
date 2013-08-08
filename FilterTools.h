/*
 * FilterTools.h
 *
 *  Created on: Mar 19, 2012
 *      Author: juan
 */

#ifndef FILTERTOOLS_H_
#define FILTERTOOLS_H_

// DynamicSimulator/server
#include "/home/grxuser/src/OpenHRP3.0/DynamicsSimulator/server/hrpModelHeaders.h"

// TVMet Files
#include <tvmet/Matrix.h>
#include <tvmet/Vector.h>
#include <tvmet/VectorFunctions.h>

// Boost files
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>

// Local include files
#include <vector>

// Define's
#define HISTBUFF_LENGTH 	12

// Namespaces
using namespace OpenHRP;
using namespace tvmet;
using namespace boost;
using namespace std;

class FilterTools {
	public:
		FilterTools();
		virtual ~FilterTools();

		/*------------------------------------------------------------ Methods ------------------------------------------------------------*/
		// Low Pass Filter Declaration
		int LowPassFilter(double in[6], double out[6]);
		//int LowPassFilter(dvector6& in, dvector6& out);
		int secOrderFilter(double in[6], double out[6]);
		//int secOrderFilter(dvector6& in, dvector6& out);

		// Moving Average Filter
		int MovingAverage(/*in,out*/dvector6& CurData,/*in*/int bufferLength);

	private:
		/*------------------------------------------------------------ Members ------------------------------------------------------------*/
		double 		A, B, C, D; 				// 1st order Low Pass Filter Parameters
		bool 		dataHistFlag;				// Flag to initialize dataHist
		bool 		ctrlInitFlag;				// Flag to initialize ctrl primitives
		dvector6	stateVar6;					// State variable for each of the elements of the FxyzTxyz used in a low-pass filter
		dvector6 	avgSig;						// contains filtered/moving average result signal
		ublas::bounded_matrix<double, HISTBUFF_LENGTH, 6, ublas::column_major> dataHist;					// Holds filter/general data history in a buffer. Matrix dims must agree with

		// Second order filter
		//double a_1, a_2, a_3, b_1, b_2, b_3;		// 2nd order Low Pass Filter parameters
		double in_t1[6]; 							// vector that contains the input at time t-1
		double in_t2[6]; 							// vector that contains the input at time t-2
		double out_t1[6]; 							// vector that contains the output at time t-1
		double out_t2[6]; 							// vector that contains the output at time t-2

};

#endif /* FILTERTOOLS_H_ */
