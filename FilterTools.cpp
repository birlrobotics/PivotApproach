/*
 * FilterTools.cpp
 *
 *  Created on: Mar 19, 2012
 *      Author: juan
 */

#include "FilterTools.h"

FilterTools::FilterTools()
{
	for(int i=0;i<6;i++)
	{
		stateVar6(i) = 0.0;
		avgSig(i)    = 0.0;
	}

	// Flags
	ctrlInitFlag = true;
	dataHistFlag = true;

	// Filter Parameters generated from Matlab. See the description in ::LowPassFilter.
	A=0.8541; B=0.2064; C=0.6555; D=0.0730;
}

FilterTools::~FilterTools(){}

/********************************************************************************************************
* Low pass filter - 1st order
* Follows the state-space variable form:
* Xn+1= AXn + Bu
* Y   = CXn + Bu
*
* Where Xn starts as zero.
*
* The variables A,B,C,D are obtained from Matlab's butter filter function using:
* 	order: 1 and a cutoff frequencies. The best one yet is 0.05.
*
* 	cutoff freq: 0.5
* 	double A=5.5511e-017, B=1.4142, C=0.3536, D=0.5;
*
* 	cutoff freq: 0.25
* 	double A=0.4142, B=0.8284, C=0.500, D=0.2929;
*
* 	cutoff freq: 0.05
* 	double A=0.8541, B=0.2064, C=0.6555, D=0.0730;
*
* 	cutoff freq:
********************************************************************************************************/
int FilterTools::LowPassFilter(double in[6], double out[6])
{
	double A=0.8541; double B=0.2064; double C=0.6555;
	// double D=0.0730;

	// First write the output equation: Y = CXn + Bu
	for(int i=0;i<6;i++)
	{
		out[i] = C*stateVar6(i) + B*in[i];

		// Second write the state vec equation: Xn+1= AXn + Bu
		stateVar6(i) = A*stateVar6(i) + B*in[i];
	}

	return 0;
}

int FilterTools::LowPassFilter(dvector6 in, dvector6& out)
{
	// First write the output equation: Y = CXn + Bu
	out = C*stateVar6 + B*in;

	// Second write the state vector equation: Xn+1= AXn + Bu
	stateVar6 = A*stateVar6 + B*in;

	return 0;
}

/********************************************************************************************************
** Adds the update joint angles to the current joint angles while using an averaging filter.
********************************************************************************************************/
int FilterTools::MovingAverage(/*in,out*/ dvector6& CurData, /*in*/ int bufferLength)
{
	// Local variables
	float  weight_sum = 0;
	int    dataLength = CurData.size();

	// Zero data buffer if first time
	if(dataHistFlag)
	{
		// Resize Data
		//dataHist.resize(bufferLength,dataLength);

		// Initialize data
		for(int i=0; i<dataHist.size1(); i++)
			for(int j=0; j<dataHist.size2(); j++)
				dataHist(i,j) = 0.0;
		dataHistFlag = false;
	}

	// 1a) Enter new data in history buffer. Move data down the buffer.
	for(int i=bufferLength-1; i>0; i--)					// Note that we have to start at bufferLength-1 b/c indeces go from 0:bufferLength-1
		for(int j=0; j<dataLength; j++)					// 0:<length
			dataHist(i,j) = dataHist(i-1,j);			// Copy data from buffer 1 to 2, 2 to 3, etc.
	// 1b) At the end, copy CurData to the first element of the buffer.
	for(int j=0;j<dataLength;j++)
	{
			dataHist(0,j)=CurData(j);	// After copying, set to zero, to do the averaging with the hist buffer.
			CurData(j) = 0;
	}

	// 2) Compute the weight factor.
	//    Will vary with the length of this history buffer. Data value will decrease linearly with time: w1*bufferLength_0 + w2*bufferLength_1 + ... + wn*bufferLength_n
	for(int k=bufferLength;k>0;k--) weight_sum+=k;

	// Moving Average
	for(int i=0;i<bufferLength;i++)
		for(int j=0;j<dataLength;j++)
			CurData(j) += ( float(bufferLength-i)/weight_sum)*dataHist(i,j);					// Have each row of data history be multiplied by a weighting factor, that decreases linearly over time as a function of history buffer length

	// Set the current result as the previous result
	for(int i=0; i<dataLength; i++)
		dataHist(0,i) = CurData(i);

	return 0;
}
