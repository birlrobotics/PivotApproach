/*
 * FilterTools.cpp
 *
 *  Created on: Mar 19, 2012
 *      Author: juan
 */

#include "FilterTools.h"

FilterTools::FilterTools()
{
	// Initialize all vectors
	for(int i=0;i<6;i++)
	{
		stateVar6(i)	= 0.0;
		avgSig(i)    	= 0.0;
		in_t1[i] 	 	= 0.0;
		in_t2[i] 	 	= 0.0;
		out_t1[i] 	 	= 0.0;
		out_t2[i] 	 	= 0.0;
	}

	// Flags
	ctrlInitFlag = true;
	dataHistFlag = true;

	// Parameters
	// 1st order Filter Parameters generated from Matlab. See the description in ::LowPassFilter.
	// Cutoff frequency: 0.050: A = 0.85410, B = 0.20640, C = 0.6555, D=0.0730;
	A=0.8541; B=0.2064; C=0.6555; D=0.0730;

	// 2nd order Filter Parameters.
/*	a_1 =  0.0055;
	a_2 =  0.0111;
	a_3 =  0.0055;
	b_1 =  1.0000;
	b_2 = -1.7786;
	b_3 =  0.8008;*/
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
*   MATLAB
*   1st Order Filter
* 	cutoff freq: 0.500: A = 5.5511e-017, B=1.4142, C=0.3536, D=0.5;
* 	cutoff freq: 0.250: A = 0.41420, B = 0.82840, C = 0.500, D=0.2929;
* 	cutoff freq: 0.050: A = 0.85410, B = 0.20640, C = 0.6555, D=0.0730;
* 	cutoff freq: 0.040: A = 0.88160, B = 0.16740, C = 0.66530, D = 0.05920
* 	cutoff freq: 0.030: A = 0.90990, B = 0.12740, C = 0.67530, D = 0.04500
* 	cutoff freq: 0.025  a = 0.9244,  b = 0.1069,  c =  0.6804, d = 0.0378
* 	cutoff freq: 0,020  a = 0.9391,  b = 0.0862,  c = 0.6856,  d = 0.0305
* 	cutoff freq: 0.010  a = 0.9691,  b = 0.0437,  c = 0.6962,  d = 0.0155
*
* 	2nd order
* 	cutoff freq: 0.05  a = 0.0055    0.0111    0.0055, b = 1.0000   -1.7786    0.8008
* 	cutoff freq: 0.04  a = 0.0036    0.0072    0.0036, b = 1.0000   -1.8227    0.8372
* 	cutoff freq: 0.03  a = 0.0021    0.0042    0.0021, b = 1.0000   -1.8669    0.8752
*
*   OCTAVE
*   1st Order Filter
*   cutoff freq 0.025: A = 0.92439, B = 0.72751, C = 0.10000, D = 0.037805
*   cutoff freq 0.001: A = 0.96907, B = 0.30454, C = 0.10000, D = 0.037805
********************************************************************************************************/
int FilterTools::LowPassFilter(double in[6], double out[6])
{

	// State-variable filter parameters, for 1st order low-pass filter with cut-off frequency = 0.04
	//double A=0.9391 /*0.8541*/;
	//double B=0.0862 /*0.2064*/;
	//double C=0.6856 /*0.6555*/;

	// First write the output equation: Y = CXn + Bu
	for(int i=0;i<6;i++)
	{
		//out[i] = C*stateVar6(i) + B*in[i];

		// Second write the state vector equation: Xn+1= AXn + Bu
		//stateVar6(i) = A*stateVar6(i) + B*in[i];

		// Main recursive 2nd order equation
		// out[i] = (a0*in[i]+a1*in_t1(i)+a2*in_t2(i)-b1*out_t1(i)-b2*out_t2(i))/b0;

		// Cutoff = 0.2 (octave). For 2nd order filter octave results are almost identical to matlab;
		// a_1 =  0.067455; a_2 =  0.134911; a_3 =  0.067455; b_1 =  1.0000; b_2 = -1.14298; b_3 =  0.41280;
		// out[i] = 0.067455*in[i] + 0.134911*in_t1[i] + 0.067455*in_t2[i] + 1.14298*out_t1[i] - 0.41280*out_t2[i];

		// Cutoff = 0.15 (octave). For 2nd order filter octave results are almost identical to matlab;
		// a_1 =  0.041254; a_2 =  0.082507; a_3 =  0.041254; b_1 =  1.0000; b_2 = -1.34897; b_3 =  0.51398;
		// out[i] = 0.041254*in[i] + 0.082507*in_t1[i] + 0.041254*in_t2[i] + 1.34897*out_t1[i] - 0.51398*out_t2[i];


		// Cutoff = 0.1 (octave). For 2nd order filter octave results are almost identical to matlab;
		// a_1 =  0.020083; a_2 =  0.040167; a_3 =  0.020083; b_1 =  1.0000; b_2 = -1.56102; b_3 =  0.64135;
		out[i] = 0.020083*in[i] + 0.040167*in_t1[i] + 0.020083*in_t2[i] + 1.56102*out_t1[i] - 0.64135*out_t2[i];


		// Cutoff = 0.05;
		// a_1 =  0.0055; a_2 =  0.0111; a_3 =  0.0055; b_1 =  1.0000; b_2 = -1.7786; b_3 =  0.8008;
		// out[i] = 0.0055*in[i] + 0.0111*in_t1[i] + 0.0055*in_t2[i] + 1.7786*out_t1[i] - 0.8008*out_t2[i];

		// cutoff freq: 0.03
		// a = 0.0021    0.0042    0.0021, b = 1.0000   -1.8669    0.8752
		//out[i] = 0.0021*in[i] + 0.0042*in_t1[i] + 0.0021*in_t2[i] + 1.8669*out_t1[i] - 0.8752*out_t2[i];

		// Save history
		in_t1[i] = in[i];
		in_t2[i] = in_t1[i];

		out_t1[i] = out[i];
		out_t2[i] = out_t1[i];
	}

	return 0;
}

/*int FilterTools::LowPassFilter(dvector6& in, dvector6& out)
{
	// First write the output equation: Y = CXn + Bu
	out = C*stateVar6 + B*in;

	// Second write the state vector equation: Xn+1= AXn + Bu
	stateVar6 = A*stateVar6 + B*in;

	return 0;
}*/
int secOrderFilter(double in[6], double out[6])
{
	for(int i=0;i<6;i++)
	{
		out[i]=in[i];
	}
	return 0;
}
/* int secOrderFilter(dvector6& in, dvector6& out)
{
	//out = in;
	// Second Order Implementation
	// A second order recursive filter implementation is of the type:
	// b0y + b1y1 b2y2 = a0x + a1x1 + a2x2
	// y = (a0x + a1x1 + a2x2 - b1y1 - b2y2)/b0
	//
	// In the first iteration x1, x2 are 0 and y1 and y2 are 0.

 for(int i=0;i<6;i++)
	{
		// Main recursive 2nd order equation
		out[i] = (a0*in[i]+a1*in_t1(i)+a2*in_t2(i)-b1*out_t1(i)-b2*out_t2(i))/b0;

		// Save history
		in_t1(i) = in[i];
		in_t2(i) = in_t1[i];

		out_t1(i) = out[i];
		out_t2(i) = out_t1[i];
	}

	return 0;
}*/

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
