////////////////////////////////////////////////////////////////////////////////
///
///   Filename: filterab.h
/// 
///   Copyright (C) 2010-2012   Brian C. Becker             www.BrianCBecker.com
///   License: LGPL             RI @ CMU                          www.ri.cmu.edu
///                             Medical Instrumentation Lab       Micron Project
///
///   Description: A generic signal processing filter function 
///   --------------------------------------------------------------------------
///	  Implements the "filter" function from Matlab for low/high/band pass
///	  filtering a temporal signal.
///
////////////////////////////////////////////////////////////////////////////////

#include "array.h"
#include "circulararray.h"

#ifndef FILTERAB_H
#define FILTERAB_H

/// Default history length preallocated for past inputs/outputs
#define HISTORY_LENGTH 10

////////////////////////////////////////////////////////////////////////////////
/// 
///   \class FilterAB
///   \brief A generic signal processing filter with custom A & B vectors
///   
///	  Similar to the "filter" function in Matlab, this class takes in custom A
///	  and B vectors to filter a signal. For instance, use "butter" in MATLAB to
///	  create Butterworth filters. A and B vectors are specified by a file whose
///	  format is
///	  
///	  <Length of A Vector>
///	  <A Vector>
///	  <Length of B Vector>
///	  <B Vector>
///	  
///	  For example:
///	  
///	  2
///	  1.000000000000000  -0.881618592363189
///	  2 0.059190703818405   0.059190703818405
///	  
///	  Is a 2nd order Butterworth low-pass filter. New values are filtered by
///	  calling the filter function. If you want to return the current value of
///	  the filtered signal, you can call the value function. 
///   
////////////////////////////////////////////////////////////////////////////////
class FilterAB
{
public:
	FilterAB();
	FilterAB(const char * filenames);
	void Init(const char * filename);
	~FilterAB() {}

	/// Filters the latest value from a signal
	double filter(double in);
	
	/// Returns last filtered value from the signal
	const double value() { return mOut; }

	/// overrride filtered value..
	void setValue(double val) { mOut = val; }

protected:
	/// Stores the B vector for the filter
	Array<double> mB;
	/// Stores the A vector for the filter
	Array<double> mA;

	/// Past inputs of the signal
	CircularArray<double> x;
	/// Past outputs of the signal
	CircularArray<double> y;

	/// Current value of filtered signal
	double mOut;

};


#endif // FILTERAB_H