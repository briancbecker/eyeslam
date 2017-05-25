////////////////////////////////////////////////////////////////////////////////
///
///   Filename: filterab.cpp
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

#include "filterab.h"

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Loads a specified filter from a file
///
///   \param filename The name of the file to load with the A and B vectors
///
////////////////////////////////////////////////////////////////////////////////
FilterAB::FilterAB()
{

};
FilterAB::FilterAB(const char * filename) :
 x(CIRCULAR_ARRAY_RECENT_FIRST, HISTORY_LENGTH), y(CIRCULAR_ARRAY_RECENT_FIRST, HISTORY_LENGTH)
{
	double len, num;
	FILE *fp = fopen(filename, "rb");
	if (fp)
	{
		// Load A vector of the filter
		fscanf(fp, "%lf\n", &len);
		for (int i = 0; i < len; i++)
		{
			num = 0;
			fscanf(fp, " %lf", &num);
			mA.add(num);
		}

		// Load B vector of the filter
		fscanf(fp, "\n%lf\n", &len);
		for (int i = 0; i < len; i++)
		{
			num = 0;
			fscanf(fp, "%lf", &num);
			mB.add(num);
		}
		fclose(fp);
	}
	mOut = 0;
}
void FilterAB::Init(const char * filename) 
{
	double len, num;
	FILE *fp = fopen(filename, "rb");
	if (fp)
	{
		// Load A vector of the filter
		fscanf(fp, "%lf\n", &len);
		for (int i = 0; i < len; i++)
		{
			num = 0;
			fscanf(fp, " %lf", &num);
			mA.add(num);
		}

		// Load B vector of the filter
		fscanf(fp, "\n%lf\n", &len);
		for (int i = 0; i < len; i++)
		{
			num = 0;
			fscanf(fp, "%lf", &num);
			mB.add(num);
		}
		fclose(fp);
	}
	mOut = 0;
}



////////////////////////////////////////////////////////////////////////////////
///
///   \brief Applies filter to next sample in the signal
///
///   \param in Unfiltered signal to apply the filter to
///
///   \return The current value after filtering
///
////////////////////////////////////////////////////////////////////////////////
double FilterAB::filter(double in)
{
	x.add(in);

	int lenA = mA.size() - 1 > y.size() ? y.size() : mA.size() - 1;
	int lenB = mB.size() > x.size() ? x.size() : mB.size();

	double out = 0;
	for (int i = 0; i < lenB; i++)
	{
		out += mB[i]*x[i];
	}

	// First A is coefficient of current output
	for (int i = 0; i < lenA; i++)
	{
		out -= mA[i+1]*y[i];
	}

	out /= mA[0];

	y.add(out);

	mOut = out;

	return out;
 }
