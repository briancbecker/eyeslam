/*==================================================================================

    Filename:  fpstracker.h

    Copyright 2007 Brian C. Becker
                   Robotics Laboratory
                   University of Central Florida
                   http://robotics.ucf.edu

    Program Contents:
    -------------------------------------------------------------------------------
    How often have you wanted to keep track of how many fps you are have
    averaged over the past X amount of time? This class does it for you 
    easily!
    -------------------------------------------------------------------------------

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA

==================================================================================*/
#ifndef FPSTRACKER_H
#define FPSTRACKER_H

#include <stdlib.h>
#include <time.h>
#include "queue.h"

////////////////////////////////////////////////////////////////////////////////
///
///   \class FPSTracker
///   \brief Tracks timing on anything that happens on a cycle, most notably FPS
///   
///   FPS tracker keeps track of how many times something happens per second
///   (such as frames per second, waha!). The thing that is happening on a
///   variable interval is referred to as a "click" like the click of a stop
///   watch. 
///
///   A history is used to smooth out variations and average out the events to
///   get a better result. You set the max history during the creation of the
///   object. Until the history is filled, the values might fluctuate more than
///   normal => then things should stabilize more. To use this function, first
///   create the object with a history and then call click. Click will return
///   the current frames (or whatever you are timing) per second.
///
////////////////////////////////////////////////////////////////////////////////
class FPSTracker
{
public:
    /// Begin tracking frames (or whatever), keeping a history of maxClicks
    FPSTracker(int maxClicks = 30) { mStart = clock(); mMaxClicks = maxClicks; }
    ~FPSTracker() { reset(); }

    /// Reset the times and clear the history
    void reset() { mTimes.clear(); mStart = clock(); }

    /// Click does two things, adds a new time to the history and calculates the average FPS
    /// over the last maxClicks history
    double click()
    {
        int end, start = clock();
        mTimes.enqueue(start);
        if ((int)mTimes.size() > mMaxClicks)
        {
            return 1000.0 / (((double)start - mTimes.dequeue()) / (mTimes.size() + 1));
        }
        else if (mTimes.size() > 1)
        {
            mTimes.get(0, end);
            return 1000.0 / (((double)start - end) / mTimes.size());
        }
        return 0;
    }

    /// Returns the FPS over the past history
    double getFPS() const 
    { 
        int start, end;
		if (mTimes.size() < 2)
			return 0;
        mTimes.get(mTimes.size() - 1, start);
        mTimes.get(0, end);
        if (mTimes.size() > 1) 
        { 
            return 1000.0 / (((double)start - end) / mTimes.size());
        } 
        return 0; 
    }

    /// Get the total elapsed time since constructor or reset
    int getElapsedTime() const { return clock() - mStart; }

private:
    int mMaxClicks;             ///< Max history
    int mStart;                 ///< Initial start time
    Queue <int> mTimes;         ///< Queue of all times 
};

#endif // FPSTRACKER_H
/*  End of File */
