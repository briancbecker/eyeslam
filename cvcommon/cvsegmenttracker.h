/*==================================================================================

    Filename:  cvsegmenttracker.h

    Copyright 2007 Daniel Barber
                   Robotics Laboratory
                   University of Central Florida
                   http://robotics.ucf.edu

    Program Contents:
    -------------------------------------------------------------------------------
    Attempts to keep track of a color segments over time.
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
#ifndef _CV_SEGMENT_TRACKER_H
#define _CV_SEGMENT_TRACKER_H

#ifdef __cplusplus

#include "cvtrackedsegment.h"
#include "array.h"


////////////////////////////////////////////////////////////////////////////////////
///
///   \class CvSegmentTracker
///   \brief Attempts to track segments based on changes in distance between
///   frames, and calculates statistics about them.
///
////////////////////////////////////////////////////////////////////////////////////
class CvSegmentTracker
{
public:
    CvSegmentTracker();
    CvSegmentTracker(const CvSegmentTracker &another);
    ~CvSegmentTracker();
    void clear();
    void setMinDist(const unsigned int dist);
    void setMaxTime(const unsigned int time);
    void setNumTracking(const unsigned int num);
    unsigned int track(const Array<CvSegment> &segments);
    unsigned int numTracking() const;
    unsigned int minDistance() const;
    unsigned int maxTime() const;
    Array<CvTrackedSegment> getSegments() const;
    Array<CvTrackedSegment> *getSegmentsPtr() const;
    CvSegmentTracker &operator=(const CvSegmentTracker &another);
protected:
    double mMinDist;                    ///<  Minimum distance
    int mMaxTime;              ///<  Maximum time to allow before updating
    int mNumTracking;          ///<  Max number to track
    Array<CvTrackedSegment> mTracked;   ///<  The tracked segments
    Array<CvSegment> mSegments;         ///<  Temp storage of segments
    Array<bool> mUpdated;               ///<  Keeps tracked of segments that have updated
};

#endif

#endif

/* End of File */
