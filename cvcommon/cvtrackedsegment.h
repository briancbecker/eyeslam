/*==================================================================================

    Filename:  cvtrackedsegment.h

    Copyright 2007 Daniel Barber
                   Robotics Laboratory
                   University of Central Florida
                   http://robotics.ucf.edu

    Program Contents:
    -------------------------------------------------------------------------------
    Segment that also contains tracking information which is updated every time
    the current segment values are updated.  The tracked information includes
    a linear regresion of the segments direction, distance traveled, and speed 
    over a specified number of frames based on the segments centroid.
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
#ifndef _CV_TRACKED_SEGMENT_H
#define _CV_TRACKED_SEGMENT_H

#ifdef __cplusplus

#include <time.h>
#include "cvsegment.h"
#include "circulararray.h"

////////////////////////////////////////////////////////////////////////////////////
///
///   \class CvTrackedSegment
///   \brief Segment that also contains tracking information which is 
///   updated every time the current segment values are updated.  
///   The tracked information includes a linear regresion for the segments 
///   direction, distance traveled, and speed over a specified number of 
///   frames based on the segments centroid.
///
////////////////////////////////////////////////////////////////////////////////////
class CvTrackedSegment : public CvSegment
{
public:
    CvTrackedSegment();
    CvTrackedSegment(const CvSegment &seg);
    CvTrackedSegment(const CvTrackedSegment &another);
    ~CvTrackedSegment();
    void setHistorySize(const unsigned int size);
    void update(const CvSegment &another);
    double direction() const;       // Linear regression angle of movement
    double distance() const;        // Distance traveled over time
    double speed() const;           // Speed of movement
    unsigned int historySize() const;
    unsigned int updateTime() const;
    operator const CvSegment() const;
    CvTrackedSegment &operator=(const CvSegment &another);
    CvTrackedSegment &operator=(const CvTrackedSegment &another);
protected:
    double mDirection;                  ///<  Linear regression direction/angle of segement
    double mDistance;                   ///<  Distance traveled
    double mSpeed;                      ///<  Speed of travel
    CircularArray<CvPoint> mHistory;    ///<  Total history
    unsigned int mUpdateTime;           ///<  Time of last update
};

#endif
#endif

/* End of File */
