/*==================================================================================

    Filename:  cvsegmenttracker.cpp

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
#include "cvsegmenttracker.h"
#include <limits.h>

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Constructor.
///
////////////////////////////////////////////////////////////////////////////////////
CvSegmentTracker::CvSegmentTracker()
{
    mNumTracking = 1;
    mMinDist = 20;
    mMaxTime = 5;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Copy constructor.
///
////////////////////////////////////////////////////////////////////////////////////
CvSegmentTracker::CvSegmentTracker(const CvSegmentTracker &another)
{
    mNumTracking = 1;
    mMinDist = 20;
    mMaxTime = 5;
    *this = another;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Destructor.
///
////////////////////////////////////////////////////////////////////////////////////
CvSegmentTracker::~CvSegmentTracker(){}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Set the maximum number of segments to look for.
///
///   \param num Max number of segments to track.
///
////////////////////////////////////////////////////////////////////////////////////
void CvSegmentTracker::setNumTracking(const unsigned int num)
{
    if(num > 0)
        mNumTracking = num;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Clear out any tracked segments.
///
////////////////////////////////////////////////////////////////////////////////////
void CvSegmentTracker::clear() { mTracked.clear(); }

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Set the minimum distance away a segment must be to qualify for
///   matching a tracked segment.
///
///   \param dist The minimum allowed distance for normal matching between 
///               a tracked segment and new segment.
///
////////////////////////////////////////////////////////////////////////////////////
void CvSegmentTracker::setMinDist(const unsigned int dist)
{
    if(dist > 0)
        mMinDist = dist;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets the maximum time a tracked segment can go without an update
///   before it is matched to the nearest segment of same type outside the
///   minDistance() value.
///
///   \param time Max time of no updates before forced update of tracked segment.
///
////////////////////////////////////////////////////////////////////////////////////
void CvSegmentTracker::setMaxTime(const unsigned int time)
{
    if(time > 0)
        mMaxTime = time;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Update tracked segments from the new segments array.
///
///   If this is the first time the function is called, the function finds
///   the numTracking() largest segments in the array.  Each additional call
///   attempts to match eached tracked segment to a new segment if it
///   is within a threshold distance, and is the shortest distance.  Finally,
///   a last pass is done to check for tracked segments that have not updated
///   in a number of seconds, and if they haven't they are updated with the
///   nearest segment.
///
///   \param segments New segments to updated tracked segments with.
///
///   \return Total number of tracked segments updated.
///
////////////////////////////////////////////////////////////////////////////////////
unsigned int CvSegmentTracker::track(const Array<CvSegment> &segments)
{
    int total = 0;
    //  Have not tracked anything
    if(mTracked.size() == 0)
    {
        CvSegment *ptr = segments.ptr();
        int i = 0;
        for(i = 0; i < mNumTracking && segments.size(); i++, ptr++)
        {
            mTracked.push(CvTrackedSegment(*ptr));
            total++;
        }
        //  If there are still more, get the largest
        if(segments.size() > mNumTracking)
        {
            for(i = 0; i < segments.size(); i++, ptr++)
            {
                CvTrackedSegment *tPtr = mTracked.ptr();
                for(int j = 0; j < mTracked.size(); j++)
                {
                    if(ptr->area > tPtr->area)
                        *tPtr = *ptr;
                }
            }
        }

        return total;
    }

    //  Copy the segment data to speed up the search 
    mSegments = segments;
    mUpdated.resize(mTracked.size());
    //  Set to false
    memset(mUpdated.ptr(), 0, sizeof(bool)*mUpdated.size());
    //  Match up the tracked segments to the current segments
    CvTrackedSegment *tPtr = mTracked.ptr();
    bool *uPtr = mUpdated.ptr();
    CvSegment *sPtr = NULL;
    for(int i = 0; i < mTracked.size(); i++, tPtr++, uPtr++)
    {
        CvSegment *sPtr = mSegments.ptr();
        //  Find the closest segment
        int minDist = INT_MAX;
        int index = -1;
        for(int j = 0; j < mSegments.size(); j++, sPtr++)
        {
            if(sPtr->valid == false)
                continue;

            int distance = tPtr->segmentDistance(*sPtr);
            if(distance < minDist && distance < minDist)
            {
                index = j;
                minDist = distance;
            }
        }
        //  A match was found!
        if(index >= 0)
        {
            total++;
            *tPtr = mSegments[index];
            mSegments[index].valid = false;
            *uPtr = true;
        }
    }

    //  If we haven't found the minimum to 
    //  track, add the largest segment that is
    //  still valid
    if(mTracked.size() < mNumTracking)
    {
        bool done = false;
        while(!done)
        {
            int index = -1;
            int maxArea = 0;
            sPtr = mSegments.ptr();
            for(int i = 0; i < mSegments.size(); i++)
            {
                if(sPtr->valid && sPtr->area > maxArea)
                {
                    index = i;
                    maxArea = sPtr->area;
                }
            }
            //  If we found a new segment add it
            if(index >= 0)
            {
                total++;
                mTracked.push(mSegments[index]);
                mSegments[index].valid = false;
                //  If we have reached our minimum
                //  stop
                if(mTracked.size() == mNumTracking)
                    done = true;
            }
            else
            {
                //  Nothing new found, stop
                done = true;
            }
        }
    }

    //  Do one final pass to see if there are any tracked segments
    //  that have not updated in a while, and match them to the largest
    //  segment that hasn't been given to another tracked segment
    tPtr = mTracked.ptr();
    uPtr = mUpdated.ptr();
    sPtr = NULL;
    for(int i = 0; i < mTracked.size(); i++, tPtr++, uPtr++)
    {
        if(*uPtr == true)
            continue;

        if((int)(time(NULL)) - (int)tPtr->updateTime() < mMaxTime)
            continue;

        CvSegment *sPtr = mSegments.ptr();
        //  Find the closest segment
        int minDist = INT_MAX;
        int index = -1;
        for(int j = 0; j < mSegments.size(); j++, sPtr++)
        {
            if(sPtr->valid == false)
                continue;

            int distance = tPtr->segmentDistance(*sPtr);
            if(distance < minDist)
            {
                index = j;
                minDist = distance;
            }
        }
        //  A match was found!
        if(index >= 0)
        {
            total++;
            *tPtr = mSegments[index];
            mSegments[index].valid = false;
            *uPtr = true;
        }
    }

    return total;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \return Number of segments being tracked (max).
///
////////////////////////////////////////////////////////////////////////////////////
unsigned int CvSegmentTracker::numTracking() const { return mNumTracking; }

////////////////////////////////////////////////////////////////////////////////////
///
///   \return Minimum distance required for normal matching of segments.
///
////////////////////////////////////////////////////////////////////////////////////
unsigned int CvSegmentTracker::minDistance() const { return (unsigned int)(mMinDist); }

////////////////////////////////////////////////////////////////////////////////////
///
///   \return Max time allowed without update before a segment is matched to 
///   the next closest segment outside of the minimum distance.
///
////////////////////////////////////////////////////////////////////////////////////
unsigned int CvSegmentTracker::maxTime() const { return (unsigned int)(mMaxTime); }


////////////////////////////////////////////////////////////////////////////////////
///
///   \return Array of tracked segments.
///
////////////////////////////////////////////////////////////////////////////////////
Array<CvTrackedSegment> CvSegmentTracker::getSegments() const { return mTracked; }

////////////////////////////////////////////////////////////////////////////////////
///
///   \return Pointer to array of tracked segments.
///
////////////////////////////////////////////////////////////////////////////////////
Array<CvTrackedSegment> *CvSegmentTracker::getSegmentsPtr() const { return (Array<CvTrackedSegment> *)(&mTracked); }

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets equal to.
///
////////////////////////////////////////////////////////////////////////////////////
CvSegmentTracker &CvSegmentTracker::operator=(const CvSegmentTracker &another)
{
    if(this != &another)
    {
        mNumTracking = another.mNumTracking;
        mTracked = another.mTracked;
        mMinDist = another.mMinDist;
        mMaxTime = another.mMaxTime;
    }

    return *this;
}

/* End of File */
