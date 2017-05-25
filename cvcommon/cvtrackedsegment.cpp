/*==================================================================================

    Filename:  cvtrackedsegment.cpp

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
#include "cvtrackedsegment.h"
#include <math.h>

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Constructor.
///
////////////////////////////////////////////////////////////////////////////////////
CvTrackedSegment::CvTrackedSegment()
{
    mDirection = mDistance = 0;
    mSpeed = 0;
    mHistory.reserve(15); // frames
    mUpdateTime = 0;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Copy constructor.
///
////////////////////////////////////////////////////////////////////////////////////
CvTrackedSegment::CvTrackedSegment(const CvSegment &seg)
{
    mDirection = mDistance = 0;
    mUpdateTime = 0;
    *this = seg;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Copy constructor.
///
////////////////////////////////////////////////////////////////////////////////////
CvTrackedSegment::CvTrackedSegment(const CvTrackedSegment &another)
{
    mDirection = mDistance = 0;
    mUpdateTime = 0;
    *this = another;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Destructor.
///
////////////////////////////////////////////////////////////////////////////////////
CvTrackedSegment::~CvTrackedSegment()
{
    mHistory.destroy();
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Set the number of past values to use for calculating tracking
///    values.
///
///   \param size Number of past segment positions to use.
///
////////////////////////////////////////////////////////////////////////////////////
void CvTrackedSegment::setHistorySize(const unsigned int size)
{
    mHistory.resize(size);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Updates the current values of the segment and recalculate tracked
///   values.
///
///   \param another The current segment values.
///
////////////////////////////////////////////////////////////////////////////////////
void CvTrackedSegment::update(const CvSegment &another)
{
    *this = another;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \return Direction in degrees the segment has been moving over the past
///   number of updates set by the history size.
///
////////////////////////////////////////////////////////////////////////////////////
double CvTrackedSegment::direction() const { return mDirection; }
////////////////////////////////////////////////////////////////////////////////////
///
///   \return Distance the segment has traveled over the past
///   number of updates set by the history size.
///
////////////////////////////////////////////////////////////////////////////////////
double CvTrackedSegment::distance() const { return mDistance; }
////////////////////////////////////////////////////////////////////////////////////
///
///   \return Speed in pixels/frame the segment has been moving over the past
///   number of updates set by the history size.
///
////////////////////////////////////////////////////////////////////////////////////
double CvTrackedSegment::speed() const { return mSpeed; }
////////////////////////////////////////////////////////////////////////////////////
///
///   \return Size of history used for tracking calculations.
///
////////////////////////////////////////////////////////////////////////////////////
unsigned int CvTrackedSegment::historySize() const { return mHistory.reserved(); }
////////////////////////////////////////////////////////////////////////////////////
///
///   \return Time of last update in seconds.
///
////////////////////////////////////////////////////////////////////////////////////
unsigned int CvTrackedSegment::updateTime() const { return mUpdateTime; }

////////////////////////////////////////////////////////////////////////////////////
///
///   \return A CvSegment.
///
////////////////////////////////////////////////////////////////////////////////////
CvTrackedSegment::operator const CvSegment() const
{
    CvSegment s;

    s.x = this->x;
    s.y = this->y;
    s.area = this->area;
    s.color = this->color;
    s.density = this->density;
    s.aspectRatio = this->aspectRatio;
    s.perimeter = this->perimeter;
    s.compactness = this->compactness;
    s.valid = this->valid;
    s.top  = this->top;
    s.bottom = this->bottom;
    s.left = this->left;
    s.right = this->right;
    s.rect = this->rect;
    s.rle = this->rle;

    return s;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Updates the current values of the segment and recalculate tracked
///   values.
///
///   \param another The current segment values.
///
////////////////////////////////////////////////////////////////////////////////////
CvTrackedSegment &CvTrackedSegment::operator=(const CvSegment &seg)
{
    //  Save the segment values to current values
    x = (int)((x*area + seg.x*seg.area)/(double)(area + seg.area));
    y = (int)((y*area + seg.y*seg.area)/(double)(area + seg.area));
    
    area = (int)((area + seg.area)/2.0);
    color = seg.color;
    perimeter = (int)((perimeter + seg.perimeter)/2.0);
    compactness = seg.compactness;
    valid = seg.valid;
    top  = (top + seg.top)/2;
    bottom = (bottom + seg.bottom)/2;
    left = (left + seg.left)/2;
    right = (right + seg.right)/2;
    rect.x = left;
    rect.y = top;
    rect.width = right - left;
    rect.height = bottom - top;
    aspectRatio = 100*(rect.width)/(rect.height > 0 ? rect.height : 1);
    if(rect.height && rect.width)
        density = 100*area/(rect.height*rect.width);
    else
        density = 0;
    
    rle = seg.rle;

    //  Add to the history
    mHistory.add(cvPoint(seg.x, seg.y));

    //  Update statistics...
    double sumx,
           sumy,
           sumx2,
           sumxy,
           num,
           sumYDiffs,
           sumXDiffs;
    double slope;

    sumx = sumy = sumx2 = sumxy = 0;
    sumYDiffs = sumXDiffs = 0;
    CvPoint *current, *prev;
    for(int i = 0; i < mHistory.size(); i++)
    {
        mHistory.get(i, &current);
        sumx = sumx + current->x;
        sumy = sumy - current->y;               //  Image coordinates have 0 at top
        sumx2 = sumx2 + current->x*current->x;
        sumxy = sumxy - current->x*current->y;  //  Image coordinates have 0 at top

        if(i > 0)
        {
            //  Remember, previous was more recently
            //  added to the array than current, since
            //  we are going backwards in time...
            sumYDiffs -= (prev->y - current->y);
            sumXDiffs += (prev->x - current->x);
        }
        prev = current;
    }
    num = mHistory.size();
    
    //  Compute the least-squares estimators.                                     
    slope = (sumxy - ((sumx * sumy)/num)) / (.00000001 + sumx2-(sumx*sumx/num));
    
    if(num >= mHistory.reserved())
    {
        mDistance = sqrt(sumYDiffs*sumYDiffs + sumXDiffs*sumXDiffs);
        if(mDistance > 0)
        {
            mDirection = atan(slope)*57.2957795;    //  Direction in degrees.

            
            //  Convert to a number between [-180,180]
            if(sumYDiffs > 0 && sumXDiffs < 0)
            {
                mDirection += 180; 
            }
            else if(sumYDiffs < 0 && sumXDiffs < 0)
            {
                mDirection -= 180;
            }
        }
        mSpeed = mDistance/num;
    }

    mUpdateTime = (unsigned int)(time(NULL));

    return *this;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets equal to.
///
////////////////////////////////////////////////////////////////////////////////////
CvTrackedSegment &CvTrackedSegment::operator =(const CvTrackedSegment &another)
{
    if(this != &another)
    {
        mDirection = another.mDirection;
        mDistance = another.mDistance;
        mSpeed = another.mSpeed;
        mHistory = another.mHistory;
        mUpdateTime = another.mUpdateTime;
    }

    return *this;
}


/*  End of File */
