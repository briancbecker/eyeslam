/*==================================================================================

    Filename:  cvsegment.cpp

    Copyright 2007 Daniel Barber
                   Robotics Laboratory
                   University of Central Florida
                   http://robotics.ucf.edu

    Program Contents:
    -------------------------------------------------------------------------------
    Structure for storing segment information an image segmentation
    process.
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
#include "cvsegment.h"
#include <math.h>


//////////////////////////////////////////////////////////////////////////////
///
///   \brief Constructor.
///
//////////////////////////////////////////////////////////////////////////////
CvSegment::CvSegment()
{
    clear();
}

//////////////////////////////////////////////////////////////////////////////
///
///   \brief Copy constructor.
///
//////////////////////////////////////////////////////////////////////////////
CvSegment::CvSegment(const CvSegment &another){ *this = another; };

//////////////////////////////////////////////////////////////////////////////
///
///   \return 0 if they do not overlap, otherwise 1.
///
///   \param seg The segment to compare against
///
//////////////////////////////////////////////////////////////////////////////
int CvSegment::boundaryOverlap(const CvSegment &seg) const
{
    if(seg.valid == CV_INVALID_SEG)
        return 0;

    //  Check for x overlap
    if( (left <= seg.left && seg.left <= right) || 
         (seg.left <= left && left <= seg.right))
    {
        //  Check for y overlap
        if( (top <= seg.top && seg.top <= bottom) || 
         (seg.top <= top && top <= seg.bottom))
        {
            return true;
        }
    }

    return false;
}



//////////////////////////////////////////////////////////////////////////////
///
///   \return Distance between segments in pixels based on bounding box.
///
///   \param seg The segment to compare against.
///
//////////////////////////////////////////////////////////////////////////////
int CvSegment::segmentDistance(const CvSegment &seg) const
{
    if(seg.valid == CV_INVALID_SEG)
        return INT_MAX;

    if(boundaryOverlap(seg))
        return 0;

    CvSegment *smallSeg, *largeSeg;

    smallSeg = (CvSegment *)(this); largeSeg = &(CvSegment)seg;
    int x, y;

    //  Check for x overlap
    if( (seg.left <= left && left <= seg.right) || 
                (left <= seg.left && seg.left <= right))
    {
        //  Shortest distance is in y direction
        if(bottom < seg.top)            //  We are above
            return seg.top - bottom;
        else                            //  We are below
            return seg.bottom - top;
    }

    //  Check for y overlap
    if( (seg.top <= top && top <= seg.bottom) || 
                (top <= seg.top && seg.top <= bottom))
    {
        //  Shortest distance is in x direction
        if(right < seg.left)            //  We are on the left
            return seg.left - right;
        else                            //  We are on the right
            return left - seg.right;

    }

    //  There is no overlapping in x or y so...
    if(bottom < seg.top) //  We are above
    {
        y = seg.top - bottom;
        //  Shortest distance is in x direction
        if(right < seg.left)            //  We are on the left
            x =  seg.left - right;
        else                            //  We are on the right
            x = left - seg.right;

        return (int)sqrt((float)x*x+y*y);
    }
    else                 //  We are below
    {
        y = seg.top - bottom;
        //  Shortest distance is in x direction
        if(right < seg.left)            //  We are on the left
            x =  seg.left - right;
        else                            //  We are on the right
            x = left - seg.right;

        return (int)sqrt((float)x*x+y*y);
    }
    
    return INT_MAX;
}


//////////////////////////////////////////////////////////////////////////////
///
///   \return distance between segment cetroids.
///
///   \param seg The segment to compare against.
///
//////////////////////////////////////////////////////////////////////////////
int CvSegment::centroidDistance(const CvSegment &seg) const
{
    if(seg.valid == CV_INVALID_SEG)
        return INT_MAX;

    int length = y - seg.y;
    int width = x - seg.x;

    return (int)sqrt((double)(length*length + width*width));
}

//////////////////////////////////////////////////////////////////////////////
///
///   \return 0 if not less than, otherwise how many pixels larger the
///   other segment is
///
///   \param seg The segment to compare against
///
//////////////////////////////////////////////////////////////////////////////
int CvSegment::operator <<(const CvSegment &seg) const
{
    if(seg.valid == CV_INVALID_SEG)
        return seg.valid;

    if(area < seg.area)
        return seg.area - area;

    else
        return 0;
}

//////////////////////////////////////////////////////////////////////////////
///
///   \return 0 if not greater than, otherwise how many pixels larger
///
///   \param seg The segment to compare against
///
//////////////////////////////////////////////////////////////////////////////
int CvSegment::operator >>(const CvSegment &seg) const
{
    if(seg.valid == CV_INVALID_SEG)
        return 0;

    if(area > seg.area)
        return area - seg.area;
    else
        return 0;
}


//////////////////////////////////////////////////////////////////////////////
///
///   \return true if the row, col is inside the boundary box
///
///   \param y The center row.
///   \param x The center col.
///   \param roi Boundary box (Region of Interest).
///
//////////////////////////////////////////////////////////////////////////////
bool CvSegment::isInside(const int row, 
                         const int col, 
                         const CvRect roi) const
{
    if( y  >= roi.y && 
        y  <= roi.y + roi.height && 
        x >= roi.x && 
        x <= roi.x + roi.width)
    {
        return true;
    }
    
    return false;
}

//////////////////////////////////////////////////////////////////////////////
///
///   \return true if the row, col is inside the boundary box
///
///   \param y The center row
///   \param x The center col
///   \param top The top row of box
///   \param bottom The bottom row of box
///   \param left The left column of box
///   \param right The right column of box
///
//////////////////////////////////////////////////////////////////////////////
bool CvSegment::isInside(const int row, 
                         const int col, 
                         const int top, 
                         const int bottom, 
                         const int left, 
                         const int right) const
{
    if(y  >= top && y  <= bottom && x >= left && x <= right)
        return true;
      
    return false;
}


//////////////////////////////////////////////////////////////////////////////
///
///   \return 0 if segment is not inside, otherwise 1.
///
///   \param seg The segment to compare against
///
//////////////////////////////////////////////////////////////////////////////
int CvSegment::isInsideBoundary(const CvSegment &seg) const
{
    //  Now check boundaries
    if(isInside(seg.bottom, seg.right, top, bottom, left, right) &&
       isInside(seg.bottom, seg.left, top, bottom, left, right)  &&
       isInside(seg.top, seg.right, top, bottom, left, right)    &&
       isInside(seg.top, seg.left, top, bottom, left, right))
    {
        return true;
    }
    
    return false;
}

//////////////////////////////////////////////////////////////////////////////
///
///   \brief Merges the segment data with the current segment, then sets
///   the segment to invalid.
///
//////////////////////////////////////////////////////////////////////////////
CvSegment &CvSegment::merge(CvSegment &seg)
{
    //  Verify that we have a valid segment
    //  than update poitners
    if( valid == CV_INVALID_SEG    ||
        seg.valid == CV_INVALID_SEG)
        return *this;

    //  Now update statistics
    //  Centroid update
    x = (int)((x*area + seg.x*seg.area)/(double)(area + seg.area));
    y = (int)((y*area + seg.y*seg.area)/(double)(area + seg.area));
    //  Bounding box update
    top = (top < seg.top) ? top : seg.top;
    bottom = (bottom > seg.bottom) ? bottom : seg.bottom;
    left = (left < seg.left) ? left : seg.left;
    right = (right > seg.right) ? right : seg.right;
    //  Bounding box update
    rect.y = top;
    rect.x = left;
    rect.height = bottom - top > 0 ? bottom - top : 1;
    rect.width = right - left > 0 ? right - left : 1;
    //  Other values
    area += seg.area;
    perimeter += seg.perimeter;
    if(rect.height && rect.width)
        density = 100*area/(rect.height*rect.width);
    aspectRatio = 100*(rect.width)/(rect.height > 0 ? rect.height : 1);
    compactness = (int)(100*perimeter*perimeter/(4*CV_PI*area));
    valid = 1;

    rle += seg.rle;
    
    //  Clear the segment since it has
    //  been conusmed..
    seg.clear();
    seg.valid = CV_INVALID_SEG;

    return *this;
}

//////////////////////////////////////////////////////////////////////////////
///
///   \brief Equals operator.
///
//////////////////////////////////////////////////////////////////////////////
CvSegment &CvSegment::operator=(const CvSegment &another)
{
    if(this != &another)
    {
        y = another.y;
        x = another.x;
        area = another.area;
        color = another.color;
        density = another.density;
        aspectRatio = another.aspectRatio;
        perimeter = another.perimeter;
        compactness = another.compactness;
        valid = another.valid;
        top = another.top;
        bottom = another.bottom;
        left = another.left;
        right = another.right;
        rect = another.rect;
        rle = another.rle;
		fx = another.fx;
		fy = another.fy;
    }
    return *this;
}

//////////////////////////////////////////////////////////////////////////////
///
///   \brief Compare area size.
///
//////////////////////////////////////////////////////////////////////////////
bool CvSegment::operator >(const CvSegment &seg) const
{
    return (area > seg.area) ? true : false;
}

//////////////////////////////////////////////////////////////////////////////
///
///   \brief Compare area size.
///
//////////////////////////////////////////////////////////////////////////////
bool CvSegment::operator >=(const CvSegment &seg) const
{
    return (area >= seg.area) ? true : false;
}

//////////////////////////////////////////////////////////////////////////////
///
///   \brief Compare area size.
///
//////////////////////////////////////////////////////////////////////////////
bool CvSegment::operator <(const CvSegment &seg) const
{
    return (area < seg.area) ? true : false;
}

//////////////////////////////////////////////////////////////////////////////
///
///   \brief Compare area size.
///
//////////////////////////////////////////////////////////////////////////////
bool CvSegment::operator <=(const CvSegment &seg) const
{
    return (area <= seg.area) ? true : false;
}

//////////////////////////////////////////////////////////////////////////////
///
///   \brief Clears values of internal data members.
///
//////////////////////////////////////////////////////////////////////////////
void CvSegment::clear()
{
    valid = 1;
	fy = fx = 0.0;
    x = y = area = color = density = 0;
    aspectRatio = perimeter = compactness = 0;
    rect.height = rect.width = rect.x = rect.y = 0;
    top = left = right = bottom = 0;
    rle.clear();
}



/* End of File */
