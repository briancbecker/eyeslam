/*==================================================================================

    Filename:  cvsegment.h

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
#ifndef _CV_SEGMENT_H
#define _CV_SEGMENT_H

#ifdef __cplusplus

#include <cv.h>
#include "cvrle.h"
#include "array.h"

#define CV_INVALID_SEG 0         ///<  Invalid segment

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \class CvSegment
///  \brief Segment/region found by the the CvFastSegmentation class.  It 
///  contains informatoin about the segment.
///
////////////////////////////////////////////////////////////////////////////////////
class CvSegment
{
public:
    CvSegment();
    CvSegment(const CvSegment &another);
    int y;           ///<  Centroid row
    int x;           ///<  Centroid column
	float fy;          ///<  Centroid row in floating point (increased accuracy)
	float fx;          ///<  Centroid column in floating piont (increased accuracy)
    int area;        ///<  Size in pixels
    int color;       ///<  Color value of segment (depends on color classifier)
    int density;     ///<  Percentage of roi filled with pixels
    int aspectRatio; ///<  Aspect ratio of segment/bounding box
    int perimeter;   ///<  Perimeter of the image segment
    int compactness; ///<  Compactness (circle is most compact (100))
    int valid;       ///<  Flag for ignoring the segment (turning off)
    int top;         ///<  Top row of ROI (rect.y)
    int bottom;      ///<  Bottom row of ROI (rect.y + rect.height)
    int left;        ///<  Smallest column of ROI (rect.x)
    int right;       ///<  Farthest column of ROI (rect.x + rect.width)
    CvRect rect;     ///<  Bounding Box / ROI (Region of Interest) 
    Array<CvRLE> rle;///<  Run length encodings belonging to the segment
    int boundaryOverlap(const CvSegment &seg) const;   ///<  If bounding box overlaps, returns true
    int isInsideBoundary(const CvSegment &seg) const;  ///<  Is the segments boundary box complete inside of ours
    int segmentDistance(const CvSegment &seg) const;   ///<  Returns distances between bounding boxes, if 0, then there is overlap
    int centroidDistance(const CvSegment &seg) const;  ///<  Returns distance between centroids of segments
    int operator >> (const CvSegment &seg) const;      ///<  If greater than, returns percentage greater, otherwise 0
    int operator << (const CvSegment &seg) const;      ///<  If less than returns percentage of larger segment, otherwise 0
    bool operator > (const CvSegment &seg) const;      ///<  Return true if area is greater.
    bool operator >= (const CvSegment &seg) const;     ///<  Return true if area is greater than or equal.
    bool operator < (const CvSegment &seg) const;      ///<  Return true if less than area.
    bool operator <=(const CvSegment &seg) const;      ///<  Return true if less than or equal area.
    void clear();
    CvSegment &merge(CvSegment &seg);
    CvSegment &operator=(const CvSegment &another);
protected:
    bool isInside(const int y,
                  const int x,
                  const CvRect roi) const;
    bool isInside(const int y,
                  const int x,
                  const int top,
                  const int bottom,
                  const int left,
                  const int right) const;
};

//////////////////////////////////////////////////////////////////////////////
///
///   \brief Draws the segment with the specified color.
///
///   \param dest Destination image.
///   \param seg The segment to draw.
///   \param color The color to draw.  If grayscale image, color is
///                converted to grayscale.
///
//////////////////////////////////////////////////////////////////////////////
inline int cvDrawSegment(IplImage *dest, 
                         const CvSegment seg, 
                         CvScalar color /* ex: CV_RGB(255, 0, 0) */ )
{
    assert(dest &&
           dest->depth == IPL_DEPTH_8U &&
           dest->nChannels <= 3 &&
           dest->nChannels >= 1);

    int width = (dest->roi) ? dest->roi->width : dest->width;
    int height = (dest->roi) ? dest->roi->height : dest->height;
    CvRLE *rle = seg.rle.ptr();
    if(seg.x < 0 || seg.y < 0 || seg.valid == CV_INVALID_SEG)
        return 0;

    for(int i = 0; i < seg.rle.size(); i++, rle++)
    {
        if(rle->y >= 0 && rle->y < height)
        {
            for(int x = rle->x; x < (rle->x + rle->width) && x >= 0 && x < width; x++)
            {
                unsigned char *base ((uchar *)(dest->imageData + dest->widthStep*rle->y));
                if(dest->nChannels == 1)
                {
                    base[x] = (unsigned char)(0.212671*color.val[2] + 
                                              0.715160*color.val[1] + 
                                              0.072169*color.val[0]);
                }
                else if(dest->nChannels == 3)
                {
                    int off = x*3;
                    if(dest->channelSeq[0] == 'R')
                    {
                        base[off] =     (unsigned char)color.val[2];
                        base[off + 1] = (unsigned char)color.val[1];
                        base[off + 2] = (unsigned char)color.val[0];
                    }
                    else
                    {
                        base[off] =     (unsigned char)color.val[0];
                        base[off + 1] = (unsigned char)color.val[1];
                        base[off + 2] = (unsigned char)color.val[2];
                    }
                    
                }
            }
        }
    }

    return 1;
}

//////////////////////////////////////////////////////////////////////////////
///
///   \brief Draws a box for the segment with the specified color.
///
///   \param dest Destination image.
///   \param seg The segment to draw.
///   \param color The color to draw.  If grayscale image, color is
///                converted to grayscale.
///   \param thickness Thickness of box, if less than 0 than box is filled.
///
//////////////////////////////////////////////////////////////////////////////
inline int cvDrawSegmentBox(IplImage *dest, 
                            const CvSegment seg, 
                            CvScalar color /* ex: CV_RGB(255, 0, 0) */,
                            const int thickness = 1)
{
    if(seg.x < 0 || seg.y < 0 || seg.valid == CV_INVALID_SEG)
        return 0;
    
    CvPoint p1, p2;
    p1.x = seg.x - seg.rect.width/2; p1.y = seg.y - seg.rect.height/2;
    p2.x = seg.x + seg.rect.width/2; p2.y = seg.y + seg.rect.height/2;
    cvRectangle(dest, p1, p2, color, thickness);

    return 1;
}

//////////////////////////////////////////////////////////////////////////////
///
///   \brief Draws an ellipse for the segment with the specified color.
///
///   \param dest Destination image.
///   \param seg The segment to draw.
///   \param color The color to draw.  If grayscale image, color is
///                converted to grayscale.
///   \param thickness Thickness of box, if less than 0 than ellipse is filled.
///
//////////////////////////////////////////////////////////////////////////////
inline int cvDrawSegmentEllipse(IplImage *dest, 
                                const CvSegment seg, 
                                CvScalar color /* ex: CV_RGB(255, 0, 0) */,
                                const int thickness = 1)
{
    if(seg.x < 0 || seg.y < 0 || seg.valid == CV_INVALID_SEG)
        return 0;
    
    CvBox2D segbox;
    segbox.center.x = (float)seg.x;
    segbox.center.y = (float)seg.y;
    segbox.size.width = (float)seg.rect.width;
    segbox.size.height = (float)seg.rect.height;
    segbox.angle = 0.0f;

    cvEllipseBox(dest, segbox, color, thickness);

    return 1;
}

#endif

#endif
/* End of File */
