/*==================================================================================

    Filename:  cvfastsegmentation.h

    Copyright 2007 Daniel Barber
                   Robotics Laboratory
                   University of Central Florida
                   http://robotics.ucf.edu

    Program Contents:
    -------------------------------------------------------------------------------
    Implementation of the "Fast and Inexpensive Color Image Segmentation for 
    Interactive Robots."  Fast segmentation which uses any method of color
    classification desired.

    Paper Reference:
    James Bruce, Tucker Balch, Manuela Veloso, "Fast and Inexpensive Color Image
    Segmentation for Interactive Robots."  Proc. of IEEE International Conference
    on Intelligent Robots and Systems, 2000.
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
#ifndef _CV_FAST_SEGMENTATION_H
#define _CV_FAST_SEGMENTATION_H

#ifdef __cplusplus

#include <cv.h>
#include "cvrle.h"
#include "cvsegment.h"
#include "cvcolorclassifier.h"
#include "array.h"  //  Dynamic array structure


//////////////////////////////////////////////////////////////////////////////
///
///   \brief Sums integers over a range.
///
///   \param x Starting value.
///   \param w Width/length of range.
///
///   \return Sum of integers over range.
///
//////////////////////////////////////////////////////////////////////////////
inline int rangeSum(const int x, const int w)
{
    return(w*(2*x + w-1) / 2);
}

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \class CvFastSegmentation
///  \brief Segments regions based on color or intensity.
///
///  Required Libraries:  OpenCV
///
////////////////////////////////////////////////////////////////////////////////////
class CvFastSegmentation
{
public:
    CvFastSegmentation();
    ~CvFastSegmentation();
    virtual int segment(const IplImage *src, 
                        IplImage *dest,
                        const int thresh,
                        const int minSize);
    virtual int segment(const IplImage *src, 
                        IplImage *dest,
                        IplImage *mask,
                        const int maskval,
                        const int thresh,
                        const int minSize);
    int numSegments(const int color);
    int drawSegments(IplImage *dest);                    
    int drawMask(IplImage *dest, const int color);
    int drawBoxes(IplImage *dest, const int thickness = 1);
    int numColors() const;
    bool useCustomFunction(int (*_colorClass)( const uchar r, const uchar g, const uchar b), 
                           const int numClasses);
    int addColorClassifier( CvColorClassifier *classifier,
                            const double threshhold);
    bool setDrawColor(const int colorClass,
                      CvScalar color /* ex: CV_RGB(255, 0, 0) */);
    Array<CvSegment> *getSegments(const int color);
protected:
    int runLengthEncode(const IplImage *src,
                        const int y0,
                        const int x0,
                        const int height,
                        const int width);     ///  Run Length Encode (RLE) the image.
    int runLengthEncode(const IplImage *src,
                        const int y0,
                        const int x0,
                        const int height,
                        const int width,
                        const IplImage *mask,
                        const int maskval,
                        const int y1,
                        const int x1);     ///  Run Length Encode (RLE) the image.
    int connectComponents();               ///  Connect the RLE components
    int extractRegions(const int minSize = 0);
    int seperateRegions(const int thresh);
    int (*mColorClass)(const unsigned char r,
                       const unsigned char g,
                       const unsigned char b);///<  Custom function for color classification
    int mHeight;            ///<  Segmented image height
    int mWidth;             ///<  Segmented image width
    int mMode;              ///<  Classification mode (function pointer or classifier)
    int mColorImage;        ///<  Do we have color segments, or binary image segments?
    Array<CvRLE> mRLE;      ///<  Run Length Encoded data
    Array<CvSegment> mSegments;             ///<  All Segments
    Array<Array<CvSegment>> mColorSegments;  ///<  Color Seperated Segments
    Array<CvColorClassifier *> mColorClassifiers; ///<  Color classifiers to use 
    Array<double> mClassifierThreshholds;         ///<  Threshholds for classifiers
    Array<CvScalar> mDrawColors;                  ///<  Colors to draw colored segments with
};



#endif
#endif
/* End of file */
