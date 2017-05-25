/*==================================================================================

    Filename:  cvfastsegmentation.cpp

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
#include "cvfastsegmentation.h"
#include "cvsegment.h"
#include <limits.h>
#include <time.h>


//////////////////////////////////////////////////////////////////////////////
///
///   \brief Constructor.
///
//////////////////////////////////////////////////////////////////////////////
CvFastSegmentation::CvFastSegmentation()
{
    mColorClass = NULL;
    mMode = 0;
    mColorImage = false;
    mHeight = mWidth = 0;
}

//////////////////////////////////////////////////////////////////////////////
///
///   \brief Destructor.
///
//////////////////////////////////////////////////////////////////////////////
CvFastSegmentation::~CvFastSegmentation()
{
}


//////////////////////////////////////////////////////////////////////////////
///
///   \brief Use a custom function to classify the color values of
///   a pixel.
///
///   If this function is called, any CvColorClassifier pointers are
///   removed, and only the function is used for color classification.
///
///   \param _colorClass Function pointer which classifies an r, g, b pixel
///          as a color class, and returns the class type [1 - numClasses]
///          0 is a background pixel, and is not segmented.
///   \param numClasses Number of possible color class types the function
///          returns.
///
//////////////////////////////////////////////////////////////////////////////
bool CvFastSegmentation::useCustomFunction(int (*_colorClass)(const uchar r, const uchar g, const uchar b),
                                           const int numClasses)
{
    assert(_colorClass  && numClasses);
    this->mColorClass = _colorClass;
    this->mMode = 0;
    mColorSegments.create(numClasses);
    mDrawColors.create(numClasses);
    srand((int)time(NULL));
    for(int i = 0; i < numClasses; i++)
    {
        CvScalar col;        
        col.val[0] = rand()*255.0f/RAND_MAX;
        col.val[1] = rand()*255.0f/RAND_MAX;
        col.val[2] = rand()*255.0f/RAND_MAX;
        mDrawColors[i] = col;
    }
    //  Clear out the CvColorClassifiers
    mColorClassifiers.clear();
    mClassifierThreshholds.clear();

    return true;
}

//////////////////////////////////////////////////////////////////////////////
///
///   \brief Adds a color classifier for pixel color classification during
///   segmentation.  To add multiple classifiers, just keep calling this
///   function.  However, and custom function initially set will be lost.
///
///   \param classifier Color classifier for pixel classification.
///   \param threshold Threshold value to use for the classifier.
///
//////////////////////////////////////////////////////////////////////////////
int CvFastSegmentation::addColorClassifier(CvColorClassifier *classifier,
                                           const double threshhold)
{
    //  Disable custom function for color
    //  classification
    if(mMode == 0 || mColorClass)
    {
        mColorSegments.clear();
        mDrawColors.clear();
        mMode = 1; 
        mColorClass = NULL;
    }
    //  Add the classifier and its threshhold to
    //  the arrays containing them.
    mColorClassifiers.push(classifier);
    mClassifierThreshholds.push(threshhold);
    mColorSegments.create(mColorClassifiers.size());
    //  Add a draw color
    CvScalar col;
    col.val[0] = rand()*255.0f/RAND_MAX;
    col.val[1] = rand()*255.0f/RAND_MAX;
    col.val[2] = rand()*255.0f/RAND_MAX;
    mDrawColors.push(col);
    return (int)mColorClassifiers.size();
}

//////////////////////////////////////////////////////////////////////////////
///
///   \brief Segments the input image.  If the image is grayscale, it is
///   treated as binary data, and any non-zero pixel is treated as 1.
///   The segments found are then drawn to dest if supplied.
///
///   \param src Source image (Binary or RGB data).
///   \param dest Where to draw results.  If NULL then no drawing done.
///   \param thresh Overlap threshold.  Adds a buffer for combining 
///                 segments within a certain distance from each other.
///                 The distance is in pixels.
///   \param minSize Minimum size of segments in pixels.
///
///   \return Number of segments found.
///
//////////////////////////////////////////////////////////////////////////////
int CvFastSegmentation::segment(const IplImage *src,
                                IplImage *dest,
                                const int thresh,
                                const int minSize)
{
    //  Make sure we have a valid input
    assert( src &&
            src->depth == IPL_DEPTH_8U &&
            src->nChannels >= 1);
    
    //  Set what kind of image is being used
    if(src->nChannels == 3)
        mColorImage = true;
    else
        mColorImage = false;

	for( int i = 0; i < mColorSegments.size(); i++)
		for(int j = 0; j < mColorSegments[i].size(); j++)
			mColorSegments[i][j].clear();

	mSegments.clear();
	mRLE.clear();

    int row, col;
    //  Get the starting position in the images and
    //  the length and width of the scanning area.
    row = src->roi ? src->roi->yOffset : 0;
    col = src->roi ? src->roi->xOffset : 0;
    mWidth = src->roi ? src->roi->width : src->width;
    mHeight = src->roi ? src->roi->height : src->height;
    
    
    //  First we need to Run Length Encode the Image
    int total = 0;
    if(runLengthEncode(src, row, col, mHeight, mWidth) > 0)
    {
        if(connectComponents())
        {
            if(extractRegions(minSize))
            {
                total = seperateRegions(thresh);               
            }
        }
    }

    if(dest)
        drawSegments(dest);

    return total;
}


//////////////////////////////////////////////////////////////////////////////
///
///   \brief Segments the input image.  If the image is grayscale, it is
///   treated as binary data, and any non-zero pixel is treated as 1.
///   The segments found are then drawn to dest if supplied.
///
///   \param src Source image (Binary or RGB data).
///   \param dest Where to draw results.  If NULL then no drawing done.
///   \param mask Binary mask for eliminating regions from being
///               encoded.  Cannot be NULL and must be grayscale data
///               with 1 channel.
///   \param maskval What pixel value in mask to use for ignoring
///                  data in src.  For example, all pixels with value
///                  255 will have the location in src skipped.
///   \param thresh Overlap threshold.  Adds a buffer for combining 
///                 segments within a certain distance from each other.
///                 The distance is in pixels.
///   \param minSize Minimum size of segments in pixels.
///
///   \return Number of segments found.
///
//////////////////////////////////////////////////////////////////////////////
int CvFastSegmentation::segment(const IplImage *src,
                                IplImage *dest,
                                IplImage *mask,
                                const int maskval,
                                const int thresh,
                                const int minSize)
{
    //  Make sure we have a valid input
    assert( src && mask &&
            src->depth == IPL_DEPTH_8U &&
            src->nChannels >= 1 &&
            mask->depth == IPL_DEPTH_8U &&
            mask->nChannels == 1);
    
    //  Set what kind of image is being used
    if(src->nChannels == 3)
        mColorImage = true;
    else
        mColorImage = false;

    int row, col;
    int row1, col1;
    int maskWidth, maskHeight;
    //  Get the starting position in the images and
    //  the length and width of the scanning area.
    row = src->roi ? src->roi->yOffset : 0;
    col = src->roi ? src->roi->xOffset : 0;
    mWidth = src->roi ? src->roi->width : src->width;
    mHeight = src->roi ? src->roi->height : src->height;

    row1 = mask->roi ? mask->roi->yOffset : 0;
    col1 = mask->roi ? mask->roi->xOffset : 0;
    maskWidth = mask->roi ? mask->roi->width : mask->width;
    maskHeight = mask->roi ? mask->roi->height : mask->height;

    assert(mHeight == maskHeight && maskWidth == mWidth);
    
    //  First we need to Run Length Encode the Image
    int total = 0;
    if(runLengthEncode(src, row, col, mHeight, mWidth, mask, maskval, row1, col1) > 0)
    {
        if(connectComponents())
        {
            if(extractRegions(minSize))
            {
                total = seperateRegions(thresh);               
            }
        }
    }

    if(dest)
        drawSegments(dest);

    return total;
}

//////////////////////////////////////////////////////////////////////////////
///
///   \brief Number of segments.
///
///   \param color What color to get number of segments for.  If the input
///   image was grayscale, than by default, the number of gayscale 
///   segments is returned no matter what color value is used.
///
//////////////////////////////////////////////////////////////////////////////
int CvFastSegmentation::numSegments(const int color)
{
    if(mColorImage == 0 && mColorSegments.size()) //  Segmented binary image
    {
        return mColorSegments[0].size();
    }
    if(color > 0 && color <= (int)mColorSegments.size())
    {
        return mColorSegments[color - 1].size();
    }

    return -1;
}


//////////////////////////////////////////////////////////////////////////////
///
///   \return Array of segments for the color.
///
///   \param color What color segments to get.  If input image was binary
///                then those segments are returned regardless of the value
///                of color.
///
//////////////////////////////////////////////////////////////////////////////
Array<CvSegment> *CvFastSegmentation::getSegments(const int color)
{
    if(mColorImage == 0 && mColorSegments.size()) //  Segmented binary image
    {
        return &mColorSegments[0];
    }
    if(color > 0 && color <= (int)mColorSegments.size())
    {
        return &mColorSegments[color - 1];
    }

    return NULL;
}

//////////////////////////////////////////////////////////////////////////////
///
///   \return Number of colors being segmented.
///
//////////////////////////////////////////////////////////////////////////////
int CvFastSegmentation::numColors() const
{
    return mColorSegments.size();
}

//////////////////////////////////////////////////////////////////////////////
///
///   \brief Set what color to draw segments of the the color class type.
///
///   \param colorClass The color id to set the draw color for.
///   \param color The color to draw with.
///
///   \return True if the color is set.
///
//////////////////////////////////////////////////////////////////////////////
bool CvFastSegmentation::setDrawColor(const int colorClass, CvScalar color)
{
    if(colorClass > 0 && colorClass <= (int)mDrawColors.size())
    {
        mDrawColors[colorClass - 1] = color;
        return true;
    }
    return false;
}

//////////////////////////////////////////////////////////////////////////////
///
///   \brief Draws the segments into dest.
///
///   \param dest Where to draw the segments.
///
///   \return Number of segments drawn.
///
//////////////////////////////////////////////////////////////////////////////
int CvFastSegmentation::drawSegments(IplImage *dest)
{
    assert( dest &&
            dest->depth == IPL_DEPTH_8U &&
            dest->nChannels <= 3 && dest->nChannels >= 1 &&
            mWidth == (dest->roi ? dest->roi->width : dest->width) &&
                mHeight == (dest->roi ? dest->roi->height : dest->height));

    int numColors = (int)mDrawColors.size();
    int total = 0;
    CvScalar *colors = mDrawColors.ptr();
    for(int i = 0; i < mColorSegments.size(); i++)
    {
        CvSegment *segments = mColorSegments[i].ptr();
        for(int j = 0; j < mColorSegments[i].size(); j++, segments++)
        {
            CvRLE *rle = segments->rle.ptr();
            for(int i = 0; i < segments->rle.size(); i++, rle++)
            {
                for(int x = rle->x; x < rle->x + rle->width; x++)
                {
                    unsigned char *base ((uchar *)(dest->imageData + dest->widthStep*rle->y));
                    if(dest->nChannels == 1)
                    {
                        base[x] = 255;
                    }
                    else if(dest->nChannels == 3)
                    {
                        if(mColorImage && rle->color > 0 && rle->color <= numColors)
                        {
                            CvScalar color = colors[rle->color - 1];
                            if(dest->channelSeq[0] == 'R') // RGB
                            {
                                base[x*3] = (unsigned char)(color.val[2]);
                                base[x*3 + 1] = (unsigned char)(color.val[1]);
                                base[x*3 + 2] = (unsigned char)(color.val[0]);
                            }
                            else
                            {
                                base[x*3] = (unsigned char)(color.val[0]);
                                base[x*3 + 1] = (unsigned char)(color.val[1]);
                                base[x*3 + 2] = (unsigned char)(color.val[2]);
                            }
                        }
                        else
                        {
                            base[x*3] = 255;
                            base[x*3 + 1] = 255;
                            base[x*3 + 2] = 255;
                        }
                    }
                }
            }
            total++;
        }        
    }

    return total;
}

//////////////////////////////////////////////////////////////////////////////
///
///   \brief Draws the segments into dest.  Creates a binary image/mask.
///
///   \param dest Where to draw the segments.
///   \param color Which color class segments type to draw.
///
///   \return Number of segments drawn.
///
//////////////////////////////////////////////////////////////////////////////
int CvFastSegmentation::drawMask(IplImage *dest, const int color)
{
    assert( dest &&
            dest->depth == IPL_DEPTH_8U &&
            dest->nChannels <= 3 && dest->nChannels >= 1 &&
            mWidth == (dest->roi ? dest->roi->width : dest->width) &&
                mHeight == (dest->roi ? dest->roi->height : dest->height));

    if(mColorImage && color < 1 || color > (int)mColorSegments.size())
        return 0;

    int numColors = (int)mDrawColors.size();
    int total = 0;
    int index = mColorImage ? color - 1 : 0;
    CvSegment *segments = mColorSegments[index].ptr();
    cvSetZero(dest);
    for(int j = 0; j < mColorSegments[index].size(); j++, segments++)
    {
        CvRLE *rle = segments->rle.ptr();
        for(int i = 0; i < segments->rle.size(); i++, rle++)
        {
            for(int x = rle->x; x < rle->x + rle->width; x++)
            {
                unsigned char *base ((uchar *)(dest->imageData + dest->widthStep*rle->y));
                if(dest->nChannels == 1)
                {
                    base[x] = 255;
                }
                else if(dest->nChannels == 3)
                {
                    int off = x*3;
                    base[off] = 255;
                    base[off + 1] = 255;
                    base[off + 2] = 255;
                }
            }
        }
        total++;
    }
        
    return total;
}


//////////////////////////////////////////////////////////////////////////////
///
///   \brief Draws boxes where the segments are.
///
///   \param dest Where to draw boxes.
///   \param thickness Thickness of outline.  If negative than
///                    filled boxes are drawn.
///
///   \return Number of boxes drawn.
///
//////////////////////////////////////////////////////////////////////////////
int CvFastSegmentation::drawBoxes(IplImage *dest, 
                                  const int thickness)
{
    assert( dest &&
            dest->depth == IPL_DEPTH_8U &&
            dest->nChannels <= 3 && dest->nChannels >= 1 &&
            mWidth == (dest->roi ? dest->roi->width : dest->width) &&
                mHeight == (dest->roi ? dest->roi->height : dest->height));

    int numColors = (int)mDrawColors.size();
    int total = 0;
    CvScalar *colors = mDrawColors.ptr();
    if(mColorImage)
    {
        for(int i = 0; i < mColorSegments.size(); i++)
        {
            CvSegment *segPtr = mColorSegments[i].ptr();
            for(int j = 0; j < mColorSegments[i].size(); j++, segPtr++)
            {
                CvPoint p1, p2;
                p1.x = segPtr->left; p1.y = segPtr->top;
                p2.x = segPtr->right; p2.y = segPtr->bottom;
                cvRectangle(dest, p1, p2, colors[i], thickness);
                total++;
            }
        }
    }
    else if(mColorSegments.size() > 0)
    {
        CvSegment *segPtr = mColorSegments[0].ptr();
        for(int i = 0; i < mColorSegments[0].size(); i++,segPtr++)
        {
            CvPoint p1, p2;
            p1.x = segPtr->left; p1.y = segPtr->top;
            p2.x = segPtr->right; p2.y = segPtr->bottom;
            cvRectangle(dest, p1, p2, CV_RGB(255, 255, 255), thickness);
            total++;
        }
    }

    return total;
}


//////////////////////////////////////////////////////////////////////////////
///
///   \brief Performs run length encoding.  This reduces the size of the 
///   image for segmentation.
///
///   \param src The source image.
///   \param y0  Starting row in image.
///   \param x0 Starting column in image.
///   \param height Height of the image.
///   \param width Width of the image.
///
///   \return Number of CvRLE encoded.
///
//////////////////////////////////////////////////////////////////////////////
int CvFastSegmentation::runLengthEncode(const IplImage *src,
                                        const int y0,
                                        const int x0,
                                        const int height,
                                        const int width)
{
    int total = 0;
    mRLE.clear();
    //  Scan the image to RLE
    for(int y = y0; y < height; y++)
    {
        //  Clear the current row in the RLE map
        CvRLE rle;
        rle.y = y;     //  Current row (x and width start at 0)
        //  Get a pointer to the row
        uchar *base = ((uchar *)(src->imageData + src->widthStep*y));
        for(int x = x0; x < width; x++)
        {            
            //  Calculate the column offset based on the number
            //  of channels (color vs. grayscale)
            int offset = 0, color = 0;
            if(src->nChannels == 1)
            {
                offset = x;
                //  Classify based on intensity (binary image)
                color = base[offset] != 0 ? 1 : 0;
            }
            else
            {
                offset = x*3;
                if(mColorClass)  //  Verify color sequence
                {
                    if(src->channelSeq[0] == 'R' && src->channelSeq[1] == 'G' && src->channelSeq[2] == 'B')
                        color = mColorClass(base[offset], base[offset + 1], base[offset + 2]);
                    else
                        color = mColorClass(base[offset + 2], base[offset + 1], base[offset]);
                }
                else
                {
                    double *dPtr = mClassifierThreshholds.ptr();
                    CvColorClassifier **cPtr = mColorClassifiers.ptr();
                    unsigned char r, g, b;

                    if(src->channelSeq[0] == 'R' && src->channelSeq[1] == 'G' && src->channelSeq[2] == 'B')
                    {   
                        r = base[offset]; g = base[offset + 1]; b = base[offset + 2];
                    }
                    else
                    {
                        r = base[offset + 2]; g = base[offset + 1]; b = base[offset];
                    }
                    color = 0;
                    double maxThresh = -10.0;
                    for(int c = 0; c < mColorClassifiers.size(); c++, dPtr++, cPtr++)
                    {
                        //  Find the classified color that has the highest threshold
                        //  because it will be the most accurate
                        if((color == 0 || *dPtr > maxThresh) && (*cPtr)->classify(r, g, b, *dPtr))
                        {
                            color = c + 1;
                            maxThresh = *dPtr;
                        }
                    }
                }
            }
            //  We have hit a change in color (make sure not to simply add first pixel)
            if(rle.color != color && (y != y0 || x != x0))
            {
                //  Add to the RLE array for the row for non-background colors only
                if(rle.color != 0)
                {
                    rle.parent = total;
                    mRLE.push(rle);  
                    total++;                //  Number of RLE found
                }
                rle.color = color;  //  Set the new color
                rle.x = x;          //  Set the new starting x
                rle.width = 1;      //  Set the new width (1 pixel width)        
            }
            else
            {
                rle.width++;        //  Increase the width of the RLE segment
                rle.color = color;  //  Set the color value
            }
        }
        if(rle.width >= (width - 1) && rle.color != 0)
        {
            rle.parent = total;
            mRLE.push(rle);
            total++;
        }
    }
    return total;
}


//////////////////////////////////////////////////////////////////////////////
///
///   \brief Performs run length encoding.  This reduces the size of the 
///   image for segmentation.
///
///   \param src The source image.
///   \param mask Mask for eliminating regions
///   \param maskval What pixel value is masked.  Any mask pixel value
///                  of maskval will have that location ignored in the
///                  src.
///   \param y0  Starting row in image.
///   \param x0 Starting column in image.
///   \param height Height of the image.
///   \param width Width of the image.
///   \param y1 Starting row in mask.
///   \param x1 Starting column in mask.
///
///   \return Number of CvRLE encoded.
///
//////////////////////////////////////////////////////////////////////////////
int CvFastSegmentation::runLengthEncode(const IplImage *src,                                        
                                        const int y0,
                                        const int x0,
                                        const int height,
                                        const int width,
                                        const IplImage *mask,
                                        const int maskval,
                                        const int y1,
                                        const int x1)
{
    int total = 0;
    mRLE.clear();
    //  Scan the image to RLE
    for(int y = y0, m = y1; y < height; y++, m++)
    {
        //  Clear the current row in the RLE map
        CvRLE rle;
        rle.y = y;     //  Current row (x and width start at 0)
        //  Get a pointer to the row
        uchar *base = (uchar *)(src->imageData + src->widthStep*y);
        uchar *mbase = (uchar *)(mask->imageData + mask->widthStep*m);
        for(int x = x0, n = x1; x < width; x++, n++)
        {            
            //  Calculate the column offset based on the number
            //  of channels (color vs. grayscale)
            int offset = 0, color = 0;
            if(mbase[n] != maskval)
            {
                if(src->nChannels == 1)
                {
                    offset = x;
                    //  Classify based on intensity (binary image)
                    color = base[offset] != 0 ? 1 : 0;
                }
                else
                {
                    offset = x*3;
                    if(mColorClass)  //  Verify color sequence
                    {
                        if(src->channelSeq[0] == 'R' && src->channelSeq[1] == 'G' && src->channelSeq[2] == 'B')
                            color = mColorClass(base[offset], base[offset + 1], base[offset + 2]);
                        else
                            color = mColorClass(base[offset + 2], base[offset + 1], base[offset]);
                    }
                    else
                    {
                        double *dPtr = mClassifierThreshholds.ptr();
                        CvColorClassifier **cPtr = mColorClassifiers.ptr();
                        unsigned char r, g, b;

                        if(src->channelSeq[0] == 'R' && src->channelSeq[1] == 'G' && src->channelSeq[2] == 'B')
                        {   
                            r = base[offset]; g = base[offset + 1]; b = base[offset + 2];
                        }
                        else
                        {
                            r = base[offset + 2]; g = base[offset + 1]; b = base[offset];
                        }
                        color = 0;
                        double maxThresh = -10.0;
                        for(int c = 0; c < mColorClassifiers.size(); c++, dPtr++, cPtr++)
                        {
                            //  Find the classified color that has the highest threshold
                            //  because it will be the most accurate
                            if((color == 0 || *dPtr > maxThresh) && (*cPtr)->classify(r, g, b, *dPtr))
                            {
                                color = c + 1;
                                maxThresh = *dPtr;
                            }
                        }
                    }
                }
            }
            //  We have hit a change in color (make sure not to simply add first pixel)
            if(rle.color != color && (y != y0 || x != x0))
            {
                //  Add to the RLE array for the row for non-background colors only
                if(rle.color != 0)
                {
                    rle.parent = total;
                    mRLE.push(rle);  
                    total++;                //  Number of RLE found
                }
                rle.color = color;  //  Set the new color
                rle.x = x;          //  Set the new starting x
                rle.width = 1;      //  Set the new width (1 pixel width)        
            }
            else
            {
                rle.width++;        //  Increase the width of the RLE segment
                rle.color = color;  //  Set the color value
            }
        }
    }
    return total;
}

//////////////////////////////////////////////////////////////////////////////
///
///   \brief Connects the CvRLE compenents found by runLengthEncode.
///
///   \return Number of connected components.
///
//////////////////////////////////////////////////////////////////////////////
int CvFastSegmentation::connectComponents()
{
    int l1, l2;
    CvRLE r1, r2;
    CvRLE *map;
    int i, j, s;
    int num = (int)(mRLE.size());
    //  l2 starts on first scan line, l1 start on second scan line
    l1 = 1;
    l2 = 0;

    if(num <= 1)
        return mRLE.size();

    while(l1 < (int)mRLE.size() && mRLE[l1].y == mRLE[l2].y) l1++;

    map = mRLE.ptr();
    r1 = map[l1];
    r2 = map[l2];
    s = l1;
    while(l1 < num)
    {
        if(r1.color == r2.color)
        {
            // case 1: r2.x <= r1.x < r2.x + r2.width
            // case 2: r1.x <= r2.x < r1.x + r1.width
            if( (r2.x <= r1.x && r1.x <= r2.x + r2.width + 2) || 
                (r1.x <= r2.x && r2.x <= r1.x + r1.width + 2))
            {                
                if(r1.parent != r2.parent)
                {
					// Doig! Do a sanity check, don't connect components that aren't on adjacent lines
					if (abs(r1.y - r2.y) <= 1)
					{
						//  Union the two parents if they are different
						//  First find the termainl roots of each path up tree
						i = r1.parent;
						while(i != map[i].parent) i = map[i].parent;
						j = r2.parent;
						while(j != map[j].parent) j = map[j].parent;

						//  Union and compress paths.  Use smaller of two possible
						//  representative indexes to preserve DAG property
						if(i < j)
						{
							map[j].parent = i;
							map[l1].parent = map[l2].parent = r1.parent = r2.parent = i;
						}
						else
						{
							map[i].parent = j;
							map[l1].parent = map[l2].parent = r1.parent = r2.parent = j;
						}
					}
                }
            }
        }        
        // Move to next point where values may change
        if(r1.x + r1.width < r2.x + r2.width)
        {
            ++l1;
            if(l1 < num)
                r1 = map[l1];
            else
                ++l2;
        }
        else
        {
            ++l2;
            if(l2 < num)
                r2 = map[l2];
            else
            {
                ++l1;
            }
        }
        //  We are on the same line, must get l1 onto second scan line (higher line)
        if(r1.y <= r2.y)
        {
            while(l1 < num && map[l1].y <= r2.y ) l1++;
            r1 = map[l1];
        }
        //  Need to reset because of a gap between rows
        else if(r1.y - r2.y > 1)
        {
            r1 = map[l1];  //  Advance l1 a little
            while(l2 < num && r1.y - map[l2].y > 1) l2++;
            r2 = map[l2];
        }    
    }

    //  Now we need to compress all parent paths
    for(i = 0; i < (int)mRLE.size(); i++)
    {
        j = map[i].parent;
        if(j > i)
        {
            while(j != map[j].parent) j = map[j].parent;
            map[i].parent = j;
        }
        else
            map[i].parent = map[j].parent;
    }

    return (int)mRLE.size();

}

//////////////////////////////////////////////////////////////////////////////
///
///   \brief Extracts segments/regions based on connected components.
///
///   \param minSize The minimum size for a component to be considered 
///                  valid.  Size in pixels.
///
///   \return Number of regions.
///
//////////////////////////////////////////////////////////////////////////////
int CvFastSegmentation::extractRegions(int minSize)
{
    int b, i, n, a;
    int total = 0;
    CvRLE *r, *rmap;
    CvSegment *reg;
    if(mRLE.size() == 0)
        return 0;

    mSegments.create(mRLE.size());

    b = i = n = a = 0;

    rmap = mRLE.ptr();
    reg = mSegments.ptr();
    for(i = 0; i < (int)mRLE.size(); i++)
    {
        if(rmap[i].color > 0)
        {
            r = &rmap[i];
            if(r->parent == i)
            {
                //  Add new region if this run is root
                rmap[i].parent = b = n; //  Renumber the parent to point to region
                reg[b].color = r->color;
                reg[b].area = r->width;                
                reg[b].left = r->x;
                reg[b].right = r->x + r->width;
                reg[b].top = r->y;
                reg[b].bottom = r->y;
                reg[b].x = rangeSum(r->x, r->width);
                reg[b].y = r->y*r->width;
                reg[b].perimeter = r->width > 1 ? 2 : 1;
                //  Update the rle encodings
                reg[b].rle.push(*r);
                n++;
                total++;
                if(n >= (int)mSegments.size()) return (int)mSegments.size();
            }
            else 
            {
                //  Update regions stats incrementally
                b = rmap[r->parent].parent;
                rmap[i].parent = b; //  Update parent to identify region id
                reg[b].area += r->width;
                reg[b].x += rangeSum(r->x,  r->width);
                reg[b].y += r->y*r->width; 
                reg[b].left = r->x < reg[b].left ? r->x : reg[b].left;
                reg[b].top = r->y < reg[b].top ? r->y : reg[b].top;
                reg[b].right = r->x + r->width > reg[b].right ? r->x + r->width : reg[b].right;
                reg[b].bottom = r->y > reg[b].bottom ? r->y : reg[b].bottom;
                reg[b].perimeter += r->width > 1 ? 2 : 1;
                //  Update the rle encodings
                reg[b].rle.push(*r);
            }
        }
    }

    //  Final pass through the regions to calculate centroids
    //  and other characteristics
    CvSegment *seg = mSegments.ptr();
    for(i = 0; i < n; i++, seg++)
    {
        double a = seg->area;
        //  Remove segments of a certain size
        if(seg->area < minSize)
        {
            seg->valid = 0;
        }
        else
        {
			seg->fx = (float)(seg->x/a);
			seg->fy = (float)(seg->y/a);
            seg->x = (int)(seg->x/a);
            seg->y = (int)(seg->y/a);
            seg->rect.y = seg->top;
            seg->rect.x = seg->left;
            seg->rect.height = seg->bottom - seg->top;
            seg->rect.height = seg->rect.height > 0 ? seg->rect.height : 1;
            seg->rect.width = seg->right - seg->left;
            seg->rect.width = seg->rect.width > 0 ? seg->rect.width : 1;
            seg->density = 100*seg->area/(seg->rect.height*seg->rect.width);
            seg->aspectRatio = 100*(seg->rect.width)/(seg->rect.height > 0 ? seg->rect.height : 1);
            seg->valid = 1;
            seg->compactness = (int)(100*seg->perimeter*seg->perimeter/(4*CV_PI*seg->area));
        }
    }

    //  Trim off the excess regions not used
    mSegments.resize(total);

    return total;
}


//////////////////////////////////////////////////////////////////////////////
///
///   \brief Seperates colored segments into different arrays.
///
///   \param thresh Allowed overlap between components.
///
///   \return Total number of final segments.
///
//////////////////////////////////////////////////////////////////////////////
int CvFastSegmentation::seperateRegions(const int thresh)
{
    //  Only sort colors if we segmented a color image
    if(!mColorImage)
    {
        int total = 0;
        CvSegment *ptr = mSegments.ptr();
        mColorSegments.create(1);
        Array<CvSegment> *final = &mColorSegments[0];
        for(int i = 0; i < mSegments.size(); i++, ptr++)
        {
            if(ptr->valid)
            {
                final->push(*ptr);
                total++;
            }
        }

        return total;
    }

    int total = 0;
    int numColors = mColorSegments.size();
    for(int i = 0; i < numColors; i++)
        mColorSegments[i].clear();

    CvSegment *ptr = mSegments.ptr();
    for(int i = 0; i < mSegments.size(); i++, ptr++)
    {
        if(ptr->valid && ptr->color > 0 && ptr->color <= (int)numColors)
        {
            total++;
            CvSegment *colorSegPtr = mColorSegments[ptr->color - 1].ptr();
            if(thresh > 0)
            {
                for(int j = 0; j < mColorSegments[ptr->color - 1].size(); j++, colorSegPtr++)
                {
                    if(colorSegPtr != ptr && colorSegPtr->segmentDistance(*ptr) < thresh)
                    {
                        colorSegPtr->merge(*ptr);
                        break;
                    }
                }
                //  If not merged..
                if(ptr->valid)
                    mColorSegments[ptr->color - 1].push(*ptr);
            }
            else
                mColorSegments[ptr->color - 1].push(*ptr);
        }
    }

    return total;
}


/* End of file */
