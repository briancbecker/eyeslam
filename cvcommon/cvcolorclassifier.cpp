/*==================================================================================

    Filename:  cvcolorclassifier.cpp

    Copyright 2007 Daniel Barber
                   Robotics Laboratory
                   University of Central Florida
                   http://robotics.ucf.edu

    Program Contents:
    -------------------------------------------------------------------------------
    Contains class for color classification that uses large amounts of memory, 
    but results in fast performance that is capable of dealing with noisy 
    environments.
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
#include "cvcolorclassifier.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>


////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Constructor.
///
////////////////////////////////////////////////////////////////////////////////////
CvColorClassifier::CvColorClassifier()
{
    mBigCube = mSmallCube = NULL;
    mVotes = NULL;
    mCode = -1;
    mSigBig = mSigSmall = -1.0;
    mBigWidth = 0;
    mSmallWidth = 0;
    mScale = 100;
    mVotesDimensions = 256*mScale/100+1;
	mDigitized = false;
}

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Constructor.
///
////////////////////////////////////////////////////////////////////////////////////
CvColorClassifier::CvColorClassifier(const CvColorClassifier &another)
{
    mBigCube = mSmallCube = NULL;
    mVotes = NULL;
    mCode = -1;
    mSigBig = mSigSmall = -1.0;
    mBigWidth = 0;
    mSmallWidth = 0;

    *this = another;
}

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Destructor.
///
////////////////////////////////////////////////////////////////////////////////////
CvColorClassifier::~CvColorClassifier()
{
    release();
}

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Train the classifier based on the input image and binary mask.
///  
///  The mask must be a 1 channel image.  "On" pixels (255) represent the color
///  to train on, and "off" pixels (0), are not the color to learn.
///
///  The input image must be three channels standard RGB or BGR sequenced.
///  Using the code parameter will automatically transform pixels to the
///  desired color space while training.  If this value is changed during
///  training, it will be detected, and all previous training data cleared.
///
///  \param src The image to train on.  Input image is in RGB color space.
///             During training the color space is converted automatically
///             based on the code value.
///  \param mask The binary mask showing locations of the color to learn.
///  \param sigBig The radius of positive gaussians for learning (0.5 is typical)
///  \param sigSmall The radius of negative guassians for non-colored regions
///                  (0.4 is typical).
///  \param code What color space to train on (CV_RGB_COLOR, CV_HSV_COLOR, 
///              or CV_YCrCb_COLOR). 
///
///  \return 1 ok, 0 failure to train.
///
////////////////////////////////////////////////////////////////////////////////////
int CvColorClassifier::train(const IplImage *src,
                             const IplImage *mask,
                             const double sigBig,
                             const double sigSmall,
                             const int code)
{
    int mrb, mrs;
    //  Check for valid parameters
    assert(src && mask && code >= 0 && code <= 2);
    //  Verify we have a color image and a binary mask image and
    //  that the two are of the same dimensions
    assert(mask->depth == IPL_DEPTH_8U && 
           mask->nChannels == 1 &&
           src->depth == IPL_DEPTH_8U &&
           src->nChannels == 3 &&
           src->width == mask->width &&
           src->height == mask->height);

    //  Check for reallocation
    allocate(sigBig, sigSmall);
    //  Set the color type we are training with
    if(code != mCode)
        clear();        //  Erase data from different color space

    mCode = code;       //  Set the current color space type
    //  Used to add/subtract 3D guassians in cube
    mrb = (int)(mSigBig*3.0);
    mrs = (int)(mSigSmall*3.0);
    //  Train on entire image
    for(int y = 0; y < src->height; y++)
    {
        for(int x = 0; x < src->width; x++)
        {
            //  Get the coordinates in the color cube based
            //  on the color value in the image
            int alpha, beta, gamma, val;
            getLocation(src, y, x, alpha, beta, gamma); 
            val = ((uchar*)(mask->imageData + mask->widthStep*y))[x];
            //  If we get an on value ( I(x,y) > 0) than we add
            //  the positive guassian distribution at this coordinate
            //  in the color space
            if(val == 255)
            {                
                //  Add the positive 3D gaussian distribution
                for(int p = -1*mrb; p <= mrb; p++)
                {
                    for(int q = -1*mrb; q <= mrb; q++)
                    {
                        for(int r = -1*mrb; q <= mrb; q++)
                        {        
                            //  Add the positive gaussian value
                            if (((alpha + p) >= 0 && alpha + p <= mVotesDimensions ) &&
                                ((beta  + q) >= 0 && beta  + q <= mVotesDimensions ) &&
                                ((gamma + r) >= 0 && gamma + r <= mVotesDimensions ))
                            {
                                int row, col, layer;
                                int gRow, gCol, gLayer;
                                row = alpha + p; col = beta + q; layer = gamma + r;
                                gRow = p + mrb;  gCol = q + mrb; gLayer = r + mrb;
                                int size = mVotesDimensions*mVotesDimensions;
                                double newval = mVotes[layer*size + row*mVotesDimensions + col] + mBigCube[gLayer*mBigWidth*mBigWidth + gRow*mBigWidth + gCol];
                                mVotes[layer*size + row*mVotesDimensions + col] = (float)(newval);
                            }
                        }
                    }
                }
            }
            //  This is not the color we want to learn, so subtract the smaller
            //  3D guassian from this point in the color space to prevent
            //  miss-clasification
            else
            {
                for(int p = -1*mrs; p <= mrs; p++)
                {
                    for(int q = -1*mrs; q <= mrs; q++)
                    {
                        for(int r = -1*mrs; q <= mrs; q++)
                        {         
                            //  Add the negative gaussian value
                            if (((alpha + p) >= 0 && alpha + p <= mVotesDimensions ) &&
                                ((beta  + q) >= 0 && beta  + q <= mVotesDimensions  ) &&
                                ((gamma + r) >= 0 && gamma + r <= mVotesDimensions ))
                            {
                                int row, col, layer;
                                int gRow, gCol, gLayer;
                                row = alpha + p; col = beta + q; layer = gamma + r;
                                gRow = p + mrs;  gCol = q + mrs; gLayer = r + mrs;
                                int size = mVotesDimensions*mVotesDimensions;
                                double newval = mVotes[layer*size + row*mVotesDimensions + col] - mSmallCube[gLayer*mSmallWidth*mSmallWidth + gRow*mSmallWidth + gCol];
                                mVotes[layer*size + row*mVotesDimensions + col] = (float)(newval);
                            }
                        }
                    }
                }
            }
        }
    }
    
    return 1;
}


////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Classifies color regions within the picture based on previous 
///  color training.  The result is stored in a mask image representing
///  color regions.
///
///  It is not necessary to convert the input image to any color space.  It must
///  be RGB or BGR data.  Conversion to HSV or YCrCb space is done automatically
///  based on how the classifier was trained.
///
///  This function does support use of ROI in that it will only classify
///  pixels in the ROI.
///
///  \param src The image to classify colors in.
///  \param dest Binary output mask representing color locations. Cannot be NULL.
///  \param threshhold Used to filter out miss classifications.  Any values
///                    greater than this within the color space equal the
///                    color trained on.
///  \param cspace Color space format of input image (CV_RGB_COLOR, CV_HSV_COLOR,
///                CV_YCrCb_COLOR).
///
///  \return 1 on ok, 0 on failure to classify.
///
////////////////////////////////////////////////////////////////////////////////////
int CvColorClassifier::classify(const IplImage *src, 
                                IplImage *dest, 
                                const double threshhold,
                                const int cspace)
{
    bool rgb = true;
    int alpha, beta, gamma, *alphaPtr = 0, *betaPtr = 0, *gammaPtr = 0;
	//void (*getLocation)(const IplImage *src, const int y, const int y, int &alpha, int &beta, int &gamma) const;

    //  Verify we have a color image and a binary mask image and
    //  that the two are of the same dimensions based on image size
    //  or the ROI (region of interest)
    assert(src && dest &&
           dest->depth == IPL_DEPTH_8U && 
           dest->nChannels == 1 &&
           src->depth == IPL_DEPTH_8U &&
           src->nChannels == 3 &&
           (src->width == dest->width || (src->roi ? src->roi->width : src->width) == dest->width) &&
           (src->height == dest->height || (src->roi ? src->roi->height : src->height) == dest->height));

    if(!mVotes)
    {
        cvSetZero(dest);
        return 0;
    }
    
    //  Check the ROI (Region of Interest) to see if we should
    //  only scan a specific area of the image
    int sRow, sCol, eRow, eCol;
    int sRow1, sCol1;
    if(src->roi != NULL)
    {
        sRow = src->roi->yOffset; sCol = src->roi->xOffset;
        eRow = src->roi->yOffset + src->roi->height;
        eCol = src->roi->xOffset + src->roi->width;

        sRow1 = dest->width == src->roi->width ? 0 : sRow;
        sCol1 = dest->height == src->roi->height ? 0 : sCol;
    }
    else
    {
        sRow = 0; sCol = 0;
        eRow = src->height; eCol = src->width;
        sRow1 = 0; sCol1 = 0;
    }

	if (src->channelSeq[0] == 'R' && src->channelSeq[1] == 'G' && src->channelSeq[2] == 'B')
	{
		alphaPtr = &alpha; betaPtr = &beta; gammaPtr = &gamma;
	}
	else
	{
		// Reverse BGR pixel format by reversing alpha/beta/gamma
		alphaPtr = &gamma; betaPtr = &beta; gammaPtr = &alpha;
	}

	if (cspace != mCode)
	{
		if (mDigitized)
		{
			for(int y = sRow, m = sRow1; y < src->height && y < eRow; y++, m++)
			{
				uchar *row = (uchar*)(dest->imageData + dest->widthStep*m + sCol1);
				for(int x = sCol, n = sCol1; x < src->width && x < eCol; x++, n++, row++)
				{
					getLocation(src, y, x, alpha, beta, gamma, cspace);
					if(alpha < mVotesDimensions && beta < mVotesDimensions && gamma < mVotesDimensions)
						*row = (uchar)mVotes[mVotesDimensions*mVotesDimensions*gamma + alpha*mVotesDimensions + beta];
				}
			}
		}
		else
		{
			for(int y = sRow, m = sRow1; y < src->height && y < eRow; y++, m++)
			{
				uchar *row = (uchar*)(dest->imageData + dest->widthStep*m + sCol1);
				for(int x = sCol, n = sCol1; x < src->width && x < eCol; x++, n++, row++)
				{
					getLocation(src, y, x, alpha, beta, gamma, cspace);
					assert(alpha < mVotesDimensions && beta < mVotesDimensions && gamma < mVotesDimensions);
					if(alpha < mVotesDimensions && beta < mVotesDimensions && gamma < mVotesDimensions &&
						mVotes[mVotesDimensions*mVotesDimensions*gamma + alpha*mVotesDimensions + beta] > (float)threshhold)
						*row = 255;
					else
						*row = 0;

				}
			}
		}
	}
	else
	{
		if (mDigitized)
		{
			for(int y = sRow, m = sRow1; y < src->height && y < eRow; y++, m++)
			{
				uchar *row = (uchar*)(dest->imageData + dest->widthStep*m + sCol1);
				uchar *img = (uchar*)(src->imageData + src->widthStep*y + sCol*3);

				for(int x = sCol, n = sCol1; x < src->width && x < eCol; x++, n++, row++)
				{
					*alphaPtr = *img++*mScale/100;
					*betaPtr = *img++*mScale/100;
					*gammaPtr = *img++*mScale/100;
					assert(alpha < mVotesDimensions && beta < mVotesDimensions && gamma < mVotesDimensions);
					*row = (uchar)mVotes[mVotesDimensions*mVotesDimensions*gamma + alpha*mVotesDimensions + beta];
				}
			}
		}
		else
		{
			for(int y = sRow, m = sRow1; y < src->height && y < eRow; y++, m++)
			{
				uchar *row = (uchar*)(dest->imageData + dest->widthStep*m + sCol1);
				uchar *img = (uchar*)(src->imageData + src->widthStep*y + sCol*3);

				for(int x = sCol, n = sCol1; x < src->width && x < eCol; x++, n++, row++)
				{
					*alphaPtr = *img++*mScale/100;
					*betaPtr = *img++*mScale/100;
					*gammaPtr = *img++*mScale/100;
					assert(alpha < mVotesDimensions && beta < mVotesDimensions && gamma < mVotesDimensions);
					if(mVotes[mVotesDimensions*mVotesDimensions*gamma + alpha*mVotesDimensions + beta] > (float)threshhold)
						*row = 255;
					else
						*row = 0;
				}
			}
		}
	}

    return 1;
}

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Classifies a pixel as being of the trained color data or not.
///
///  \param r Red color channel value.
///  \param g Green color channel value.
///  \param b Blue color channel value.
///  \param threshhold Used to filter out miss classifications.  Any values
///                    greater than this within the color space equal the
///                    color trained on.
///  \param cspace Color space format of input image (CV_RGB_COLOR, CV_HSV_COLOR,
///                CV_YCrCb_COLOR).
///
///  \return 1 if pixel equals color, otherwise false.
///
////////////////////////////////////////////////////////////////////////////////////
int CvColorClassifier::classify(const unsigned char r,
                                const unsigned char g,
                                const unsigned char b, 
                                const double threshhold,
                                const int cspace) const
{
    if(!mVotes)
        return 0;

	int alpha, beta, gamma;
	switch(mCode)
	{
	case CV_RGB_COLOR:
		alpha = r;
		beta = g;
		gamma = b;
		break;
	case CV_HSV_COLOR:
		RGB2HSV(r, g, b, alpha, beta, gamma);
		break;
	case CV_YCrCb_COLOR:
		RGB2YCrCb(r, g, b, alpha, beta, gamma);
		break;
	default:
		return 0;
		break;
	}
	//  Convert to fit scaled color space.
	alpha = alpha*mScale/100;
	beta = beta*mScale/100;
	gamma = gamma*mScale/100;

	if( alpha >= 0 && alpha < mVotesDimensions  &&
		beta >= 0  && beta < mVotesDimensions   &&
		gamma >= 0 && gamma < mVotesDimensions )
	{
		if (mDigitized)
			return (int)mVotes[mVotesDimensions*mVotesDimensions*gamma + alpha*mVotesDimensions + beta];
		else
			return (mVotes[mVotesDimensions*mVotesDimensions*gamma + alpha*mVotesDimensions + beta] > (float)threshhold) ? 1 : 0;
	}

	return 0;
}

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Classifies a pixel as being of the trained color data or not.
///
///  \param color RGB pixel value to classify.
///  \param threshhold Used to filter out miss classifications.  Any values
///                    greater than this within the color space equal the
///                    color trained on.
///  \param cspace Color space format of input image (CV_RGB_COLOR, CV_HSV_COLOR,
///                CV_YCrCb_COLOR).
///
///  \return 1 if pixel equals color, otherwise false.
///
////////////////////////////////////////////////////////////////////////////////////
int CvColorClassifier::classify(CvScalar color,
                                const double threshhold,
                                const int cspace) const
{
    if(!mVotes)
        return 0;

    int r, g, b;
    b = (int)color.val[0];
    g = (int)color.val[1];
    r = (int)color.val[2];
	
	return classify(r, g, b, threshhold, cspace);
}

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Classifies a pixel within an input image as being the color trained
///         for or not.
///
///  Source image must contain RGB data.
///
///  \param src Location of image data.
///  \param y The row in the src image to get pixel data from.
///  \param x The column in the src image to get pixel data from.
///  \param threshhold Used to filter out miss classifications.  Any values
///                    greater than this within the color space equal the
///                    color trained on.
///  \param cspace Color space format of input image (CV_RGB_COLOR, CV_HSV_COLOR,
///                CV_YCrCb_COLOR).
///
///  \return 1 if pixel equals color, otherwise false.
///
////////////////////////////////////////////////////////////////////////////////////
int CvColorClassifier::classify(const IplImage *src,
                                const int y,
                                const int x,
                                const double threshhold,
                                const int cspace) const
{
    if(!mVotes)
        return 0;

    //  Verify we have a color image and a binary mask image and
    //  that the two are of the same dimensions
    assert(src &&
           src->depth == IPL_DEPTH_8U &&
           src->nChannels == 3);
    if( y < 0 || y >= src->height ||
        x < 0 || x >= src->width)
        return 0;

    int alpha, beta, gamma;
    getLocation(src, y, x, alpha, beta, gamma, cspace);

    if( alpha >= 0 && alpha < mVotesDimensions  &&
        beta >= 0  && beta < mVotesDimensions   &&
        gamma >= 0 && gamma < mVotesDimensions )
    {
		if (mDigitized)
	        return (int)mVotes[mVotesDimensions*mVotesDimensions*gamma + alpha*mVotesDimensions + beta];
		else
			return (mVotes[mVotesDimensions*mVotesDimensions*gamma + alpha*mVotesDimensions + beta] > (float)threshhold) ? 1 : 0;
    }

    return 0;
}

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Classifies color regions within the picture based on previous 
///  color training.  The result is stored in a mask image representing
///  color regions.
///
///  It is not necessary to convert the input image to any color space.  It must
///  be RGB or BGR data.  Conversion to HSV or YCrCb space is done automatically
///  based on how the classifier was trained.
///
///  This function does support use of ROI in that it will only classify
///  pixels in the ROI.  The destination must be either the same size as the
///  input image, or the same size as the inputs ROI.
///
///  \param src The image to classify colors in.
///  \param dest Binary output mask representing color locations. Cannot be NULL.
///  \param mask Used to mask areas of the image you do not want data classified
///              in.  Areas in the input image with a 255, "on", value in the
///              mask image are ignored.  If NULL than it is ignored.  The mask
///              must be a single channel binary image.
///  \param threshhold Used to filter out miss classifications.  Any values
///                    greater than this within the color space equal the
///                    color trained on.
///  \param cspace Color space format of input image (CV_RGB_COLOR, CV_HSV_COLOR,
///                CV_YCrCb_COLOR).
///
///  \return 1 on ok, 0 on failure to classify.
///
////////////////////////////////////////////////////////////////////////////////////
int CvColorClassifier::classify(const IplImage *src, 
                                IplImage *dest, 
                                const IplImage *mask, 
                                const double threshhold,
                                const int cspace)
{
    bool rgb = true;
    //  Verify we have a color image and a binary mask image and
    //  that the two are of the same dimensions
    assert(src && dest &&
           dest->depth == IPL_DEPTH_8U && 
           dest->nChannels == 1 &&
           src->depth == IPL_DEPTH_8U &&
           src->nChannels == 3 &&
           (src->width == dest->width || (src->roi ? src->roi->width : src->width) == dest->width) &&
           (src->height == dest->height || (src->roi ? src->roi->height : src->height) == dest->height));

    if(mask)
    {
        assert(mask->depth == IPL_DEPTH_8U && 
               mask->nChannels == 1 &&
               (src->width == mask->width || (src->roi ? src->roi->width : src->width) == mask->width) &&
               (src->height == mask->height || (src->roi ? src->roi->height : src->height) == mask->height));
    }

    if(!mVotes)
    {
        cvSetZero(dest);
        return 0;
    }
    
    //  Check the ROI (Region of Interest) to see if we should
    //  only scan a specific area of the image
    int sRow, sCol, eRow, eCol;
    int sRow1, sCol1, sRow2, sCol2;
    if(src->roi != NULL)
    {
        sRow = src->roi->yOffset; sCol = src->roi->xOffset;
        eRow = src->roi->yOffset + src->roi->height;
        eCol = src->roi->xOffset + src->roi->width;

        sRow1 = dest->width == src->roi->width ? 0 : sRow;
        sCol1 = dest->height == src->roi->height ? 0 : sCol;
        sRow2 = (mask && mask->width == src->roi->width) ? 0 : sRow;
        sCol2 = (mask && mask->height == src->roi->height) ? 0 : sCol;
    }
    else
    {
        sRow = 0; sCol = 0;
        eRow = src->height; eCol = src->width;
        sRow1 = 0; sCol1 = 0;
        sCol2 = 0, sRow2 = 0;
    }

	if (mDigitized)
	{
		for(int y = sRow, m = sRow1, i = sRow2; y < src->height && y < eRow; y++, m++, i++)
		{
			for(int x = sCol, n = sCol1, j = sCol2; x < src->width && x < eCol; x++, n++, j++)
			{
				int alpha, beta, gamma;
				getLocation(src, y, x, alpha, beta, gamma, cspace);
				if((!mask || (mask && ((uchar *)(mask->imageData + mask->widthStep*i))[j] == 0)) &&
					alpha < mVotesDimensions &&
					beta < mVotesDimensions &&
					gamma < mVotesDimensions &&
					mVotes[mVotesDimensions*mVotesDimensions*gamma + alpha*mVotesDimensions + beta] > (float)threshhold)
					((uchar*)(dest->imageData + dest->widthStep*m))[n] = 255;
				else
					((uchar*)(dest->imageData + dest->widthStep*m))[n] = 0;

			}
		}
	}
	else
	{
		for(int y = sRow, m = sRow1, i = sRow2; y < src->height && y < eRow; y++, m++, i++)
		{
			for(int x = sCol, n = sCol1, j = sCol2; x < src->width && x < eCol; x++, n++, j++)
			{
				int alpha, beta, gamma;
				getLocation(src, y, x, alpha, beta, gamma, cspace);
				if((!mask || (mask && ((uchar *)(mask->imageData + mask->widthStep*i))[j] == 0)) &&
					alpha < mVotesDimensions &&	beta < mVotesDimensions && gamma < mVotesDimensions)
					((uchar*)(dest->imageData + dest->widthStep*m))[n] = (uchar)mVotes[mVotesDimensions*mVotesDimensions*gamma + alpha*mVotesDimensions + beta];
			}
		}
	}

    return 1;
}


////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Classifies color regions within the picture based on previous 
///  color training.  The result is drawn onto an output image.
///
///  It is not necessary to convert the input image to any color space.  It must
///  be RGB or BGR data.  Conversion to HSV or YCrCb space is done automatically
///  based on how the classifier was trained.
///
///  This function does support use of ROI in that it will only classify
///  pixels in the ROI.  The destination must be either the same size as the
///  input image, or the same size as the inputs ROI if used.
///
///  This function is useful when you want to color over regions within in
///  and image or create a colored representation of the color found.
///
///  \param src The image to classify colors in.
///  \param dest Where to draw the classified colors in.  If NULL, than the
///  input image is drawn on.  If a grayscale image, than a binary image is
///  created.
///  \param threshhold Used to filter out miss classifications.  Any values
///                    greater than this within the color space equal the
///                    color trained on.
///  \param color What color to draw with (CV_RGB(255, 0, 0))
///  \param cspace Color space format of input image (CV_RGB_COLOR, CV_HSV_COLOR,
///                CV_YCrCb_COLOR).
///
///  \return 1 on ok, 0 on failure to classify.
///
////////////////////////////////////////////////////////////////////////////////////
int CvColorClassifier::classify(IplImage *src, 
                                IplImage *dest, 
                                const double threshhold,
                                CvScalar color,
                                const int cspace)
{
    bool rgb = true;
    //  Verify we have a color image and a binary mask image and
    //  that the two are of the same dimensions
    if(dest)
        assert(src && dest &&
               dest->depth == IPL_DEPTH_8U && 
               src->depth == IPL_DEPTH_8U &&
               src->nChannels == 3 &&
               (src->width == dest->width || (src->roi ? src->roi->width : src->width) == dest->width) &&
               (src->height == dest->height || (src->roi ? src->roi->height : src->height) == dest->height));
    else
        assert(src && src->depth == IPL_DEPTH_8U && src->nChannels == 3);

    if(!mVotes)
    {
        if(dest)
            cvSetZero(dest);
        return 0;
    }

    //  Check the ROI (Region of Interest) to see if we should
    //  only scan a specific area of the image
    int sRow, sCol, eRow, eCol;
    int sRow1, sCol1;
    if(src->roi != NULL)
    {
        sRow = src->roi->yOffset; sCol = src->roi->xOffset;
        eRow = src->roi->yOffset + src->roi->height;
        eCol = src->roi->xOffset + src->roi->width;

        sRow1 = (dest && dest->width == src->roi->width) ? 0 : sRow;
        sCol1 = (dest && dest->height == src->roi->height) ? 0 : sCol;
    }
    else
    {
        sRow = 0; sCol = 0;
        eRow = src->height; eCol = src->width;
        sRow1 = 0; sCol1 = 0;
    }

    for(int y = sRow, m = sRow1; y < src->height && y < eRow; y++, m++)
    {
        for(int x = sCol, n = sCol1; x < src->width && x < eCol; x++, n++)
        {
            int alpha, beta, gamma;
            getLocation(src, y, x, alpha, beta, gamma, cspace);
            
            if( alpha < mVotesDimensions &&
                beta < mVotesDimensions &&
                gamma < mVotesDimensions &&
				((mDigitized && (int)mVotes[mVotesDimensions*mVotesDimensions*gamma + alpha*mVotesDimensions + beta]) ||
				mVotes[mVotesDimensions*mVotesDimensions*gamma + alpha*mVotesDimensions + beta] > (float)threshhold))
            {
                uchar *base;
                //  Get a pointer to where we are drawing the result.
                if(dest)
                    base = (uchar *)(dest->imageData + dest->widthStep*m);
                else
                    base = (uchar *)(src->imageData + src->widthStep*m);

                if(dest && dest->nChannels == 1)
                    base[n] = (uchar)mVotes[mVotesDimensions*mVotesDimensions*gamma + alpha*mVotesDimensions + beta];
                else
                {
                    int off = n*3;                    
                    if(src->channelSeq[0] == 'R' && 
                       src->channelSeq[1] == 'G' &&
                       src->channelSeq[2] == 'B')
                    {
                        base[off] = (uchar)color.val[2];
                        base[off + 1] = (uchar)color.val[1];
                        base[off + 2] = (uchar)color.val[0];
                    }
                    else
                    {
                        base[off]     = (uchar)color.val[0];
                        base[off + 1] = (uchar)color.val[1];
                        base[off + 2] = (uchar)color.val[2];
                    }

                }
            }
            else if(dest) //  Ony draw in destination if it is not NULL.
            {               
                uchar *destBase = (uchar *)(dest->imageData + dest->widthStep*m);
                uchar *srcBase = (uchar *)(src->imageData + src->widthStep*y);
                if(dest->nChannels == 1)
                    destBase[n] = 0;
                else if(dest->nChannels == 3)
                {
                    int desOff = n*3;
                    int srcOff = x*3;
                    /*
                    destBase[desOff]     = srcBase[srcOff];
                    destBase[desOff + 1] = srcBase[srcOff + 1];
                    destBase[desOff + 2] = srcBase[srcOff + 2];
                    */
                }
            }
        
        }
    }

    return 1;
}


////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Classifies color regions within the picture based on previous 
///  color training.  The result is drawn onto an output image.
///
///  It is not necessary to convert the input image to any color space.  It must
///  be RGB or BGR data.  Conversion to HSV or YCrCb space is done automatically
///  based on how the classifier was trained.
///
///  This function is useful when you want to color over regions within in
///  and image or create a colored representation of the color found.
///
///  This function does support use of ROI in that it will only classify
///  pixels in the ROI.  The destination must be either the same size as the
///  input image, or the same size as the inputs ROI.  Same for the mask.
///
///  \param src The image to classify colors in.
///  \param dest Where to draw the classified colors in.  If NULL, than the
///  input image is drawn on.  If a grayscale image, than a binary image is
///  created.
///  \param mask Used to mask areas of the image you do not want data classified
///              in.  Areas in the input image with a 255, "on", value in the
///              mask image are ignored.  If NULL than it is ignored.  The mask
///              must be a single channel binary image.
///  \param threshhold Used to filter out miss classifications.  Any values
///                    greater than this within the color space equal the
///                    color trained on.
///  \param color What color to draw with (CV_RGB(255, 0, 0))
///  \param cspace Color space format of input image (CV_RGB_COLOR, CV_HSV_COLOR,
///                CV_YCrCb_COLOR).
///
///  \return 1 on ok, 0 on failure to classify.
///
////////////////////////////////////////////////////////////////////////////////////
int CvColorClassifier::classify(IplImage *src, 
                                IplImage *dest, 
                                const IplImage *mask,
                                const double threshhold,
                                CvScalar color,
                                const int cspace)
{
    bool rgb = true;
    //  Verify we have a color image and a binary mask image and
    //  that the two are of the same dimensions
    if(dest)
        assert(src && 
               dest->depth == IPL_DEPTH_8U && 
               src->depth == IPL_DEPTH_8U &&
               src->nChannels == 3 &&
               (src->width == dest->width || (src->roi ? src->roi->width : src->width) == dest->width) &&
               (src->height == dest->height || (src->roi ? src->roi->height : src->height) == dest->height));
    else
        assert(src && src->depth == IPL_DEPTH_8U && src->nChannels == 3);

    if(mask)
    {
        assert(mask->depth == IPL_DEPTH_8U && 
               mask->nChannels == 1 &&
               (src->width == mask->width || (src->roi ? src->roi->width : src->width) == mask->width) &&
               (src->height == mask->height || (src->roi ? src->roi->height : src->height) == mask->height));
    }

    if(!mVotes)
    {
        if(dest)
            cvSetZero(dest);
        return 0;
    }
    
    //  Check the ROI (Region of Interest) to see if we should
    //  only scan a specific area of the image
    int sRow, sCol, eRow, eCol;
    int sRow1, sCol1, sRow2, sCol2;
    if(src->roi != NULL)
    {
        sRow = src->roi->yOffset; sCol = src->roi->xOffset;
        eRow = src->roi->yOffset + src->roi->height;
        eCol = src->roi->xOffset + src->roi->width;

        sRow1 = (dest && dest->width == src->roi->width) ? 0 : sRow;
        sCol1 = (dest && dest->height == src->roi->height) ? 0 : sCol;
        sRow2 = (mask && mask->width == src->roi->width) ? 0 : sRow;
        sCol2 = (mask && mask->height == src->roi->height) ? 0 : sCol;
    }
    else
    {
        sRow = 0; sCol = 0;
        eRow = src->height; eCol = src->width;
        sRow1 = 0; sCol1 = 0;
        sCol2 = 0, sRow2 = 0;
    }

    for(int y = sRow, m = sRow1, i = sRow2; y < src->height && y < eRow; y++, m++, i++)
    {
        for(int x = sCol, n = sCol1, j = sCol2; x < src->width && x < eCol; x++, n++, j++)
        {
            int alpha, beta, gamma;
            getLocation(src, y, x, alpha, beta, gamma, cspace);
            if((!mask || (mask && ((uchar *)(mask->imageData + mask->widthStep*i))[j] == 0)) &&
                alpha < mVotesDimensions &&
                beta  < mVotesDimensions &&
                gamma < mVotesDimensions &&
                ((mDigitized && (int)mVotes[mVotesDimensions*mVotesDimensions*gamma + alpha*mVotesDimensions + beta]) ||
					mVotes[mVotesDimensions*mVotesDimensions*gamma + alpha*mVotesDimensions + beta] > (float)threshhold))
            {
                uchar *base;
                //  Get a pointer to where we are drawing the result.
                if(dest)
                    base = (uchar *)(dest->imageData + dest->widthStep*m);
                else
                    base = (uchar *)(src->imageData + src->widthStep*m);

                if(dest && dest->nChannels == 1)
                    base[n] = (uchar)mVotes[mVotesDimensions*mVotesDimensions*gamma + alpha*mVotesDimensions + beta];
                else
                {
                    int off = n*3;                    
                    if(src->channelSeq[0] == 'R' && 
                       src->channelSeq[1] == 'G' &&
                       src->channelSeq[2] == 'B')
                    {
                        base[off] = (uchar)color.val[2];
                        base[off + 1] = (uchar)color.val[1];
                        base[off + 2] = (uchar)color.val[0];
                    }
                    else
                    {
                        base[off]     = (uchar)color.val[0];
                        base[off + 1] = (uchar)color.val[1];
                        base[off + 2] = (uchar)color.val[2];
                    }

                }
            }
            else if(dest) //  Ony draw in destination if it is not NULL.
            {               
                uchar *destBase = (uchar *)(dest->imageData + dest->widthStep*m);
                uchar *srcBase = (uchar *)(src->imageData + src->widthStep*m);
                if(dest->nChannels == 1)
                    destBase[n] = 0;
                else if(dest->nChannels == 3)
                {
                    int desOff = n*3;
                    int srcOff = x*3;
                    /*
                    destBase[desOff]     = srcBase[srcOff];
                    destBase[desOff + 1] = srcBase[srcOff + 1];
                    destBase[desOff + 2] = srcBase[srcOff + 2];
                    */
                }
            }
        
        }
    }

    return 1;
}


////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Gets a location within the color cube based on the location and
///  values within an input image.
///
///  This function handles any conversion to HSV or YCrCb and checks channel
///  sequence to get coordinates [0,255] within the color cube.
///
///  \param src Input image.
///  \param y Row in input image.
///  \param x Column in input image.
///  \param alpha Row in color space.
///  \param beta Column in color space.
///  \param gamma Layer in color space.
///  \param cspace Color space format of input image (CV_RGB_COLOR, CV_HSV_COLOR,
///                CV_YCrCb_COLOR).
///
////////////////////////////////////////////////////////////////////////////////////
inline void CvColorClassifier::getLocation(const IplImage *src, 
                                           const int y, 
                                           const int x, 
                                           int &alpha, 
                                           int &beta, 
                                           int &gamma,
                                           const int cvspace) const
{
    int rr, gg, bb, off;
    off = x*3;
    uchar *base = (uchar *)(src->imageData + src->widthStep*y);
    //  Check channel sequence and get the RGB values
    if(src->channelSeq[0] == 'R' && 
       src->channelSeq[1] == 'G' &&
       src->channelSeq[2] == 'B')
    {
        rr = base[off];
        gg = base[off+1];
        bb = base[off+2];
    }
    else
    {
        bb = base[off];
        gg = base[off+1];
        rr = base[off+2];
    }   
    //  If the image data is already
    //  in the same color space as the trained
    //  classifier, than we don't need to
    //  do any conversion.
    if(cvspace == mCode)
    {
        alpha = rr*mScale/100;
        beta = gg*mScale/100;
        gamma = bb*mScale/100;
        return;
    }
    else
    {
        //  Check for color conversion if needed
        if(mCode == CV_HSV_COLOR)
            RGB2HSV(rr, gg, bb, alpha, beta, gamma);
        else if(mCode == CV_YCrCb_COLOR)
            RGB2YCrCb(rr, gg, bb, alpha, beta, gamma);
        else
        {
            alpha = rr; beta = gg; gamma = bb;
        }
    }

    alpha = alpha*mScale/100;
    beta = beta*mScale/100;
    gamma = gamma*mScale/100;
}

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Releases allocated memory.
///
////////////////////////////////////////////////////////////////////////////////////
void CvColorClassifier::release()
{
    if(mBigCube)
        delete[] mBigCube;
    if(mSmallCube)
        delete[] mSmallCube;
    if(mVotes)
        delete[] mVotes;

    mBigCube = mSmallCube = NULL;
    mVotes = NULL;
    mSigBig = mSigSmall = -1.0;
    mCode = -1;
    mBigWidth = 0;
    mSmallWidth = 0;
}

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Allocates memory based on the sigma values.
///
///  \param sigbig Larger sigma value.
///  \param sigsmall Smaller sigma value.
///
////////////////////////////////////////////////////////////////////////////////////
void CvColorClassifier::allocate(const double sigbig, const double sigsmall)
{
    int mrb, mrs, bwidth, swidth;

    //  See if we even need to allocate
    if(mVotes && 
       mBigCube && 
       mSmallCube &&
       fabs(mSigBig - sigbig) < .000001 &&
       fabs(mSigSmall - sigsmall) < .000001)
       return;

    if(sigbig > 0)
        mSigBig = sigbig;
    else
        mSigBig = 2.0;

    if(sigsmall > 0)
        mSigSmall = sigsmall;
    else
        mSigSmall = 1.0;

    mrb = (int)(mSigBig*3);
    mrs = (int)(mSigSmall*3);
    
    if(!mVotes)
    {
        mVotes = new float[mVotesDimensions*mVotesDimensions*mVotesDimensions];
        assert(mVotes);
        memset(mVotes, 0, sizeof(float)*(mVotesDimensions*mVotesDimensions*mVotesDimensions));
    }

    if(mBigCube)
        delete[] mBigCube;
    if(mSmallCube)
        delete[] mSmallCube;

    mBigCube = mSmallCube = NULL;

    bwidth = 2*mrb + 1;
    swidth = 2*mrs + 1;
    mBigCube = new double[bwidth*bwidth*bwidth];
    mSmallCube = new double[swidth*swidth*swidth];    
    assert(mVotes && mBigCube && mSmallCube);
    mBigWidth = bwidth;
    mSmallWidth = swidth;
    //  Clear out the cube
    memset(mBigCube, 0, sizeof(double)*mBigWidth*mBigWidth*mBigWidth);
    memset(mSmallCube, 0, sizeof(double)*mSmallWidth*mSmallWidth*mSmallWidth);
   
    for(int i = 0; i < mBigWidth; i++)          //  Row
    {
        for(int j = 0; j < mBigWidth; j++)      //  Column
        {
            for(int k = 0; k < mBigWidth; k++)  //  Layer
            {
                double gauss;
                //  Calculate the gaussian value in three dimensions
                gauss = exp((-1.0 * ((i*i) + (j*j) + (k*k))/(2.0*mSigBig*mSigBig)) / (2 * CV_PI * mSigBig * mSigBig));
                mBigCube[k*mBigWidth*mBigWidth + i*mBigWidth + j] = gauss;
            }
        }
    }
    
    for(int i = 0; i < mSmallWidth; i++)          //  Row
    {
        for(int j = 0; j < mSmallWidth; j++)      //  Column
        {
            for(int k = 0; k < mSmallWidth; k++)  //  Layer
            {
                double gauss;
                //  Calculate the gaussian value in three dimensions
                gauss = exp((-1.0 * ((i*i) + (j*j) + (k*k))/(2.0*mSigSmall*mSigSmall)) / (2 * CV_PI * mSigSmall * mSigSmall));
                mSmallCube[k*mSmallWidth*mSmallWidth + i*mSmallWidth + j] = gauss;
            }
        }
    }
    
}

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Clears out classifer data.
///
////////////////////////////////////////////////////////////////////////////////////
void CvColorClassifier::clear()
{
    if(mVotes)
    {
        memset(mVotes, 0, sizeof(float)*(mVotesDimensions*mVotesDimensions*mVotesDimensions));
    }
}

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Sets a value for scaling the color space cube.  This reduces the
///         amount of memory required, but can also reduce accuracy.
///
///  \param scale New color space scale factor [1,100%], where 100% is full
///               size gaussian color cube.
///
////////////////////////////////////////////////////////////////////////////////////
void CvColorClassifier::setColorSpaceScale(const int scale)
{
	double b, s;
    if(scale > 0 && scale <= 100 && scale != mScale)
    {
        mScale = scale;
        mVotesDimensions = 256*mScale/100+1;
        b = mSigBig; s = mSigSmall;
        release();
        allocate(b, s);
    }
}


////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Saves trained classifier data to file.
///
///  \param filename Where to save file.
///
///  \return 1 on pass, 0 on fail.
///
////////////////////////////////////////////////////////////////////////////////////
int CvColorClassifier::save(const char *filename)
{
    if(!mVotes)
        return 0;

    gzFile fp;
    fp = gzopen(filename, "wb");
    if(fp == NULL)
    {
        printf("Could not open file!\n");
        return 0;
    }
    gzprintf(fp, "Robotics Laboratory at UCF Color Classifier Training File\n");
    gzprintf(fp, "Scale: %d, Color Space: %d [0 = RGB, 1 - HSV, 2 = YCrCb]\n", mScale, mCode);
    gzprintf(fp, "SigBig: %lf, SigSmall: %lf\n", mSigBig, mSigSmall);
    unsigned int wrote = 0;
    unsigned char *ptr = (unsigned char *)(mVotes);
    wrote = gzwrite(fp, ptr, sizeof(float)*mVotesDimensions*mVotesDimensions*mVotesDimensions);
    gzclose(fp);
    if(wrote == mVotesDimensions*mVotesDimensions*mVotesDimensions*sizeof(float))
        return 1;
    /*  OLD METHOD WITHOUT ZLIB'S AWESOME COMPRESSION!
    FILE *fp = fopen(filename, "wb");
    if(!fp)
        return 0;

    fprintf(fp, "Robotics Laboratory at UCF Color Classifier Training File\n");
    fprintf(fp, "Scale: %d, Color Space: %d [0 = RGB, 1 - HSV, 2 = YCrCb]\n", mScale, mCode);
    int wrote = 0;
    for(int i = 0; i < mVotesDimensions*mVotesDimensions*mVotesDimensions; i++)
        wrote += (int)fwrite(&mVotes[i], sizeof(float), 1, fp);
    fclose(fp);
    if(wrote == mVotesDimensions*mVotesDimensions*mVotesDimensions)
        return 1;
    */

    return 0;
}


////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Loads trained classifier data from file.
///
///  \param filename Where to load data from.
///
///  \return 1 on pass, 0 on fail.
///
////////////////////////////////////////////////////////////////////////////////////
int CvColorClassifier::load(const char *filename)
{
    release();
    char buff[500];
    gzFile fp = gzopen(filename, "rb");
    if(!fp)
    {
        printf("Failed to open file\n");
        return 0;
    }
    unsigned int len = 0;
    while((buff[len] = gzgetc(fp)) != '\n' && len < 498)
        len++;
    buff[len + 1] = '\0';
    if(strcmp(buff, "Robotics Laboratory at UCF Color Classifier Training File\n") != 0)
    {
        printf("Not a Color Classifier Training File\n");
        gzclose(fp);
        return 0;
    }
    len = 0;
    while((buff[len] = gzgetc(fp)) != '\n' && len < 498)
        len++;
    buff[len + 1] = '\0';
    len = 0;
    if(sscanf(buff, "Scale: %d, Color Space: %d [0 = RGB, 1 - HSV, 2 = YCrCb]\n", &mScale, &mCode) == EOF)
    {
        mScale = 100;
        mCode = -1;
        gzclose(fp);
        return 0;
    }
    mVotesDimensions = 256*mScale/100+1;
    while((buff[len] = gzgetc(fp)) != '\n' && len < 498)
        len++;
    buff[len + 1] = '\0';
    len = 0;
    if(sscanf(buff, "SigBig: %lf, SigSmall: %lf\n", &mSigBig, &mSigSmall) == EOF)
    {
        mSigBig = 2.0;
        mSigSmall = .25;
        gzclose(fp);
        return 0;
    }
    allocate(mSigBig, mSigSmall);
    unsigned int read = 0;
    unsigned char *ptr = (unsigned char *)(mVotes);
    read = gzread(fp, ptr, sizeof(float)*mVotesDimensions*mVotesDimensions*mVotesDimensions);
    gzclose(fp);
    if(read != mVotesDimensions*mVotesDimensions*mVotesDimensions*sizeof(float))
        return 0;
    /*  OLD METHOD BEFORE USE OF ZLIB AWESOME COMPRESSION
    FILE *fp = fopen(filename, "rb");
    if(!fp)
        return 0;

    if(fscanf(fp, "Robotics Laboratory at UCF Color Classifier Training File\n") == EOF)
        return 0;
    if(fscanf(fp, "Scale: %d, Color Space: %d [0 = RGB, 1 - HSV, 2 = YCrCb]\n", &mScale, &mCode) == EOF)
    {
        mScale = 100;
        mCode = -1;
        return 0;
    }
    mVotesDimensions = 256*mScale/100+1;

    allocate(2.0, .25);
    int i = 0, read = 0;
    for(int i = 0; i < mVotesDimensions*mVotesDimensions*mVotesDimensions && !feof(fp); i++)
        read += (int)fread(&mVotes[i], sizeof(float), 1, fp);
    if(read != mVotesDimensions*mVotesDimensions*mVotesDimensions)
    {
        release();
        return 0;
    }

    fclose(fp);
    */
    return 1;
}

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Sets equal to. (Copies data, not pointers).
///
////////////////////////////////////////////////////////////////////////////////////
CvColorClassifier &CvColorClassifier::operator =(const CvColorClassifier &another)
{
    if(this != &another)
    {
        if(mVotes && !another.mVotes)
        {
            release();
        }
        else if(another.mVotes)
        {
            if(!mVotes || mVotesDimensions != another.mVotesDimensions)
            {
                release();
                mScale = another.mScale;
                mVotesDimensions = another.mVotesDimensions;
                allocate(another.mSigBig, another.mSigSmall);
            }
            
            memcpy(mVotes, another.mVotes, sizeof(float)*mVotesDimensions*mVotesDimensions*mVotesDimensions);
        }

        mSigBig = another.mSigBig;
        mSigSmall = another.mSigSmall;
        mBigWidth = another.mBigWidth;
        mSmallWidth = another.mSmallWidth;
        mCode = another.mCode;
		mDigitized = mDigitized;
    }

    return *this;
}

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Converts RGB to HSV.  Code is form cvcolor.cpp
///
///  \param r Red channel value.
///  \param g Green channel value.
///  \param b Blue channel value.
///  \param _h Hue.
///  \param _s Saturation.
///  \param _v Value/Intensity.
///
////////////////////////////////////////////////////////////////////////////////////
inline void RGB2HSV(const int r,
                    const int g,
                    const int b,
                    int &_h,
                    int &_s,
                    int &_v)
{
    int h, s, v = b;
    int vmin = b, diff;
    int vr, vg;
   
    //  FROM cvcolor.cpp
    const int hsv_shift = 12;

    static const int div_table[] = {
        0, 1044480, 522240, 348160, 261120, 208896, 174080, 149211,
        130560, 116053, 104448, 94953, 87040, 80345, 74606, 69632,
        65280, 61440, 58027, 54973, 52224, 49737, 47476, 45412,
        43520, 41779, 40172, 38684, 37303, 36017, 34816, 33693,
        32640, 31651, 30720, 29842, 29013, 28229, 27486, 26782,
        26112, 25475, 24869, 24290, 23738, 23211, 22706, 22223,
        21760, 21316, 20890, 20480, 20086, 19707, 19342, 18991,
        18651, 18324, 18008, 17703, 17408, 17123, 16846, 16579,
        16320, 16069, 15825, 15589, 15360, 15137, 14921, 14711,
        14507, 14308, 14115, 13926, 13743, 13565, 13391, 13221,
        13056, 12895, 12738, 12584, 12434, 12288, 12145, 12006,
        11869, 11736, 11605, 11478, 11353, 11231, 11111, 10995,
        10880, 10768, 10658, 10550, 10445, 10341, 10240, 10141,
        10043, 9947, 9854, 9761, 9671, 9582, 9495, 9410,
        9326, 9243, 9162, 9082, 9004, 8927, 8852, 8777,
        8704, 8632, 8561, 8492, 8423, 8356, 8290, 8224,
        8160, 8097, 8034, 7973, 7913, 7853, 7795, 7737,
        7680, 7624, 7569, 7514, 7461, 7408, 7355, 7304,
        7253, 7203, 7154, 7105, 7057, 7010, 6963, 6917,
        6872, 6827, 6782, 6739, 6695, 6653, 6611, 6569,
        6528, 6487, 6447, 6408, 6369, 6330, 6292, 6254,
        6217, 6180, 6144, 6108, 6073, 6037, 6003, 5968,
        5935, 5901, 5868, 5835, 5803, 5771, 5739, 5708,
        5677, 5646, 5615, 5585, 5556, 5526, 5497, 5468,
        5440, 5412, 5384, 5356, 5329, 5302, 5275, 5249,
        5222, 5196, 5171, 5145, 5120, 5095, 5070, 5046,
        5022, 4998, 4974, 4950, 4927, 4904, 4881, 4858,
        4836, 4813, 4791, 4769, 4748, 4726, 4705, 4684,
        4663, 4642, 4622, 4601, 4581, 4561, 4541, 4522,
        4502, 4483, 4464, 4445, 4426, 4407, 4389, 4370,
        4352, 4334, 4316, 4298, 4281, 4263, 4246, 4229,
        4212, 4195, 4178, 4161, 4145, 4128, 4112, 4096
    };

    v = MAX(v, g);
    v = MAX(v, r);
    vmin = MIN(vmin, g);
    vmin = MIN(vmin, r);

    diff = v - vmin;
    vr = v == r ? -1 : 0;
    vg = v == g ? -1 : 0;

    s = diff * div_table[v] >> hsv_shift;
    h = (vr & (g - b)) +
        (~vr & ((vg & (b - r + 2 * diff)) + ((~vg) & (r - g + 4 * diff))));
    h = ((h * div_table[diff] * 15 + (1 << (hsv_shift + 6))) >> (7 + hsv_shift))\
        + (h < 0 ? 30*6 : 0);

    _h = (uchar)h;
    _s = (uchar)s;
    _v = (uchar)v;

}

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Converts RGB to YCrCb.
///
///  \param r Red channel value.
///  \param g Green channel value.
///  \param b Blue channel value.
///  \param y Luminosity (Grayscale)
///  \param cr Chrominance in red channel.
///  \param cb Chrominance in blue channel.
///
////////////////////////////////////////////////////////////////////////////////////
inline void RGB2YCrCb(const int r,
                      const int g,
                      const int b,
                      int &y,
                      int &cr,
                      int &cb)
{
    double Y, Cr, Cb;
    Y=0.299*r + 0.587*g + 0.114*b;
    Cr=(r-Y)*0.713 + 128;
    Cb=(b-Y)*0.564 + 128;

    y = (Y > 255) ? 255 : (int)(Y);
    cr = (Cr > 255) ? 255 : (int)(Cr);
    cb = (Cb > 255) ? 255 : (int)(Cb);
}

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \brief Digitizes the color cube so it is a straight lookup to the pixel
///         value
///   
///   This avoids checking threshold values for every pixel to determine whether
///   to set it to high or low. Here we pre-calculate it and store it directly
///   in the color cube. 
///   
///   WARNING: Do not do do more training or save the
///   colorcube after digitizing it because the original values have been
///   overwritten. To "undigitize" the colorcube, load it again.
/// 
///   \param thresh Used to filter out miss classifications.  Any values
///                    greater than this within the color space equal the color
///                    trained on. 
///   \param high The resulting output value if the pixel is "classified"
///   \param low The resulting output value if the pixel is not "classified"
///
////////////////////////////////////////////////////////////////////////////////////
int CvColorClassifier::digitize(double thresh, int high /*= 255*/, int low /*= 0*/)
{
	int result = false;

	if (mVotes && !mDigitized)
	{
		for (int i = 0; i < mVotesDimensions; i++)
		{
			for (int j = 0; j < mVotesDimensions; j++)
			{
				for (int k = 0; k < mVotesDimensions; k++)
				{
					if (mVotes[mVotesDimensions*mVotesDimensions*i + j*mVotesDimensions + k] > (float)thresh)
					{
						mVotes[mVotesDimensions*mVotesDimensions*i + j*mVotesDimensions + k] = (float)high;
					}
					else
					{
						mVotes[mVotesDimensions*mVotesDimensions*i + j*mVotesDimensions + k] = (float)low;
					}
				}
			}

			mDigitized = true;
			result = true;
		}
	}

	return result;
}

/* End of file */
