/*==================================================================================

    Filename:  image.cpp

    Copyright 2007 Daniel Barber
                   Robotics Laboratory
                   University of Central Florida
                   http://robotics.ucf.edu

    Program Contents:
    -------------------------------------------------------------------------------
    Wrapper class for allocating and releasing IplImage data.
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
#include "image.h"
#include <assert.h>


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Constructor.
///
////////////////////////////////////////////////////////////////////////////////////
Image::Image() : mImage(NULL) 
{
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Creates image data.
///
///   \param d Depth in bits (IPL_DEPTH_8U, IPL_DEPTH_8S, IPL_DEPTH_16S,
///            IPL_DEPTH_32S, IPL_DEPTH_32F, and IPL_DEPTH_64F.
///   \param h Height of image.
///   \param w Width of image.
///   \param c Number of channels (supports up to 4).
///
///   \return Access to image.
///
////////////////////////////////////////////////////////////////////////////////////
Image::Image(const int w,
             const int h,
             const int d,
             const int c)
{
    mImage = NULL;
    create(w, h, d, c);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Creates image data.
///
///   \param d Depth in bits (IPL_DEPTH_8U, IPL_DEPTH_8S, IPL_DEPTH_16S,
///            IPL_DEPTH_32S, IPL_DEPTH_32F, and IPL_DEPTH_64F.
///   \param s Size of the image.
///   \param c Number of channels (supports up to 4).
///
///   \return Access to image.
///
////////////////////////////////////////////////////////////////////////////////////
Image::Image(const CvSize s,
             const int d,
             const int c)
{
    mImage = NULL;
    create(s, d, c);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Constructor.
///   
///   \param another Image to be equal to (copy).
///
////////////////////////////////////////////////////////////////////////////////////
Image::Image(const IplImage *another) : mImage(NULL) 
{
    if(another)
    {
        copy(another);
    }
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Constructor.
///
///   \param Image to be equal to (copy).
///
////////////////////////////////////////////////////////////////////////////////////
Image::Image(const Image &another) : mImage(NULL) 
{
    copy(another);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Destructor.
///
////////////////////////////////////////////////////////////////////////////////////
Image::~Image()
{
    if(mImage)
    {
        cvReleaseImage(&mImage);
    }
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \return Size of image data in bytes.
///
////////////////////////////////////////////////////////////////////////////////////
int Image::bytes() const  { return mImage ? mImage->imageSize : 0; }
////////////////////////////////////////////////////////////////////////////////////
///
///   \return Size of image (height/rows and columns/width).
///     
////////////////////////////////////////////////////////////////////////////////////
CvSize Image::size() const 
{
    if(mImage)
    {   
        return cvSize(mImage->width, mImage->height);
    }
    return cvSize(0, 0); 
}
////////////////////////////////////////////////////////////////////////////////////
///
///   \return Size of image (height/rows and columns/width).
///     
////////////////////////////////////////////////////////////////////////////////////
CvSize Image::roiSize() const 
{
    if(mImage)
    {   
        if(mImage->roi)
            return cvSize(mImage->roi->width, mImage->roi->height);
        else //  Whole image selected
            return cvSize(mImage->width, mImage->height);
    }
    return cvSize(0, 0); 
}
////////////////////////////////////////////////////////////////////////////////////
///
///   \breif Clears the region of interest for the image.  When cleared roi is
///   NULL and the whole picture is selected for processing.
///
////////////////////////////////////////////////////////////////////////////////////
void Image::resetROI()
{
    if(mImage)
    {
        cvResetImageROI(mImage);
    }
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \breif Set the region of interest for the image.  This affects what areas
///   of an image are processed by filters.
///
///   \param yoffset Starting row in image.
///   \param xoffset Starting column in image.
///   \param height How much to travel to end row (yoffset + height).
///   \param width How much to travel to end col (xoffset + width).
///
////////////////////////////////////////////////////////////////////////////////////
void Image::setROI(const int yoffset,
                   const int xoffset,
                   const int height,
                   const int width)
{
    if(mImage)
        cvSetImageROI( mImage, cvRect( xoffset, yoffset, width, height ));
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \breif Set the region of interest for the image.  This affects what areas
///   of an image are processed by filters.
///
///   \param _roi The region of interest.
///
////////////////////////////////////////////////////////////////////////////////////
void Image::setROI(const CvRect _roi)
{
    if(mImage)
        cvSetImageROI(mImage, _roi);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \return Region of interest.  If image is NULL and rectangle with values of
///   0 is returned.
///
////////////////////////////////////////////////////////////////////////////////////
CvRect Image::roi() const { return mImage ? cvGetImageROI(mImage) : cvRect(0, 0, 0, 0); }

////////////////////////////////////////////////////////////////////////////////////
///
///   \return Size of image data in bytes.
///
////////////////////////////////////////////////////////////////////////////////////
int Image::pixelSize() const  { return mImage ? mImage->imageSize : 0; }
////////////////////////////////////////////////////////////////////////////////////
///
///   \return Version Id (0).
///
////////////////////////////////////////////////////////////////////////////////////
int Image::id() const  { return mImage ? mImage->ID : 0; }
////////////////////////////////////////////////////////////////////////////////////
///
///   \return Number of channels (1, 2, 3, 4).
///
////////////////////////////////////////////////////////////////////////////////////
int Image::channels() const { return mImage ? mImage->nChannels : 0;}
////////////////////////////////////////////////////////////////////////////////////
///
///   \return Pixel depth in bits (IPL_DEPTH_8U, IPL_DEPTH_8S, IPL_DEPTH_16S,
///           IPL_DEPTH_32S, IPL_DEPTH_32F, and IPL_DEPTH_64F.
///
////////////////////////////////////////////////////////////////////////////////////
int Image::depth() const { return mImage ? mImage->depth : 0; }
////////////////////////////////////////////////////////////////////////////////////
///
///   \return Pixel depth in bits (IPL_DEPTH_8U, IPL_DEPTH_8S, IPL_DEPTH_16S,
///           IPL_DEPTH_32S, IPL_DEPTH_32F, and IPL_DEPTH_64F.
///
////////////////////////////////////////////////////////////////////////////////////
int Image::d() const { return mImage ? mImage->depth : 0; }
////////////////////////////////////////////////////////////////////////////////////
///
///   \return Data order (0 - interleved color channels, 1 - seperate color channels).
///
////////////////////////////////////////////////////////////////////////////////////
int Image::dataOrder() const  { return mImage ? mImage->dataOrder : 0; }
////////////////////////////////////////////////////////////////////////////////////
///
///   \return Origin (0 - top-left origin, 1 - bottom-left origin like Windows bitmaps).
///
////////////////////////////////////////////////////////////////////////////////////
int Image::origin() const  { return mImage ? mImage->origin : 0; }
////////////////////////////////////////////////////////////////////////////////////
///
///   \return Alignment of image rows (4 or 8) ignored by OpenCV for widthStep.
///
////////////////////////////////////////////////////////////////////////////////////
int Image::alignment() const { return mImage ? mImage->align : 0; }
////////////////////////////////////////////////////////////////////////////////////
///
///   \return Image width in pixels.
///
////////////////////////////////////////////////////////////////////////////////////
int Image::width() const  { return mImage ? mImage->width : 0; }
////////////////////////////////////////////////////////////////////////////////////
///
///   \return Image height in pixels.
///
////////////////////////////////////////////////////////////////////////////////////
int Image::height() const  { return mImage ? mImage->height : 0; }
////////////////////////////////////////////////////////////////////////////////////
///
///   \return Image width in pixels.
///
////////////////////////////////////////////////////////////////////////////////////
int Image::w() const  { return mImage ? mImage->width : 0; }
////////////////////////////////////////////////////////////////////////////////////
///
///   \return Image height in pixels.
///
////////////////////////////////////////////////////////////////////////////////////
int Image::h() const  { return mImage ? mImage->height : 0; }
////////////////////////////////////////////////////////////////////////////////////
///
///   \return Size of aligned image row in bytes.  Mainly used for pixel/channel access.
///
////////////////////////////////////////////////////////////////////////////////////
int Image::widthStep() const  { return mImage ? mImage->widthStep : 0; }

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Deletes allocated memory.
///
////////////////////////////////////////////////////////////////////////////////////
void Image::release()
{
    if(mImage)
        cvReleaseImage(&mImage);
    mImage = NULL;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Deletes allocated memory.
///
////////////////////////////////////////////////////////////////////////////////////
void Image::destroy()
{
    if(mImage)
        cvReleaseImage(&mImage);
    mImage = NULL;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets all values in image to zero.
///
////////////////////////////////////////////////////////////////////////////////////
void Image::clear()
{
    if(mImage)
    {
        cvSetZero(mImage);
    }
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \return Pointer to image data.
///
////////////////////////////////////////////////////////////////////////////////////
unsigned char *Image::ptr()
{
    if(mImage)
    {
        return (unsigned char *)mImage->imageData;
    }

    return NULL;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \return Pointer to image data.
///
////////////////////////////////////////////////////////////////////////////////////
const unsigned char *Image::ptr() const
{
    if(mImage)
    {
        return (unsigned char *)mImage->imageData;
    }

    return NULL;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Creates image data.
///
///   \param d Depth in bits (IPL_DEPTH_8U, IPL_DEPTH_8S, IPL_DEPTH_16S,
///            IPL_DEPTH_32S, IPL_DEPTH_32F, and IPL_DEPTH_64F.
///   \param h Height of image.
///   \param w Width of image.
///   \param c Number of channels (supports up to 4).
///
///   \return Access to image.
///
////////////////////////////////////////////////////////////////////////////////////
Image &Image::create(const int w,
                     const int h,
                     const int d,
                     const int c)
{
    //  Verify correct arguments
    assert(h > 0 && w > 0 && d > 0 && c > 0);
    if(mImage)
    {
        if( mImage->height == h &&
            mImage->width == w &&
            mImage->depth == d &&
            mImage->nChannels == c)
        {
            if(mImage->roi)
                cvResetImageROI(mImage);

            return *this;
        }
    }
    release();  //  Release current allocated memory
    mImage = cvCreateImage(cvSize(w, h), d, c);
    clear();
    resetROI();
    return *this;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Creates image based on the settings of the image passed, but does not
///   copy the data in the image.
///
///   Function does take into account the image ROI.
///
///   \param another The image to be of the same type.
///
///   \return Access to image.
///
////////////////////////////////////////////////////////////////////////////////////
Image &Image::create(const Image &another)
{
    if(*this != another)
    {
        if(another.mImage)
        {
            if(another.mImage->roi)
            {
                create(another.mImage->roi->width,
                       another.mImage->roi->height,
                       another.mImage->depth,
                       another.mImage->nChannels);
            }
            else
            {
                create(another.mImage->width, 
                       another.mImage->height,
                       another.mImage->depth,
                       another.mImage->nChannels);
            }
        }
        else
            release();
    }

    return *this;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Creates image based on the settings of the image passed, but does not
///   copy the data in the image.
///
///   Function does take into account the image ROI.
///
///   \param another The image to be of the same type.
///
///   \return Access to image.
///
////////////////////////////////////////////////////////////////////////////////////
Image &Image::create(const IplImage *another)
{
    if(*this != another)
    {
        if(another)
        {
            if(another->roi)
            {
                create(another->roi->width,
                       another->roi->height,
                       another->depth,
                       another->nChannels);
            }
            else
            {
                create(another->width, 
                       another->height,
                       another->depth,
                       another->nChannels);
            }
        }
        else
            release();
    }

    return *this;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Creates image data.
///
///   \param d Depth in bits (IPL_DEPTH_8U, IPL_DEPTH_8S, IPL_DEPTH_16S,
///            IPL_DEPTH_32S, IPL_DEPTH_32F, and IPL_DEPTH_64F.
///   \param s Size of the image.
///   \param c Number of channels (supports up to 4).
///
///   \return Access to image.
///
////////////////////////////////////////////////////////////////////////////////////
Image &Image::create(const CvSize s,
                     const int d,
                     const int c)
{
    //  Verify correct arguments
    assert(s.height > 0 && s.width > 0 && c > 0);
    if(mImage)
    {
        if( mImage->height == s.height &&
            mImage->width == s.width &&
            mImage->depth == d &&
            mImage->nChannels == c)
        {
            if(mImage->roi)
                cvResetImageROI(mImage);

            return *this;
        }
    }
    release();  //  Release current allocated memory
    mImage = cvCreateImage(s, d, c);
    clear();
    resetROI();
    return *this;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Converts pixels to seperate channels/planes.
///
///   \param dst0 Channel 1 (if NULL data not retrieved).
///   \param dst1 Channel 2 (if NULL data not retrieved).
///   \param dst2 Channel 3 (if NULL data not retrieved).
///   \param dst3 Channel 4 (if NULL data not retrieved).
///
////////////////////////////////////////////////////////////////////////////////////
void Image::pixToPlane(CvArr *dst0, CvArr *dst1,
                    CvArr *dst2, CvArr *dst3)
{
    cvCvtPixToPlane(mImage, dst0, dst1, dst2, dst3);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \return Pointer to IplImage structure.
///
////////////////////////////////////////////////////////////////////////////////////
IplImage *Image::image() const { return mImage; }
////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Implicit operator for compatibility with existing OpenCV functions.
///
////////////////////////////////////////////////////////////////////////////////////
Image::operator IplImage() const { return *mImage; }
////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Implicit operator for compatibility with existing OpenCV functions.
///
////////////////////////////////////////////////////////////////////////////////////
Image::operator IplImage *() const { return mImage; }
////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Implicit operator for compatibility with existing OpenCV functions.
///
////////////////////////////////////////////////////////////////////////////////////
Image::operator const IplImage *() const { return mImage; }
////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Implicit operator for compatibility with existing OpenCV functions.
///
////////////////////////////////////////////////////////////////////////////////////
Image::operator IplImage *&() { return mImage; }
////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Implicit operator for compatibility with existing OpenCV functions.
///
////////////////////////////////////////////////////////////////////////////////////
Image::operator CvArr *() const { return mImage; }
////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Implicit operator for compatibility with existing OpenCV functions.
///
////////////////////////////////////////////////////////////////////////////////////
Image::operator const CvArr *() const { return mImage; }


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Flips the current image horizontally, vertically, etc.
///
///   \param mode 0 means flipping around x-axis, flip_mode > 0 (e.g. 1) 
///          means flipping around y-axis and flip_mode < 0 (e.g. -1) means 
///         flipping around both axises. See also the discussion below for the formulas.
///
///   The function cvFlip flips the array in one of different 3 ways 
///   (row and column indices are 0-based):
///   
///   B(i,j)=A(rows(A)-i-1,j) if flip_mode = 0
///   B(i,j)=A(i,cols(A)-j-1) if flip_mode > 0
///   B(i,j)=A(rows(A)-i-1,cols(A)-j-1) if flip_mode < 0
///
///   The typical scenaria of the function use are:
///
///   * vertical flipping of the image (flip_mode > 0) to switch between 
///     top-left and bottom-left image origin, which is typical operation 
///     in video processing under Win32 systems.
///   * horizontal flipping of the image with subsequent horizontal shift 
///     and absolute difference calculation to check for a vertical-axis 
///     symmetry (flip_mode > 0)
///   * simultaneous horizontal and vertical flipping of the image with 
///     subsequent shift and absolute difference calculation to check 
///     for a central symmetry (flip_mode < 0)
///   * reversing the order of 1d point arrays(flip_mode > 0)
///
////////////////////////////////////////////////////////////////////////////////////
Image &Image::flip(const int mode)
{
    cvFlip(mImage, NULL, mode);
    return *this;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Creates internal pixel image from differant planes.
///
///   Any plane that is NULL does not have data added to the image.  src0 Must not
///   be NULL.  Uses the cvCvtPlaneToPix function.
///
///   \param src0 Plane 1 data.
///   \param src1 Plane 2 data.
///   \param src2 Plane 3 data.
///   \param src3 Plane 4 data.
///
////////////////////////////////////////////////////////////////////////////////////
Image &Image::planeToPix(CvArr *src0, 
                             CvArr *src1,
                             CvArr *src2, 
                             CvArr *src3)
{
    cvCvtPlaneToPix(mImage, src0, src1, src2, src3);
    return *this;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Copies data from parameter to internal data members.
///
///   If an image ROI is set in another, than it is used.
///
///   \param another Image to copy data from.
///
////////////////////////////////////////////////////////////////////////////////////
Image &Image::copy(const Image &another)
{
    if(this != &another)
    {
        if(!another.mImage)
        {
            release();
        }
        else
        {
            if(another.mImage->roi)
            {
                //  Check for resize or no allocated
                //  memory space.
                if(!mImage || 
                    mImage->width != another.mImage->roi->width   || 
                    mImage->height != another.mImage->roi->height ||
                    mImage->nChannels != another.mImage->nChannels ||
                    mImage->depth != another.mImage->depth)
                {
                    create(another.mImage->roi->width, another.mImage->roi->height, another.mImage->depth, another.mImage->nChannels);
                }
            }
            else
            {
                //  Check for resize or no allocated
                //  memory space.
                if(!mImage || 
                    mImage->width != another.mImage->width   || 
                    mImage->height != another.mImage->height ||
                    mImage->nChannels != another.mImage->nChannels ||
                    mImage->depth != another.mImage->depth)
                {
                    create(another.mImage->width, another.mImage->height, another.mImage->depth, another.mImage->nChannels); 
                }
            }

            cvCopyImage(another.mImage, mImage);
        }
    }

    return *this;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Copies data from parameter to internal data members.
///
///   If an image ROI is set in another, than it is used.
///
///   \param another Pointer to IplImage data for copying.
///
////////////////////////////////////////////////////////////////////////////////////
Image &Image::copy(const IplImage *another)
{
    if(!another || another->width == 0 || another->height == 0)
    {
        release();
    }
    else
    {
        if(!another->roi)
        {
            //  Check for resize or no allocated
            //  memory space.
            if(!mImage || 
                mImage->width != another->width  || 
                mImage->height != another->height||
                mImage->nChannels != another->nChannels ||
                mImage->depth != another->depth)
            {
                create(another->width, another->height, another->depth, another->nChannels); 
            }
        }
        else
        {
            //  Check for resize or no allocated
            //  memory space.
            if(!mImage || 
                mImage->width != another->roi->width    || 
                mImage->height != another->roi->height  ||
                mImage->nChannels != another->nChannels ||
                mImage->depth != another->depth)
            {
                create(another->roi->width, another->roi->height, another->depth, another->nChannels);
            }
        }

        cvCopyImage(another, mImage);
    }
    
    return *this;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Copies data from parameter to internal data members and flips while copying.
///
///   If an image ROI is set in another, than it is used.
///
///   \param another Image to copy data from.
///   \param flipMode Flips the input image when copying using the cvFlip function.
///
////////////////////////////////////////////////////////////////////////////////////
Image &Image::copy(const Image &another, const int flipMode)
{
    if(this != &another)
    {
        if(!another.mImage)
        {
            release();
        }
        else
        {
            if(another.mImage->roi)
            {
                //  Check for resize or no allocated
                //  memory space.
                if(!mImage || 
                    mImage->width != another.mImage->roi->width   || 
                    mImage->height != another.mImage->roi->height ||
                    mImage->nChannels != another.mImage->nChannels ||
                    mImage->depth != another.mImage->depth)
                {
                    create(another.mImage->roi->width, another.mImage->roi->height, another.mImage->depth, another.mImage->nChannels);
                }
            }
            else
            {
                //  Check for resize or no allocated
                //  memory space.
                if(!mImage || 
                    mImage->width != another.mImage->width   || 
                    mImage->height != another.mImage->height ||
                    mImage->nChannels != another.mImage->nChannels ||
                    mImage->depth != another.mImage->depth)
                {
                    create(another.mImage->width, another.mImage->height, another.mImage->depth, another.mImage->nChannels); 
                }
            }
            
            if(another.mImage->roi)
            {
                cvSetImageROI( mImage, 
                               cvRect( another.mImage->roi->xOffset, 
                                       another.mImage->roi->yOffset, 
                                       another.mImage->roi->width,
                                       another.mImage->roi->height));
            }
            else if(!another.mImage->roi && mImage->roi)
            {
                cvResetImageROI(mImage);
            }
            cvFlip(another.mImage, mImage, flipMode);
        }
    }

    return *this;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Copies data from parameter to internal data members and flips while copying.
///
///   If an image ROI is set in another, than it is used.
///
///   \param another Image to copy data from.
///   \param flipMode Flips the input image when copying using the cvFlip function.
///
////////////////////////////////////////////////////////////////////////////////////
Image &Image::copy(const IplImage *another, const int flipMode)
{
    if(!another || another->width == 0 || another->height == 0)
    {
        release();
    }
    else
    {
        if(!another->roi)
        {
            //  Check for resize or no allocated
            //  memory space.
            if(!mImage || 
                mImage->width != another->width  || 
                mImage->height != another->height||
                mImage->nChannels != another->nChannels ||
                mImage->depth != another->depth)
            {
                create(another->width, another->height, another->depth, another->nChannels); 
            }
        }
        else
        {
            //  Check for resize or no allocated
            //  memory space.
            if(!mImage || 
                mImage->width != another->roi->width    || 
                mImage->height != another->roi->height  ||
                mImage->nChannels != another->nChannels ||
                mImage->depth != another->depth)
            {
                create(another->roi->width, another->roi->height, another->depth, another->nChannels);
            }
        }
        
        if(another->roi)
        {
            cvSetImageROI( mImage, 
                           cvRect( another->roi->xOffset, 
                                   another->roi->yOffset, 
                                   another->roi->width,
                                   another->roi->height));
        }
        else if(!another->roi && mImage->roi)
        {
            cvResetImageROI(mImage);
        }
        
        cvFlip(another, mImage, flipMode);
    }
    
    return *this;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Overloaded equal operator.
///
////////////////////////////////////////////////////////////////////////////////////
Image &Image::operator =(const Image &another)
{
    return copy(another);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Overloaded equal operator.  Does not copy the pointer values, but
///   instead copies the data pointed to by the IplImage pointer.
///
////////////////////////////////////////////////////////////////////////////////////
Image &Image::operator =(const IplImage *another)
{
    return copy(another);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Overloaded equal operator.
///
////////////////////////////////////////////////////////////////////////////////////
Image &Image::operator =(const IplImage &another)
{
    return copy(&another);
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Checks to see if the dimensions, depth, pixels size, number of
///   channels and channel sequence are not equal.  Check for not same type of
///   image.
///
////////////////////////////////////////////////////////////////////////////////////
bool Image::operator !=(const Image &another) const
{
    if(mImage && another.mImage &&
       mImage->height == another.mImage->height &&
       mImage->width == another.mImage->width &&
       mImage->depth == another.mImage->depth &&
       mImage->origin == another.mImage->origin &&
       mImage->widthStep == another.mImage->widthStep &&
       mImage->imageSize == another.mImage->imageSize &&
       mImage->dataOrder == another.mImage->dataOrder &&
       mImage->nChannels == another.mImage->nChannels &&
       mImage->nSize == another.mImage->nSize &&
       mImage->colorModel[0] == another.mImage->colorModel[0] &&
       mImage->colorModel[1] == another.mImage->colorModel[1] &&
       mImage->colorModel[2] == another.mImage->colorModel[2] &&
       mImage->colorModel[3] == another.mImage->colorModel[3] &&
       mImage->channelSeq[0] == another.mImage->channelSeq[0] &&
       mImage->channelSeq[1] == another.mImage->channelSeq[1] &&
       mImage->channelSeq[2] == another.mImage->channelSeq[2] &&
       mImage->channelSeq[3] == another.mImage->channelSeq[3])    
    {
        
           return false;
    }
    return true;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Checks to see if the dimensions, depth, pixels size, number of
///   channels and channel sequence are not equal.  Check for not same type of
///   image.
///
////////////////////////////////////////////////////////////////////////////////////
bool Image::operator !=(const IplImage &another) const
{
    if(mImage &&
       mImage->height == another.height &&
       mImage->width == another.width &&
       mImage->depth == another.depth &&
       mImage->origin == another.origin &&
       mImage->widthStep == another.widthStep &&
       mImage->imageSize == another.imageSize &&
       mImage->dataOrder == another.dataOrder &&
       mImage->nChannels == another.nChannels &&
       mImage->nSize == another.nSize &&
       mImage->colorModel[0] == another.colorModel[0] &&
       mImage->colorModel[1] == another.colorModel[1] &&
       mImage->colorModel[2] == another.colorModel[2] &&
       mImage->colorModel[3] == another.colorModel[3] &&
       mImage->channelSeq[0] == another.channelSeq[0] &&
       mImage->channelSeq[1] == another.channelSeq[1] &&
       mImage->channelSeq[2] == another.channelSeq[2] &&
       mImage->channelSeq[3] == another.channelSeq[3])
    {
        return false;
    }
    return true;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Checks to see if the dimensions, depth, pixels size, number of
///   channels and channel sequence are equal.  Check for same type of
///   image and same size.
///
////////////////////////////////////////////////////////////////////////////////////
bool Image::operator==(const Image &another) const
{
    return !(*this != another);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Checks to see if the dimensions, depth, pixels size, number of
///   channels and channel sequence are equal.  Check for same type of
///   image and same size.
///
////////////////////////////////////////////////////////////////////////////////////
bool Image::operator==(const IplImage &another) const
{
    return !(*this != another);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Checks to see if the dimensions, depth, pixels size, number of
///   channels and channel sequence are equal.  Check for same type of
///   image and same size.
///
////////////////////////////////////////////////////////////////////////////////////
bool Image::operator==(const IplImage *another) const
{
    return !(*this != another);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Overloaded equal operator.
///
////////////////////////////////////////////////////////////////////////////////////
bool Image::operator !=(const IplImage *another) const
{
    if(mImage && another &&
       mImage->height == another->height &&
       mImage->width == another->width &&
       mImage->depth == another->depth &&
       mImage->origin == another->origin &&
       mImage->widthStep == another->widthStep &&
       mImage->imageSize == another->imageSize &&
       mImage->dataOrder == another->dataOrder &&
       mImage->nChannels == another->nChannels &&
       mImage->nSize == another->nSize &&
       mImage->colorModel[0] == another->colorModel[0] &&
       mImage->colorModel[1] == another->colorModel[1] &&
       mImage->colorModel[2] == another->colorModel[2] &&
       mImage->colorModel[3] == another->colorModel[3] &&
       mImage->channelSeq[0] == another->channelSeq[0] &&
       mImage->channelSeq[1] == another->channelSeq[1] &&
       mImage->channelSeq[2] == another->channelSeq[2] &&
       mImage->channelSeq[3] == another->channelSeq[3])
    {
        return false;
    }
    return true;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Uses the cvLoadImage function to open an saved image.  Supports most
///   image formats including jpeg, png, bmp, tiff, etc.
///
///   This functions also ensure the origin is in the top left corner.
///
///   \param fname The file name of an image in the current directory.
///   \param code  If less than 0, number of channels is based on image file, if
///                0 than the file is loaded as grayscale, otherwise loaded
///                as three channels regardless.
///
///   \return 0 on failure, 1 on success.
///
////////////////////////////////////////////////////////////////////////////////////
int Image::load(const char *fname, int code)
{
    release();
    mImage = cvLoadImage(fname, code);
    
    if(!mImage)
        return 0;
    //  Ensure the origin is in the top left corner
    if(mImage->origin != IPL_ORIGIN_TL)
        flip();

    return 1;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Uses the cvLoadImage function to open an saved image.  Supports most
///   image formats including jpeg, png, bmp, tiff, etc.
///
///   This functions also ensure the origin is in the top left corner.
///
///   \param fname The file name of an image in the current directory.
///   \param code  If less than 0, number of channels is based on image file, if
///                0 than the file is loaded as grayscale, otherwise loaded
///                as three channels regardless.
///
///   \return 0 on failure, 1 on success.
///
////////////////////////////////////////////////////////////////////////////////////
int Image::loadImage(const char *fname, int code)
{
    release();
    mImage = cvLoadImage(fname, code);
    
    if(!mImage)
        return 0;
    //  Ensure the origin is in the top left corner
    if(mImage->origin != IPL_ORIGIN_TL)
        flip();

    return 1;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Saves the image to disk.
///
///   \param fname The file name to save to in current directory.
///
///   \return 0 on failure, 1 on success.
///
////////////////////////////////////////////////////////////////////////////////////
int Image::save(const char *fname)
{
    if(!mImage)
        return 0;

    return cvSaveImage(fname, mImage);
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Saves the image to disk.
///
///   \param fname The file name to save to in current directory.
///
///   \return 0 on failure, 1 on success.
///
////////////////////////////////////////////////////////////////////////////////////
int Image::saveImage(const char *fname)
{
    if(!mImage)
        return 0;

    return cvSaveImage(fname, mImage);
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Overlays one image ontop of another (handles mixed gray/color)
///
///   \param top Top image to overlay ontop of the image
///   \param row Row in the bottom image to start the overlay
///   \param col Column in the bottom image to start the overlay
///
///   \return True on success, false otherwise
///
////////////////////////////////////////////////////////////////////////////////
bool Image::overlay(const Image &top, unsigned int row, unsigned int col)
{
	int result = false;

	// The bottom image is this one
	Image &bottom = *this;

	if (bottom.width() && bottom.height() && top.width() && top.height() && 
		(int)row + top.height() < bottom.height() && (int)col + top.width() < bottom.width())
	{
		unsigned int height = top.height();
		unsigned int width = top.width();
		unsigned int bottomChannels = bottom.channels();
		unsigned int topChannels = top.channels();
		unsigned int topStep = top.widthStep();
		unsigned int bottomStep = bottom.widthStep();
		uchar *bottomPtr = (uchar *)(bottom.ptr());
		uchar *topPtr = (uchar *)(top.ptr());
		int pix = 0;

		if (topChannels == bottomChannels)
		{
			for(unsigned int l=row;l<(height+row);l++) 
			{
				for(unsigned int j=col;j<(width+col);j++) 
				{
					for(unsigned int k=0;k<bottomChannels;k++)
					{
						bottomPtr[(l)*bottomStep+(j)*bottomChannels+k] = topPtr[(l-row)*topStep+(j-col)*bottomChannels+k];
					}
				}
			}

			result = true;
		}
		else 
		{
			// Perform copying with grayscale <=> color conversion
			for(unsigned int l=row;l<(height+row);l++) 
			{
				for(unsigned int j=col;j<(width+col);j++) 
				{
					pix = 0;
					for(unsigned int k=0;k<topChannels;k++)
					{
						pix += topPtr[(l-row)*topStep+(j-col)*topChannels+k];
					}

					pix /= topChannels;

					for(unsigned int k=0;k<bottomChannels;k++)
					{
						bottomPtr[(l)*bottomStep+(j)*bottomChannels+k] = pix;
					}
				}
			}

			result = true;
		}
	}

	return !!result;
}
/* End of file */
