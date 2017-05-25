/*==================================================================================

    Filename:  image.h

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
#ifndef _CV_IMAGE_H
#define _CV_IMAGE_H

#ifdef __cplusplus

#include <cv.h>
#include <highgui.h>

////////////////////////////////////////////////////////////////////////////////////
///
///   \class Image
///   \brief C++ Wrapper class for storing IplImage data information for use
///   in OpenCV.  This class takes care of all memory allocation and deletion for
///   images, which can help prevent accidental memory leaks.  Implicit operators
///   exist for 
///
///   Required Libraries:
///   OpenCV 
///
////////////////////////////////////////////////////////////////////////////////////
class Image
{
public:
    Image();
    Image(const CvSize s, const int d, const int c);
    Image(const int w, const int h, const int d, const int c);
    Image(const IplImage *img);
    Image(const Image &another);
    ~Image();
    int bytes() const;                  ///<  Image size in bytes
    int pixelSize() const;              ///<  Size of pixel/image data (height*widthStep)
    int id() const;                     ///<  Version (= 0)
    int channels() const;               ///<  Number of channels in image
    int d() const;                      ///<  Depth (pixel depth)
    int depth() const;                  ///<  Depth (pixel depth)
    int dataOrder() const;              ///<  0 = interleved, 1 = separate color channels
    int origin() const;                 ///<  0 = top-left, 1 = bottom-left origin 
    int alignment() const;              ///<  Alignment of image rows (4 or 8)
    int w() const;                      ///<  Width of the image (columns)
    int width() const;                  ///<  Width of the image (columns)
    int h() const;                      ///<  Height of the image (height)
    int height() const;                 ///<  Height of the image (height)
    int widthStep() const;              ///<  Size of aligned image row in bytes
    void release();                     ///<  Releases allocated memory 
    void destroy();                     ///<  Releases allocated memory
    void clear();                       ///<  Sets all values in image to zero
    void resetROI();                    ///<  Clear the current region of interest (select whole picture)
    void setROI(const int yoffset,
                const int xoffset,
                const int height,
                const int width);       ///<  Set the region of interest for filters/image processing
    void setROI(const CvRect _roi);     ///<  Set the region of interest for filters/image processing
    void pixToPlane(CvArr *dst0, 
                    CvArr *dst1 = NULL,
                    CvArr *dst2 = NULL, 
                    CvArr *dst3 = NULL);///<  Convert to color planes
    CvSize size() const;                ///<  Size of image (height and width)
    CvSize roiSize() const;             ///<  Size of the ROI
    CvRect roi() const;                 ///<  Get the ROI (Region of Interest)
    unsigned char *ptr();               ///<  Pointer to image data
    const unsigned char *ptr() const;   ///<  Pointer to image data
    int load(const char *fname, 
             int code = -1);            ///<  Load an image
    int loadImage(const char *fname, 
                  int code = -1);       ///<  Load an image
    int save(const char *fname);        ///<  Save an image
    int saveImage(const char *fname);   ///<  Save an image
    Image &create(const int w, 
                  const int h, 
                  const int d, 
                  const int c);
    Image &create(const CvSize s, 
                  const int d, 
                  const int c);
    Image &create(const Image &another);    ///<  Creates a same type of image (doesn't copy data)
    Image &create(const IplImage *another); ///<  Ditto
    Image &planeToPix(CvArr *src0, 
                      CvArr *src1 = NULL,
                      CvArr *src2 = NULL, 
                      CvArr *src3 = NULL);
    Image &flip(const int mode = 0);
	bool overlay(const Image &top, unsigned int row, unsigned int col);
    IplImage *image() const;
    operator IplImage() const;
    operator IplImage*() const;
    operator IplImage*&();
    operator const IplImage*() const;
    operator CvArr*() const;
    operator const CvArr*() const;
    Image &copy(const Image &another);
    Image &copy(const Image &another, const int flipMode);
    Image &copy(const IplImage *another);
    Image &copy(const IplImage *another, const int flipMode);
    Image &operator=(const Image &another);
    Image &operator=(const IplImage *img);
    Image &operator=(const IplImage &img);
    bool operator!=(const Image &another) const;
    bool operator!=(const IplImage &img) const;
    bool operator!=(const IplImage *img) const;
    bool operator==(const Image &another) const;
    bool operator==(const IplImage &img) const;
    bool operator==(const IplImage *img) const;
protected:
    IplImage *mImage;   ///<  Image data
};
#endif

#endif
/* End of file */
