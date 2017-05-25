/*==================================================================================

    Filename:  cvcolorclassifier.h

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
#ifndef _CV_COLOR_CLASSIFIER_H
#define _CV_COLOR_CLASSIFIER_H

#ifdef __cplusplus

#include <cv.h>

#include <zlib.h>       //  Required for creating compressed files

#ifndef CV_RGB_COLOR 
#define CV_RGB_COLOR 0  //  Train on the RGB color space
#endif

#ifndef CV_HSV_COLOR
#define CV_HSV_COLOR 1  //  Train on the HSV color space
#endif

#ifndef CV_YCrCb_COLOR
#define CV_YCrCb_COLOR 2  //  Train on the YIQ color space
#endif

inline void RGB2HSV(const int r,
                    const int g,
                    const int b,
                    int &h,
                    int &s,
                    int &v);
inline void RGB2YCrCb(const int r,
                      const int g,
                      const int b,
                      int &y,
                      int &cr,
                      int &cb);

////////////////////////////////////////////////////////////////////////////////////
/// 
///  \class CvColorClassifier
///  \brief Color classification software which can be trained to learn colors.
///  Colors can be learned in RGB, HSV, or YCrCb color spaces.  
///
///  Depending on usage, the classifier will automatically convert to
///  a different color space if needed.  However it will not convert from
///  HSV or YCrCb to RGB, only the other way around.
///
///  This class uses Gaussian distributions to build a 3D color cube representing
///  the distribution of the color learned.  It requires a lot of memory, but
///  is extremely fast at run-time because it uses a look-up table (color cube).
///
///  Required Libraries:  OpenCV
///
////////////////////////////////////////////////////////////////////////////////////
class CvColorClassifier
{
public:
    CvColorClassifier();
    CvColorClassifier(const CvColorClassifier &another);
    ~CvColorClassifier();
    int load(const char *filename);
    int save(const char *filename);
    int train(const IplImage *src, 
              const IplImage *mask, 
              const double sigBig,
              const double sigSmall,
              const int code = CV_RGB_COLOR);
    int classify(const IplImage *src, 
                 IplImage *dest, 
                 const double thresh,
                 const int cspace = CV_RGB_COLOR);
    int classify(const IplImage *src, 
                 IplImage *dest,
                 const IplImage *mask,
                 const double thresh,
                 const int cspace = CV_RGB_COLOR);
    int classify(IplImage *src, 
                 IplImage *dest, 
                 const double threshold,
                 CvScalar color /* ex: CV_RGB(255, 0, 0) or brightness */,
                 const int cspace = CV_RGB_COLOR);
    int classify(IplImage *src, 
                 IplImage *dest, 
                 const IplImage *mask,
                 const double threshold,
                 CvScalar color /* ex: CV_RGB(255, 0, 0) or brightness */,
                 const int cspace = CV_RGB_COLOR);
    int classify(const unsigned char r,
                 const unsigned char g,
                 const unsigned char b,
                 const double threshhold,
                 const int cspace = CV_RGB_COLOR) const;
    int classify(CvScalar color /* CV_RGB(255, 0, 0) */,
                 const double threshhold,
                 const int cspace = CV_RGB_COLOR) const;
    int classify(const IplImage *src,
                 const int y,
                 const int x,
                 const double threshhold,
                 const int cpsace = CV_RGB_COLOR) const;
    int mode() const;
    void clear();   
    void setColorSpaceScale(const int scale);
	int digitize(double thresh, int high = 255, int low = 0);
    CvColorClassifier &operator=(const CvColorClassifier &another);
protected:
    void release();
    void allocate(const double sigbig,
                  const double sigsmall);

    inline void getLocation(const IplImage *src, 
                            const int y, 
                            const int x, 
                            int &alpha, 
                            int &beta, 
                            int &gamma,
                            const int cspace = CV_RGB_COLOR) const;
    double *mBigCube;     ///<  Precalculated values for large/positive Gaussian sphere
    double *mSmallCube;   ///<  Precalculated values for small/negative Gaussian sphere
    float *mVotes;        ///<  Final Cube containing color distributions
    int mCode;            ///<  Color mode.
    int mBigWidth;        ///<  Width of the larger Gaussian 2D array
    int mSmallWidth;      ///<  Width of the smaller Gaussian 2D array
    int mScale;           ///<  How much to scale the cube by
    int mVotesDimensions; ///<  Dimensionality of mVotes.
    double mSigBig;       ///<  Size of positive Gaussian sphere
    double mSigSmall;     ///<  Size of negative Gaussian sphere
	bool mDigitized;      ///<  Straight digitized lookup or float comparisons
};

#endif
#endif
/* End of file */
