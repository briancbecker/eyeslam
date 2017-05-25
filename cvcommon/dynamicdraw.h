////////////////////////////////////////////////////////////////////////////////
///
///   Filename: dynamicdraw.h
///
///   Copyright (C) 2008-2012   Brian C. Becker             www.BrianCBecker.com
///   License: LGPL             RI @ CMU                          www.ri.cmu.edu
///                             Surgical Mechatronics Lab       Micron Project
///
///   Description: A way to store primitives for delayed, dynamic drawing later
///   --------------------------------------------------------------------------
///	  Often, one might want to draw useful annotations on images. However, for
///	  archival purposes, it doesn't make sense to draw directly onto the images
///	  because that is a destructive process - you might want to later want the
///	  original, untouched images/video. In this case, it makes sense to have a
///	  way to accumulate all drawings or annotations to a frame and then have a
///	  unified way to either draw them onto the image for display or to save them
///	  to disk along with the original video so you dynamically draw them later. 
///	  
///	  Requires OpenCV to draw shapes into IplImages
///
////////////////////////////////////////////////////////////////////////////////

#ifndef DYNAMICDRAW_H
#define DYNAMICDRAW_H

#include <cv.h>
#include <stdio.h>

////////////////////////////////////////////////////////////////////////////////
///   
///   \class DynamicDraw 
///   \brief The parent class for all drawing objects on OpenCV images
///   
///   DynamicDraw is a polymorphic parent class for all other drawing classes so
///   you can easily collect an array or queue of drawing primitives. Child
///   drawing objects must implement draw, save, load, loadShort, and copy.
///   
///   Currently, all drawing happens on OpenCV IplImages. Also there is no smart
///   way of using a factory that knows about all the types of drawing objects,
///   so loading from files must first check the first part of the line to
///   determine the drawing type, instantiate the appropriate type, and call the
///   loadShort function to load just the raw data for the drawing object. 
///   
////////////////////////////////////////////////////////////////////////////////
class DynamicDraw
{
public:
	DynamicDraw(){}
	virtual ~DynamicDraw(){}

	/// Pure virtual function for drawing the object on the image
	virtual void draw(IplImage *img) = 0;
	/// Pure virtual function for saving the drawing object to file
	virtual void save(FILE *fp) = 0;
	virtual void saveSpec(FILE *fp) = 0;

	/// Pure virtual function for loading the type of drawing object and data from file
	virtual void load(FILE *fp) = 0;
	/// Pure virtual function for loading just the data of the drawing object from file
	virtual void loadShort(FILE *fp) = 0;
	/// Pure virtual function for copying the drawing object

	virtual	void setSpec(int spec) = 0;
	virtual	int loadSpec() =0;

	virtual DynamicDraw* copy() = 0;

	/// Helper function to convert a CvScalar color to a raw integer
	static int colorToInt(CvScalar s)
	{
		return (int)((int)s.val[0] << 0) | (int)((int)s.val[1] << 8) | (int)((int)s.val[2] << 16) | (int)((int)s.val[3] << 24);
	}
	/// Helper function to convert a a raw color integer to a CvScalar
	static CvScalar intToColor(int i)
	{
		CvScalar s;
		s.val[0] = (i >> 0) & 0xFF;
		s.val[1] = (i >> 8) & 0xFF;
		s.val[2] = (i >> 16) & 0xFF;
		s.val[3] = (i >> 24) & 0xFF;
		return s;
	}

};

////////////////////////////////////////////////////////////////////////////////
///   
///   \class DynamicDrawLine 
///   \brief Specifies a line drawing object, see OpenCV's cvLine
///   
////////////////////////////////////////////////////////////////////////////////
class DynamicDrawLine : public DynamicDraw
{
public:
	DynamicDrawLine();
	DynamicDrawLine(CvPoint pt1, CvPoint pt2, CvScalar color, int thickness=1, int line_type=8, int shift=0, int spec=0);
	~DynamicDrawLine(){}

	//void set(CvPoint pt1, CvPoint pt2, CvScalar color, int thickness=1, int line_type=8, int shift=0);
	void set(CvPoint pt1, CvPoint pt2, CvScalar color, int thickness=1, int line_type=8, int shift=0, int spec=0);

	void draw(IplImage *img);
	void save(FILE *fp);
	void saveSpec(FILE *fp);

	void load(FILE *fp);
	void loadShort(FILE *fp);

	void setSpec(int spec);
	int loadSpec();

	DynamicDraw* copy() { return new DynamicDrawLine(pt1, pt2, color, thickness, line_type, shift, spec); }

//private:
	CvPoint pt1;
	CvPoint pt2;
	CvScalar color;
	int thickness;
	int line_type;
	int shift;

	int spec; //modified by Yang, 08/09/2013, for defining the specification of the dynamic drawing
};

////////////////////////////////////////////////////////////////////////////////
///   
///   \class DynamicDrawCircle
///   \brief Specifies a circle drawing object, see OpenCV's cvCircle
///   
////////////////////////////////////////////////////////////////////////////////
class DynamicDrawCircle : public DynamicDraw
{
public:
	DynamicDrawCircle();
	//DynamicDrawCircle(CvPoint center, int radius, CvScalar color, int thickness=1, int line_type=8, int shift=0);
	DynamicDrawCircle(CvPoint center, int radius, CvScalar color, int thickness=1, int line_type=8, int shift=0, int spec=0);
	~DynamicDrawCircle(){}

	void set(CvPoint center, int radius, CvScalar color, int thickness=1, int line_type=8, int shift=0, int spec=0);
	//void set(CvPoint center, int radius, CvScalar color, int thickness, int line_type, int shift, int spec);

	void draw(IplImage *img);
	void save(FILE *fp);
	void saveSpec(FILE *fp);


	void load(FILE *fp);
	void loadShort(FILE *fp);
	void setSpec(int spec);
	int loadSpec();

	CvPoint getCenter() { return center; }

	DynamicDraw* copy() { return new DynamicDrawCircle(center, radius, color, thickness, line_type, shift, spec); }

//private:
	CvPoint center;
	int radius;
	CvScalar color;
	int thickness;
	int line_type;
	int shift;

	int spec; //modified by Yang, 08/09/2013, for defining the specification of the dynamic drawing
};

////////////////////////////////////////////////////////////////////////////////
///   
///   \class DynamicDrawRect
///   \brief Specifies a circle drawing object, see OpenCV's cvRectangle
///   
////////////////////////////////////////////////////////////////////////////////
class DynamicDrawRect : public DynamicDraw
{
public:
	DynamicDrawRect();
	DynamicDrawRect(CvPoint pt1, CvPoint pt2, CvScalar color, int thickness=1, int line_type=8, int shift=0, int spec=0);
	DynamicDrawRect(CvRect r, CvScalar color, int thickness=1, int line_type=8, int shift=0, int spec=0);
	//DynamicDrawRect(CvRect r, CvScalar color, int thickness=1, int line_type=8, int shift=0, int spec);

	~DynamicDrawRect(){}

	void set(CvPoint pt1, CvPoint pt2, CvScalar color, int thickness=1, int line_type=8, int shift=0, int spec=0);
	void draw(IplImage *img);
	void save(FILE *fp);
	void saveSpec(FILE *fp);


	void load(FILE *fp);
	void loadShort(FILE *fp);
	void setSpec(int spec);
	int loadSpec();

	DynamicDraw* copy() { return new DynamicDrawRect(pt1, pt2, color, thickness, line_type, shift, spec); }

//private:
	CvPoint pt1;
	CvPoint pt2;
	CvScalar color;
	int thickness;
	int line_type;
	int shift;

	int spec; //modified by Yang, 08/09/2013, for defining the specification of the dynamic drawing
};

#endif // DYNAMICDRAW_H
