////////////////////////////////////////////////////////////////////////////////
///
///   Filename: dynamicdraw.cpp
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

#include "dynamicdraw.h"

DynamicDrawLine::DynamicDrawLine()
{
	pt1.x = pt1.y = pt2.x = pt2.y = 0;
	color = CV_RGB(0,0,0);
	thickness = 1;
	line_type = 8;
	shift = 0;
	spec = 0;
}

DynamicDrawLine::DynamicDrawLine(CvPoint pt1, CvPoint pt2, CvScalar color, int thickness/*=1*/, int line_type/*=8*/, int shift/*=0*/, int spec)
{
	//set(pt1, pt2, color, thickness, line_type, shift);
	set(pt1, pt2, color, thickness, line_type, shift, spec);
}


void DynamicDrawLine::set(CvPoint pt1, CvPoint pt2, CvScalar color, int thickness/*=1*/, int line_type/*=8*/, int shift/*=0*/, int spec)
{
	this->pt1=pt1; 
	this->pt2=pt2; 
	this->color=color; 
	this->thickness=thickness; 
	this->line_type=line_type; 
	this->shift=0;	
	this->spec = spec;
}

void DynamicDrawLine::draw(IplImage *img)
{
	cvLine(img, pt1, pt2, color, thickness, line_type, shift);
}

void DynamicDrawLine::save(FILE *fp)
{
	fprintf(fp, "Type: Line %d %d  %d %d  %d  %d  %d %d\n", pt1.x, pt1.y, pt2.x, pt2.y, colorToInt(color), thickness, line_type, shift);
}

void DynamicDrawLine::saveSpec(FILE *fp)
{
	fprintf(fp, "Type %d: Line %d %d  %d %d  %d  %d  %d %d\n", spec, pt1.x, pt1.y, pt2.x, pt2.y, colorToInt(color), thickness, line_type, shift);
}

void DynamicDrawLine::load(FILE *fp)
{
	int color = 0;
	fscanf(fp, "Type: Line %d %d  %d %d  %d  %d  %d %d\n", &pt1.x, &pt1.y, &pt2.x, &pt2.y, &color, &thickness, &line_type, &shift);
	this->color = intToColor(color);
}
void DynamicDrawLine::loadShort(FILE *fp)
{
	int color = 0;
	fscanf(fp, "%d %d  %d %d  %d  %d  %d %d\n", &pt1.x, &pt1.y, &pt2.x, &pt2.y, &color, &thickness, &line_type, &shift);
	this->color = intToColor(color);
}

void DynamicDrawLine::setSpec(int spec)
{
	this->spec =spec;
	
}
int  DynamicDrawLine::loadSpec()
{
	return this->spec;
	
}

DynamicDrawCircle::DynamicDrawCircle()
{
	center.x = center.y = 0;
	radius = 0;
	color = CV_RGB(0,0,0);
	thickness=1;
	line_type=8;
	shift=0;
	spec = 0;
}

//DynamicDrawCircle::DynamicDrawCircle(CvPoint center, int radius, CvScalar color, int thickness/*=1*/, int line_type/*=8*/, int shift/*=0*/)
//{
//	set(center, radius, color, thickness, line_type, shift);
//}
DynamicDrawCircle::DynamicDrawCircle(CvPoint center, int radius, CvScalar color, int thickness/*=1*/, int line_type/*=8*/, int shift/*=0*/, int spec/*=0*/)
{

	set(center, radius, color, thickness, line_type, shift, spec);
}


void DynamicDrawCircle::set(CvPoint center, int radius, CvScalar color, int thickness/*=1*/, int line_type/*=8*/, int shift/*=0*/, int Spec/*=0*/)
{
	this->center = center;
	this->radius = radius;
	this->color = color;
	this->thickness = thickness;
	this->line_type = line_type;
	this->shift = shift;
	this->spec = Spec;
}

void DynamicDrawCircle::draw(IplImage *img)
{
	if (radius < img->width+img->height && radius > 0)
		cvCircle(img, center, radius, color, thickness, line_type, shift);
}

void DynamicDrawCircle::save(FILE *fp)
{
	fprintf(fp, "Type: Circle %d %d  %d  %d  %d  %d\n", center.x, center.y, radius, colorToInt(color), thickness, line_type, shift);
}

void DynamicDrawCircle::saveSpec(FILE *fp)
{
	fprintf(fp, "Type %d: Circle %d %d  %d  %d  %d  %d\n", spec, center.x, center.y, radius, colorToInt(color), thickness, line_type, shift);
}

void DynamicDrawCircle::load(FILE *fp)
{
	int color = 0;
	fscanf(fp, "Type: Circle %d %d  %d  %d  %d  %d\n", &center.x, &center.y, &radius, &color, &thickness, &line_type, &shift);
	this->color = intToColor(color);
}

void DynamicDrawCircle::loadShort(FILE *fp)
{
	int color = 0;
	fscanf(fp, "%d %d  %d  %d  %d  %d\n", &center.x, &center.y, &radius, &color, &thickness, &line_type, &shift);
	this->color = intToColor(color);
}

void DynamicDrawCircle::setSpec(int spec)
{
	this->spec =spec;
	
}

int DynamicDrawCircle::loadSpec()
{
	return this->spec;
	
}

DynamicDrawRect::DynamicDrawRect()
{
	pt1.x = pt1.y = pt2.x = pt2.y = 0;
	color = CV_RGB(0,0,0);
	thickness = 1;
	line_type = 8;
	shift = 0;
	spec = 0;
}

DynamicDrawRect::DynamicDrawRect(CvPoint pt1, CvPoint pt2, CvScalar color, int thickness/*=1*/, int line_type/*=8*/, int shift/*=0*/, int spec)
{
	set(pt1, pt2, color, thickness, line_type, shift, spec);
}

DynamicDrawRect::DynamicDrawRect(CvRect r, CvScalar color, int thickness/*=1*/, int line_type/*=8*/, int shift/*=0*/, int spec)
{
	set(cvPoint(r.x, r.y), cvPoint(r.x+r.width, r.y+r.height), color, thickness, line_type, shift, spec);
}

void DynamicDrawRect::set(CvPoint pt1, CvPoint pt2, CvScalar color, int thickness/*=1*/, int line_type/*=8*/, int shift/*=0*/,int spec)
{
	this->pt1=pt1; 
	this->pt2=pt2; 
	this->color=color; 
	this->thickness=thickness; 
	this->line_type=line_type; 
	this->shift=0;	
	this->spec=spec;	
}

void DynamicDrawRect::draw(IplImage *img)
{
	cvRectangle(img, pt1, pt2, color, thickness, line_type, shift);
}

void DynamicDrawRect::save(FILE *fp)
{
	fprintf(fp, "Type: Rect %d %d  %d %d  %d  %d  %d %d\n", pt1.x, pt1.y, pt2.x, pt2.y, colorToInt(color), thickness, line_type, shift);
}
void DynamicDrawRect::saveSpec(FILE *fp)
{
	fprintf(fp, "Type %d: Rect %d %d  %d %d  %d  %d  %d %d\n", spec, pt1.x, pt1.y, pt2.x, pt2.y, colorToInt(color), thickness, line_type, shift);
}

void DynamicDrawRect::load(FILE *fp)
{
	int color = 0;
	fscanf(fp, "Type: Rect %d %d  %d %d  %d  %d  %d %d\n", &pt1.x, &pt1.y, &pt2.x, &pt2.y, &color, &thickness, &line_type, &shift);
	this->color = intToColor(color);
}

void DynamicDrawRect::loadShort(FILE *fp)
{
	int color = 0;
	fscanf(fp, "%d %d  %d %d  %d  %d  %d %d\n", &pt1.x, &pt1.y, &pt2.x, &pt2.y, &color, &thickness, &line_type, &shift);
	this->color = intToColor(color);
}

void DynamicDrawRect::setSpec(int spec)
{
	this->spec =spec;
	
}
int DynamicDrawRect::loadSpec()
{
	return this->spec;
	
}