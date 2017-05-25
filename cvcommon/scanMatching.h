#ifndef SCANMATCHING_H
#define SCANMATCHING_H

#include <cv.h>
#include <highgui.h>

#include <opencv2/core/core.hpp>

#include <opencv2/video/tracking.hpp>

#include <iostream>
#include <fstream>
#include <Windows.h>
#include <math.h> 

#ifndef M_PI
#define M_PI    3.14159265358979323846
#endif

class scanMatching
{
public:
	scanMatching();
	~scanMatching();

	// Generate a low resolution matrix
	void lowResolutionMat(cv::Mat &input, int x, int y = 0);

	void bruteForce(cv::Mat map, cv::Mat img, std::vector<cv::Point2f> veinPts, cv::Mat &T, cv::Mat &R);

	// Multi resolution method
	// map : current build map
	// veinPts : detected vein points on the current frame
	// T & R : rigid transformation model (I/O variable)
	// xPxSize & yPxSize : used to calibrate the pixel size of the low resolution matrix, if = 0 the value is chosen automaticaly
	void multiResolution(cv::Mat map, std::vector<cv::Point2f> veinPts, cv::Mat &T, cv::Mat &R, int &fail, int xPxSize = 0, int yPxSize = 0);

	void applyTrans(std::vector<cv::Point2f> &pts, const cv::Mat &R, const cv::Mat &T);

	void insideLoop(cv::Mat map, std::vector<cv::Point2f> veinPts, int dx, int dy, int dtheta, int &bdx, int &bdy, int &bdtheta, float &bestScore, int &bestMatch);

	void GetTiming(double &lowResScanMatching, double &highResScanMatching, double &veinPtsLength, double &lowResMap)
	{
		lowResScanMatching = mLowResScanMatching;
		highResScanMatching = mHighResScanMatching;
		veinPtsLength = mVeinPtsLength;
		lowResMap = mLowResMap;
	}

private:
	// indicate the number of points outside the borders of the map
	int mPtsOutx;
	int mPtsOuty;


	// Computation time variables
	double mLowResScanMatching;
	double mHighResScanMatching;
	double mVeinPtsLength;
	double mLowResMap;
};

#endif