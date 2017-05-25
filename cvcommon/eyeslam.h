#ifndef EYESLAM_H
#define EYESLAM_H

#include <cv.h>
#include <highgui.h>

#include "thread.h"
#include "veintracer.h"
#include "mutex.h"
#include "queue.h"
#include "scanMatching.h"

#include <opencv2/core/core.hpp>
#undef min
#undef max
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/video/tracking.hpp>


#include "array.h"
#include "math3dgeom.h"
//#include "MicronVisionSim_Def.h"

//#include "KalmanFilter.h"

#include <iostream>
#include <fstream>

struct TimingInfo
{
	double lowResScanMatching;
	double highResScanMatching;
	double ScanMatchingAlgo;
	double veinDetectionAlgo;
	double eyeSLAMTime;
	double veinPtsLenght;
	double lowResMap;
	///
};

class KalmanPosVel2D
{
public:
	KalmanPosVel2D(float measNoise = 1e2, float procNoise = 1e-2) 
	{ 
		kf.init(2, 1, 0);
		kf.transitionMatrix = *(cv::Mat_<float>(2, 2) << 1, 1, 0, 1);

		cv::setIdentity(kf.measurementMatrix);
		cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(procNoise));
		cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(measNoise));
		cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1));

		cv::randn(kf.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));

		//float m[4] = {1, 1, 0, 1};

		//kf.init(2, 1, 0); 
		//kf.transitionMatrix = cv::Mat(2, 2, CV_32F, m);

		//cv::setIdentity(kf.measurementMatrix);
		//cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-5));

		//kf.measurementNoiseCov.at<float>(0,0) = 1e-1;
		//cv::setIdentity(kf.errorCovPost, cv::Scalar(1));
		//
		//cv::randn(kf.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));

		//cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-1));
		//cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
		//cv::setIdentity(kf.errorCovPost, cv::Scalar(1));
		//cv::randn(kf.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));
	}
	~KalmanPosVel2D() {}

	cv::Mat predictState() { return kf.predict(); }
	float predict() 
	{ 
		//cv::Mat state = kf.predict(); return state.at<float>(0,0); 

		cv::Mat state = kf.predict();
		float prediction = state.at<float>(0);
		return prediction;

	}
	float correct(float val) 
	{ 
		//cv::Mat meas(1,1,CV_32F); meas.at<float>(0,0) = val; kf.correct(meas); 
		cv::Mat measurement = cv::Mat::zeros(1, 1, CV_32F);
		measurement = val;
		return kf.correct(measurement).at<float>(cv::Point(0, 0));
	}

private:
	cv::KalmanFilter kf;
	
};

class EyeSLAM : public Thread
{
public:
	EyeSLAM();
	~EyeSLAM();

	virtual void execute();
	virtual void cleanup() {}

	void start() { Thread::createThread(); }
	//void stop() { Thread::stopThread(10); }
	void stop(void); 
	void slamIt(const cv::Mat &img);
	int getQueueLength() { return mInput.size(); }

	int numSkipped() { return mSkipped; }

	int knnNaiveCPU(const std::vector<cv::Point> &map, const std::vector<cv::Point> pts, std::vector<int> &idx, std::vector<double> &dist);
	//int knnBruteGPU(const cv::Mat &map, const cv::Mat &pts, std::vector<int> &idx, std::vector<double> &dist);

	int icp(const cv::Mat &map, const cv::Mat &pts, cv::Mat &R, cv::Mat &T);
	int icp(const std::vector<cv::Point2f> &map, const std::vector<cv::Point2f> &pts, cv::Mat &R, cv::Mat &T, int &numInliers);
	int absor(const cv::Mat &map, const cv::Mat &pts, cv::Mat &R, cv::Mat &T);
	int absor(const std::vector<cv::Point2f> &map, const std::vector<cv::Point2f> &pts, cv::Mat &R, cv::Mat &T);

	void getTrans(cv::Mat &Rf, cv::Mat &Tf) { mOutMutex.enter(); Rf = mRf.clone(); Tf = mTf.clone(); mOutMutex.leave(); }

	static void applyTrans(std::vector<cv::Point2f> &pts, const cv::Mat &R, const cv::Mat &T);

	// added By Daniel : export results on .cvs file
	void OutputTransformFile(std::string FileName);

	float flann_knn(cv::Mat& m_destinations, cv::Mat& m_object, std::vector<int>& ptpairs, std::vector<float>& dists = std::vector<float>()) {
		//// find nearest neighbors using FLANN
		//cv::Mat m_indices(m_object.rows, 1, CV_32S);
		//cv::Mat m_dists(m_object.rows, 1, CV_32F);

		//cv::Mat dest_32f; m_destinations.convertTo(dest_32f,CV_32FC2);
		//cv::Mat obj_32f; m_object.convertTo(obj_32f,CV_32FC2);

		//assert(dest_32f.type() == CV_32F);

		//cv::flann::Index flann_index(dest_32f, cv::flann::KDTreeIndexParams(2));  // using 2 randomized kdtrees
		//flann_index.knnSearch(obj_32f, m_indices, m_dists, 1, cv::flann::SearchParams(64) );

		//int* indices_ptr = m_indices.ptr<int>(0);
		////float* dists_ptr = m_dists.ptr<float>(0);
		//for (int i=0;i<m_indices.rows;++i) {
		//	ptpairs.push_back(indices_ptr[i]);
		//}

		//dists.resize(m_dists.rows);
		//m_dists.copyTo(Mat(dists));

		//return cv::sum(m_dists)[0];
	}

	void reset() { mMutex.enter(); mFirst = 1; bFlagTR = false; mMutex.leave(); }

	// Warped points registerd using ICP (final output)
	void getVeinMap(std::vector<cv::Point2f> &map) { mOutMutex.enter(); map = mWarpedMapPts; mOutMutex.leave();}
	void getVeinMapfromH(std::vector<cv::Point2f> &map);

	// This gives you the raw points detected in the image
	void getVeinPts(std::vector<cv::Point> &pts) { mOutMutex.enter(); pts = mVeinPts; mOutMutex.leave();}
	// This gives you the occupancy map 
	void getOccMap(cv::Mat &map) { mOutMutex.enter(); map = mOccMap.clone(); mOutMutex.leave();}

	void getGaussianOccMap(cv::Mat &map) { mOutMutex.enter(); map = mGaussianOccMap.clone(); mOutMutex.leave(); }

	// Get the rotation/translation of registered image to the first frame (bcb: added last minute, might not compile)
	void getTransformation(cv::Mat &R, cv::Mat &T) { R = mRf.clone(); T = mTf.clone(); }

	// Get matrix border of the image in the map coordinate system
	// src : to have the input image dimension
	// occMap : to set the size of the subOccMapBound
	// subOccMapBound : the Mat that will include the boundaries
	// R & T : transformation matrix from ICP
	// Trans : TransMap  : to center the data into the Map
	// margin : to set a margin in the border of the image
	void GetSubOccMapBoundaries(cv::Mat src, cv::Mat Map, cv::Mat &subOccMapBound, cv::Mat R, cv::Mat T, int margin = 0);

	void EyeSLAM::FindMinMax(std::vector<cv::Point2f> Data, int &Xmin, int &Xmax, int &Ymin, int &Ymax);
	

	int isDry() { return mIsDry; }

	//Added by Daniel to apply vector translation
	void TranslationData(std::vector<cv::Point2f> &pts, int Tx, int Ty);
	void TranslationData(std::vector<cv::Point> &pts, int Tx, int Ty);
	
	void GetmDisplay(cv::Mat &Display) { mOutMutex.enter(); Display = mDisplay.clone(); mOutMutex.leave(); }

	//Function to extract the data points from the veinTracer algorithm
	void GetVeinTracerDataPts(std::vector<std::vector<cv::Point>> &SavedInitialPts, std::vector<std::vector<cv::Point>> &SavedSeedPts, std::vector<std::vector<cv::Point>> &SavedVeinPts)
	{
		mVeinTracer.GetDataPts(SavedInitialPts, SavedSeedPts, SavedVeinPts);
	}

	// Get the timing info and store it into the TimingInfo structure
	void GetTiming(TimingInfo &frame_timing_info)
	{
		// Get the timing info from the scanMatching algo
		mScanMatching.GetTiming(frame_timing_info.lowResScanMatching, frame_timing_info.highResScanMatching, frame_timing_info.veinPtsLenght, frame_timing_info.lowResMap);
		// Get the timing info from EyeSLAM
		frame_timing_info.veinDetectionAlgo = mVeinDetectionAlgo;
		frame_timing_info.ScanMatchingAlgo = mScanMatchingAlgo;
		frame_timing_info.eyeSLAMTime = mEyeSLAMTime;
	}

	void ResetTiming()
	{
		mVeinDetectionAlgo = 0;
		mScanMatchingAlgo = 0;
		mEyeSLAMTime = 0;
	}

	std::vector<cv::Point2f> getBorderPoints() { std::vector<cv::Point2f> results; mMutex.enter(); results = mBorderPts; mMutex.leave(); return results; }

public:
	cv::Mat Tr2dC, Tr2dX;
	cv::Mat get3dTransform(void){return TR3d;};
	cv::Mat get2dTransform(void){return Tr2dC;};

	void setHomography(cv::Mat h){  mMutex.enter(); H = h; mMutex.leave();};
	void setProjectionMatrix(CvMat *pMl, CvMat *pMr){ mMutex.enter(); m_pMl = pMl; m_pMr = pMr;  mMutex.leave(); };


private:
	CvMat *m_pMl, *m_pMr;
	cv::Mat H;
	Math3dGeom m_geom;
public:
	cv::Mat TR3d;
	bool bFlagTR;

	cv::Mat mGaussianOccMap;

	std::vector<cv::Point2f> mBorderPts;


private:
	Mutex mMutex;
	Mutex mOutMutex;
	Queue<cv::Mat> mInput;
	int mSkipped;
	VeinTracer mVeinTracer;
	scanMatching mScanMatching;
	int iteration;
	int mFirst;
	cv::Mat R, T;
	std::vector<cv::Point2f> mWarpedMapPts;
	std::vector<cv::Point> mVeinPts;

	// Timing variables
	double mVeinDetectionAlgo;
	double mScanMatchingAlgo;
	double mEyeSLAMTime;

	//Added by Daniel
	//vecteur translation to center data in the map
	cv::Point TransMap;

	// for debbug mode, used to display a mat from the eyeSLAM code
	cv::Mat mDisplay;
	cv::Mat mOccMap;
	//Mat Translation and Rotation
	cv::Mat mRf, mTf;
	//Vector of mRf and mTf
	std::vector<cv::Mat> mVecRf, mVecTf;

	int mICPfail;
	int mFail;

	KalmanPosVel2D *mTxFilterKF, *mTyFilterKF, *mThetaFilterKF;

	cv::Point2f calcMeanPt(const std::vector<cv::Point2f> &pts);
	void subPt(std::vector<cv::Point2f> &pts, cv::Point2f pt);
	void vecOfPtsToMat(const std::vector<cv::Point2f> &pts, cv::Mat &mat);
	cv::Mat estimate3DTransform(void);
	std::vector<int> randperm(int n)
	{
		int i, j, t;
		std::vector<int> perm;

		perm.reserve(n);
		for(i=0; i<n; i++)
			perm.push_back(i);

		for(i=0; i<n; i++)
		{
			j = rand()%(n-i)+i;
			t = perm[j];
			perm[j] = perm[i];
			perm[i] = t;
		}

		return perm;
	}

	int mIsDry;
};

#endif
