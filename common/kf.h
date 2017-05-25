#pragma once

#include <cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>


class KalmanPosVel1D
{
public:
	KalmanPosVel1D(float measNoise = 1e2, float procNoise = 1e-2, float initPos = 0.0f, float initVel = 0.0f) 
	{ 
		kf.init(2, 1, 0);
		kf.transitionMatrix = *(cv::Mat_<float>(2, 2) << 1, 1, 0, 1);

		cv::setIdentity(kf.measurementMatrix);
		cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(procNoise));
		cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(measNoise));
		cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1));

		cv::randn(kf.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));

		kf.statePre.at<float>(0) = initPos;
		kf.statePre.at<float>(1) = initVel;

	}
	~KalmanPosVel1D() {}

	cv::Mat predictState() { return kf.predict(); }
	float predict() 
	{ 
		//cv::Mat state = kf.predict(); return state.at<float>(0,0); 

		cv::Mat state = kf.predict();
		float prediction = state.at<float>(0);
	/*	
		kf.statePre.copyTo(kf.statePost);
		kf.errorCovPre.copyTo(kf.errorCovPost);*/
		
		return prediction;



	}
	float correct(float val) 
	{ 
		//cv::Mat meas(1,1,CV_32F); meas.at<float>(0,0) = val; kf.correct(meas); 
		cv::Mat measurement = cv::Mat::zeros(1, 1, CV_32F);
		measurement = val;
		cv::Mat res = kf.correct(measurement);
		return res.at<float>(0,0);
	}
	float postState(void)
	{
		return kf.statePost.at<float>(0,0);
	}
	void initState(float initPos, float initVal = 0)
	{
		kf.statePre.at<float>(0) = initPos;
		kf.statePre.at<float>(1) = initVal;
	}
	void init(float measNoise = 1e2, float procNoise = 1e-2)
	{
		kf.init(2, 1, 0);
		kf.transitionMatrix = *(cv::Mat_<float>(2, 2) << 1, 1, 0, 1);

		cv::setIdentity(kf.measurementMatrix);
		cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(procNoise));
		cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(measNoise));
		cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1));

	}
	void resetNoise(float measNoise = 1e2, float procNoise = 1e-2)
	{
		cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(procNoise));
		cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(measNoise));
	}
	void filter(float &val)
	{
		kf.predict();
		float res = correct(val);
		val = res;
	}


private:
	cv::KalmanFilter kf;
	
};

class KalmanConstVel2D
{
public:

	KalmanConstVel2D(double measNoise = 1e2, double procNoise = 1e-2,
		cv::Point2f initPos = cv::Point2f(0.0f, 0.0f), cv::Point2f initVel = cv::Point2f(0.0f, 0.0f)) 
	{ 
		init(measNoise, procNoise);

		initState(initPos, initVel);

		

	}
	~KalmanConstVel2D() {}
	cv::Point2f lastVal;

	cv::Mat predictState() { return kf.predict(); }
	cv::Point2f predict() 
	{ 
		//cv::Mat state = kf.predict(); return state.at<float>(0,0); 

		cv::Mat state = kf.predict();
		cv::Point2f prediction;
		prediction.x = state.at<float>(0);
		prediction.y = state.at<float>(1);
		//prediction.z = state.at<float>(2);
	/*	
		kf.statePre.copyTo(kf.statePost);
		kf.errorCovPre.copyTo(kf.errorCovPost);*/
		
		return prediction;



	}
	cv::Point2f correct(cv::Point2f val) 
	{ 
		//cv::Mat meas(1,1,CV_32F); meas.at<float>(0,0) = val; kf.correct(meas); 
		cv::Mat measurement = cv::Mat::zeros(2, 1, CV_32F);
		measurement.at<float>(0) = val.x;
		measurement.at<float>(1) = val.y;
		//measurement.at<float>(2) = val.z;

		cv::Mat res = kf.correct(measurement);

		lastVal = cv::Point2f(res.at<float>(0), res.at<float>(1));
		return lastVal;
	}
	cv::Point2f value(void) 
	{ 
		return lastVal;
	}
	void initState(cv::Point2f initPos, cv::Point2f initVal)
	{
		kf.statePre.at<float>(0) = initPos.x;
		kf.statePre.at<float>(1) = initPos.y;

		kf.statePre.at<float>(2) = initVal.x;
		kf.statePre.at<float>(3) = initVal.y;


	}
	void init(double measNoise = 1e2, double procNoise = 1e-2)
	{
		kf.init(4, 2);
		kf.transitionMatrix = cv::Mat::eye(4, 4, CV_32F);
		kf.transitionMatrix.at<float>(0,2) = 1.0f;
		kf.transitionMatrix.at<float>(1,3) = 1.0f;
	

		cv::setIdentity(kf.measurementMatrix);
		cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(procNoise));
		cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(measNoise));
		cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1));

		cv::randn(kf.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));

	}
	void resetNoise(double measNoise = 1e2, double procNoise = 1e-2)
	{
		cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(procNoise));
		cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(measNoise));
	}
	void filter(cv::Point2f &val)
	{
		kf.predict();
		cv::Point2f res = correct(val);
		val = res;
	}
	

private:
	cv::KalmanFilter kf;
	
};

class KalmanConstVel3D
{
public:

	KalmanConstVel3D(double measNoise = 1e2, double procNoise = 1e-2,
		cv::Point3f initPos = cv::Point3f(0.0, 0.0, 0.0), cv::Point3f initVel = cv::Point3f(0.0, 0.0, 0.0)) 
	{ 
		init(measNoise, procNoise);

		initState(initPos, initVel);

		

	}
	~KalmanConstVel3D() {}
	cv::Point3f lastVal;

	cv::Mat predictState() { return kf.predict(); }
	cv::Point3f predict() 
	{ 
		//cv::Mat state = kf.predict(); return state.at<float>(0,0); 

		cv::Mat state = kf.predict();
		cv::Point3f prediction;
		prediction.x = state.at<float>(0);
		prediction.y = state.at<float>(1);
		prediction.z = state.at<float>(2);
	/*	
		kf.statePre.copyTo(kf.statePost);
		kf.errorCovPre.copyTo(kf.errorCovPost);*/
		
		return prediction;



	}
	cv::Point3f correct(cv::Point3f val) 
	{ 
		//cv::Mat meas(1,1,CV_32F); meas.at<float>(0,0) = val; kf.correct(meas); 
		cv::Mat measurement = cv::Mat::zeros(3, 1, CV_32F);
		measurement.at<float>(0) = val.x;
		measurement.at<float>(1) = val.y;
		measurement.at<float>(2) = val.z;

		cv::Mat res = kf.correct(measurement);

		lastVal = cv::Point3f(res.at<float>(0), res.at<float>(1), res.at<float>(2));
		return lastVal;
	}
	cv::Point3f value(void) 
	{ 
		return lastVal;
	}
	void initState(cv::Point3f initPos, cv::Point3f initVal)
	{
		kf.statePre.at<float>(0) = initPos.x;
		kf.statePre.at<float>(1) = initPos.y;
		kf.statePre.at<float>(2) = initPos.z;

		kf.statePre.at<float>(3) = initVal.x;
		kf.statePre.at<float>(4) = initVal.y;
		kf.statePre.at<float>(5) = initVal.z;

	}
	void init(double measNoise = 1e2, double procNoise = 1e-2)
	{
		kf.init(6, 3);
		kf.transitionMatrix = cv::Mat::eye(6, 6, CV_32F);
		kf.transitionMatrix.at<float>(0,3) = 1.0f;
		kf.transitionMatrix.at<float>(1,4) = 1.0f;
		kf.transitionMatrix.at<float>(2,5) = 1.0f;


		cv::setIdentity(kf.measurementMatrix);
		cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(procNoise));
		cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(measNoise));
		cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1));

		cv::randn(kf.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));

	}
	void resetNoise(double measNoise = 1e2, double procNoise = 1e-2)
	{
		cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(procNoise));
		cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(measNoise));
	}
	void filter(cv::Point3f &val)
	{
		kf.predict();
		cv::Point3f res = correct(val);
		val = res;
	}
	

//private:
public:
	cv::KalmanFilter kf;
	
};