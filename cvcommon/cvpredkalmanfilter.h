#ifndef CVPREDKALMANFILTER_H
#define CVPREDKALMANFILTER_H

#include <cv.h>
#include <cxcore.h>

class PredKalmanFilter : public cv::KalmanFilter
{
public:
	PredKalmanFilter() : cv::KalmanFilter() {}
	PredKalmanFilter(int dynamParams, int measureParams, int controlParams=0) : cv::KalmanFilter(dynamParams, measureParams, controlParams) {}
	void backup(PredKalmanFilter &kf)
	{

		kf.statePre = this->statePre.clone();
		kf.statePost = this->statePost.clone();
		kf.transitionMatrix = this->transitionMatrix.clone();
		kf.controlMatrix = this->controlMatrix.clone();
		kf.measurementMatrix = this->measurementMatrix.clone();
		kf.processNoiseCov = this->processNoiseCov.clone();
		kf.measurementNoiseCov = this->measurementNoiseCov.clone();
		kf.errorCovPre = this->errorCovPre.clone();
		kf.gain = this->gain.clone();
		kf.errorCovPost = this->errorCovPost.clone();
		kf.temp1 = this->temp1.clone();
		kf.temp2 = this->temp2.clone();
		kf.temp3 = this->temp3.clone();
		kf.temp4 = this->temp4.clone();
		kf.temp5 = this->temp5.clone();
	}

	cv::Mat predictN(int n)
	{
		cv::Mat pred;
		PredKalmanFilter current;
		backup(current);

		for (int i = 0; i < n; i++)
		{
			if (i != 0)
				correct(this->measurementMatrix*this->statePre);
			predict();
		}

		pred = this->statePre.clone();

		// Restore
		current.backup(*this);

		return pred;
	}

	cv::Mat predictLiteN(int n)
	{
		cv::Mat pred = this->statePost.clone();
		for (int i = 0; i < n; i++)
		{
			pred = this->transitionMatrix*pred;
			//float thresh = 0.02;
			//if (pred.at<float>(0,2) > thresh)
			//	pred.at<float>(0,2) = thresh;
			//if (pred.at<float>(0,2) < -thresh)
			//	pred.at<float>(0,2) = -thresh;
			//thresh = 1.5;
			//if (pred.at<float>(0,1) > thresh)
			//	pred.at<float>(0,1) = thresh;
			//if (pred.at<float>(0,1) < -thresh)
			//	pred.at<float>(0,1) = -thresh;
		}

		return pred;
	}

	cv::Mat predictFeedN(int n)
	{
		cv::Mat pred = this->statePost.clone();
		PredKalmanFilter current;
		backup(current);

		// temp2 = H*P'(k)
		temp2 = measurementMatrix * errorCovPre;
		// temp3 = temp2*Ht + R
		gemm(temp2, measurementMatrix, 1, measurementNoiseCov, 1, temp3, cv::GEMM_2_T); 
		// temp4 = inv(temp3)*temp2 = Kt(k)
		solve(temp3, temp2, temp4, cv::DECOMP_SVD);
		// K(k)
		gain = temp4.t();

		cv::Mat measurement(1,1,CV_32F);
		cv::Mat measurement2(1,1,CV_32F);
		measurement.at<float>(0,0) = statePost.at<float>(0,0);
		measurement2.at<float>(0,0) = statePost.at<float>(0,0);
		for (int i = 0; i < n; i++)
		{
			//measurement2.at<float>(0,0) = measurement.at<float>(0,0);
			//measurement.at<float>(0,0) = statePost.at<float>(0,0);
			// update the state: x'(k) = A*x(k)
			statePre = transitionMatrix*statePost;
			// temp5 = z(k) - H*x'(k)
			temp5 = measurement2 - measurementMatrix*statePre;
			// x(k) = x'(k) + K(k)*temp5
			statePost = statePre + gain*temp5;
		}

		pred = statePost.clone();

		// Restore
		current.backup(*this);
		return pred;

		//in = inv(s.H*s.P*s.Ht+s.R);
		//K = s.P*s.Ht*in;
		//for j = 1:ahead
		//	s.z(1:3) = s.x(1:3);
		//	s.x = s.A*s.x;
		//	s.x = s.x + K*(s.z-s.H*s.x);
		//	Xp(t,:) = s.x;
		//end
	}
private:
};

#endif