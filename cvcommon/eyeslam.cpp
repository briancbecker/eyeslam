#include "eyeslam.h"

#define STAND_ALONE_GUI 0
#define CULL_BAD_COLORS 1
#define CULL_R_THRESH 75 //75
#define CULL_RG_THRESH 1.5

Array<cv::Scalar> colors;

//Path to save the results
//#define RESULTS_TRANSFORM_PATH "C:\\Users\\dbraun\\Google Drive\\PFE\\EyeSLAM_shared\\Results\\"
#define RESULTS_TRANSFORM_PATH "C:\\Users\\Daniel\\Google Drive\\PFE\\EyeSLAM_shared\\Results\\"

#define RUN_KALMAN_FILTER 1


// timeBeginPeriod(1); // Set timer resolution to 1 ms
// startTime = timeGetTime();
// endTime = timeGetTime();

template<class T>
void imagesc(const char *name, const cv::Mat &dispF)
{
	if (!colors.size())
	{
		for (int i = 0; i < 255; i += 63)
			for (int j = 0; j < 255; j += 63)
				for (int k = 0; k < 255; k += 63)
					colors.add(CV_RGB(i, j, k));
	}

	cv::Mat mFalseColor(dispF.size(), CV_8UC3);
	double minVal = 0, maxVal = 0, range = 0;
	cv::minMaxLoc(dispF, &minVal, &maxVal);
	//minVal = -1; maxVal = numberOfDisparities;
	range = maxVal - minVal;
	int len = colors.size();

	unsigned char *ptrBase = mFalseColor.data;
	int step = mFalseColor.step;
	int rows = mFalseColor.rows;
	int cols = mFalseColor.cols;

	mFalseColor.setTo(0);
	for (int i = 0; i < rows; i++)
	{
		unsigned char * ptr = ptrBase + i*step;
		for (int j = 0; j < cols; j++)
		{
			T val = (dispF.at<T>(i, j) - minVal) / range * len;
			int idx = floor(val);
			if (val - (T)idx == 0)
			{
				*ptr++ = colors[idx][0];
				*ptr++ = colors[idx][1];
				*ptr++ = colors[idx][2];
			}
			else
			{
				float amount = (val - idx);
				*ptr++ = amount*colors[idx][0] + (1.0-amount)*colors[idx+1][0];
				*ptr++ = amount*colors[idx][1] + (1.0-amount)*colors[idx+1][1];
				*ptr++ = amount*colors[idx][2] + (1.0-amount)*colors[idx+1][2];
			}
		}
	}

	cv::imshow(name, mFalseColor);
}


EyeSLAM::EyeSLAM()
{
	mSkipped = 0;
	iteration = 0;
	mFirst = 1;
	mTxFilterKF = 0;
	mTyFilterKF = 0;
	mThetaFilterKF = 0;
	mIsDry = false;
	Tr2dC = cv::Mat::eye(3,3,CV_32F);
	TR3d = cv::Mat::eye(4, 4, CV_64FC1);
	bFlagTR = false;
	mFail = 0;
}

EyeSLAM::~EyeSLAM()
{
	this->stopThread();

	delete mTxFilterKF;
	delete mTyFilterKF;
	delete mThetaFilterKF;

	mTxFilterKF = 0;
	mTyFilterKF = 0;
	mThetaFilterKF = 0;
	mIsDry = false;
}

void EyeSLAM::execute()
{
	cv::Mat veinMap, veinOut;
	std::vector<cv::Point> veinPts;
	cv::Mat inImg;
	cv::Mat occMap;

	float occDecay = 0.01 * 255;
	float occMax = 5*255;
	float occMin = 0;
	float blurScale = 1;
	int minInliers = 50;
	int skelThresh = 127;

	// This variable is here to select the desirable mode 
	// mapMode = 1 -> with preprocessing
	// mapMode = 2 -> without preprocessing, Gaussian blur on the newOccMap (outside the occMap loop)
	// mapMode = 3 -> without preprocessing, Gaussian blur on the occMap (inside the occMap loop)
	// mapMode = 4 -> without preprocessing, Gaussian blur on the veinPoints, before updating the occMap
	int mapMode = 2;

	int ctr = 0;
	int totTime = 0;
	int Fail = 0;

	//while the thread is still running
	while (!Thread::quitFlag())
	{
		
		// ============================	//
		//	 Generating image channels	//
		// ============================	//
		cv::Mat img;
		mMutex.enter();
		int first = 0;
		// if Queue<cv::Mat> mInput is not empty
		if (mInput.size())
		{
			//mInput is not empty
			mIsDry = false;
			mInput.dequeue(img);
			while (mInput.size())
			{
				mSkipped++;
				//remove an element from mInput and add it to img (needs to be confirm)
				mInput.dequeue(img);
			}
			first = mFirst;
			mFirst = 0;
		}
		else
		{
			//mInput is empty
			mIsDry = true;
		}
		mMutex.leave();

		if (img.rows && img.cols)
		{
			int startTime = timeGetTime();
			std::vector<cv::Mat> channels;
			//Divides the multi-channel queue img into several single-channel vector channels.
			cv::split(img, channels);
			if (channels.size() <= 1)
				// skip the rest of the loop if there is no channel
				continue;


			cv::Mat imgHSV;
			std::vector<cv::Mat> channels2;
			//Converts the image from RGB to HSV (Red Green Blue to Hue Saturation Value)
			cv::cvtColor(img, imgHSV, CV_BGR2HSV);
			//split the multichannel Mat image into singlechannels Mat image 
			cv::split(imgHSV, channels2);


			// green channel from RGB image : used for the vessel detection
			cv::Mat rimg = channels[0];
			cv::Mat gimg = channels[1];
			// value channel from HSV image : used for segmented the image to eliminate the false positive seed points.
			cv::Mat vimg = channels2[2];

			// ================================================================	//
			//			Can et al. algorithm : fast retinal detection			//
			// ================================================================	//
			img.copyTo(veinOut);
			int traceStartTime = timeGetTime();
			// experiment with histogram normalization for gimg
			mVeinTracer.trace(gimg, vimg, veinMap, veinOut, veinPts);
			// imshow of veinMap
			int traceEndTime = timeGetTime();

			mVeinDetectionAlgo = traceEndTime - traceStartTime;

			

			//	mDisplay = veinOut.clone();
			//	cvWaitKey(1);
			//	mDisplay = veinMap.clone();


			// Initialization if it is the first iteration
		//	if (occMap.rows < veinMap.rows || occMap.cols < veinMap.cols || first || occMap.rows > 4 * veinMap.rows || occMap.cols > 4 * veinMap.cols || Fail > 30)
			if (occMap.rows < gimg.rows || occMap.cols < gimg.cols || first || occMap.rows > 4 * veinMap.rows || occMap.cols > 4 * veinMap.cols || Fail > 30)
			{
				occMap.create(cv::Size(veinMap.cols, veinMap.rows), CV_32F);
				occMap.setTo(0);

				// translation vector initialization
				//TransMap = cv::Point(occMap.cols/4, occMap.rows/4);
				TransMap = cv::Point(0, 0);

				//fail ICP counter. Reset every time the program reset
				mICPfail = 0;
				Fail = 0;

				delete mTxFilterKF;
				delete mTyFilterKF;
				delete mThetaFilterKF;

				double measurementNoise = 1;
				double processNoise = 1;
				mTxFilterKF = new KalmanPosVel2D(measurementNoise, processNoise);
				mTyFilterKF = new KalmanPosVel2D(measurementNoise, processNoise);
				mThetaFilterKF = new KalmanPosVel2D(measurementNoise, processNoise);

				//mTxFilterKF = new KalmanPosVel2D(10e0, 1e-0);
				//mTyFilterKF = new KalmanPosVel2D(10e0, 1e-0);
				//mThetaFilterKF = new KalmanPosVel2D(10e0, 1e-0);

				R.create(2, 2, CV_32F);
				T.create(2, 1, CV_32F);
				R.setTo(0);
				T.setTo(0);
				R.at<float>(0, 0) = 1;
				R.at<float>(1, 1) = 1;
			}

			cv::Mat dispOccMap, distTrans, skelIn, skelDist, skelLap, skelConv, skelOut;

		//	if (mapMode == 2)
		//		dispOccMap = occMap.clone();
		//	else
				cv::convertScaleAbs(occMap, dispOccMap, 1.0 / 5.0);

			// vector containing the points that will be added to the occMap
			std::vector<cv::Point> mapPts;

			// mode with preporcessing
			if (mapMode == 1)
			{
				//  threshold(src, dst, threshold value, max, type)
				cv::threshold(dispOccMap, skelIn, skelThresh, 255, cv::THRESH_BINARY);
				//	mDisplay = skelIn.clone();
				skelDist.setTo(0);
				// distanceTransform computes the distance to the closest zero pixel for each pixel of the source image.
				cv::distanceTransform(skelIn, skelDist, CV_DIST_L2, 3);

				cv::morphologyEx(skelDist, skelDist, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), 3);
				// Erosion to eliminate blur on the threshold image
				//cv:erode(skelDist, skelDist, cv::Mat(), cv::Point(-1, -1), 2);
				//	mDisplay = skelDist.clone();
				// applies high filter laplacian operator to the input image (skelDist)
				cv::Laplacian(skelDist, skelLap, CV_32F, 5);
				double minv, maxv;
				// Finds the global minimum and maximum in the matrix
				cv::minMaxIdx(skelLap, &minv, &maxv);
				// rescale the grayscale value based on the min and max obtained in minMaxIdx
				cv::convertScaleAbs(skelLap, skelConv, 255 / (maxv - minv), minv);
				cv::threshold(skelConv, skelOut, 150, 225, CV_THRESH_BINARY);
				//mDisplay = skelOut.clone();

				// define skelOutPtr, the pointer to skelOut
				for (int i = 0; i < skelOut.rows; i++)
				{
					// ptr - pointer to 8-bit unsigned elements ; step - full row length in bytes
					unsigned char *skelOutPtr = (unsigned char*)(skelOut.ptr() + i*skelOut.step);
					for (int j = 0; j < skelOut.cols; j++)
					{
						if (*skelOutPtr++)
							// add points in mapPts that correspond to the rows and cols of skelOut and limited by skelOutPtr value, I think ?
							mapPts.push_back(cv::Point(j, i));
					}
				}
				cv::Mat newOccMap;
				cv::GaussianBlur(skelOut, newOccMap, cv::Size(5, 5), 3);

				//	mDisplay = newOccMap.clone();
			}
			/*
			else if (mapMode == 2)
			{
				for (int i = 0; i < dispOccMap.rows; i++)
				{
					// ptr - pointer to 8-bit unsigned elements ; step - full row length in bytes
					unsigned char *dispOccMapPtr = (unsigned char*)(dispOccMap.ptr() + i*dispOccMap.step);
					for (int j = 0; j < dispOccMap.cols; j++)
					{
						if (*dispOccMapPtr++ > 50)
							mapPts.push_back(cv::Point(j, i));
					}
				}
			}*/
			// without preprocessing, to generate points to display to the output image
			else
			{
				for (int i = 0; i < dispOccMap.rows; i++)
				{
					// ptr - pointer to 8-bit unsigned elements ; step - full row length in bytes
					unsigned char *dispOccMapPtr = (unsigned char*)(dispOccMap.ptr() + i*dispOccMap.step);
					for (int j = 0; j < dispOccMap.cols; j++)
					{
						if (*dispOccMapPtr++ > 200)
							mapPts.push_back(cv::Point(j, i));
					}
				}
			}

			// without preporcessing, we just apply a Gaussian blur on the newOccMap that is used by the registration algorithm.
			// There is no Gaussian blur applyed on the occMap.
			cv::Mat newOccMap;
			if (mapMode == 2)
			{
				// Gaussian blur after update, outside the loop
				
				cv::GaussianBlur(dispOccMap, newOccMap, cv::Size(3, 3), 1);
				mGaussianOccMap = newOccMap;

#if 0
				mapPts.clear();
				for (int i = 0; i < newOccMap.rows; i++)
				{
					// ptr - pointer to 8-bit unsigned elements ; step - full row length in bytes
					unsigned char *dispOccMapPtr = (unsigned char*)(newOccMap.ptr() + i*newOccMap.step);
					for (int j = 0; j < newOccMap.cols; j++)
					{
						if (*dispOccMapPtr++ > 100)
							mapPts.push_back(cv::Point(j, i));
					}
				}
#endif

				//cv::imshow("newOccMap", newOccMap);
				//cv::waitKey(1);

				// mDisplay = newOccMap.clone();
			}
			// without preprocessing, we still apply a Gaussian blur on the occMap
			else if (mapMode == 3)
				cv::GaussianBlur(dispOccMap, dispOccMap, cv::Size(5, 5), 3);
			
			


			// Eliminate fringing effects around light sources
			cv::Mat tooBrightThresh, tooBrightDist;
			// Changed by Daniel, threshold according to the saturation channel
			cv::threshold(channels2[1], tooBrightThresh, 0.1 * 255, 255, CV_THRESH_BINARY);
			//	cv::threshold(channels[2], tooBrightThresh, 245, 255, CV_THRESH_BINARY_INV);
			//	cv::distanceTransform(tooBrightThresh, tooBrightDist, CV_DIST_L2, 3);
			//	mDisplay = tooBrightThresh;
			//	cvWaitKey(1);
			std::vector<cv::Point> pts;
			//	float brightThresh = 15;
			veinMap.setTo(0);
			mOutMutex.enter();
			mVeinPts.clear();
			for (int i = 0; i < veinPts.size(); i++)
			{
				//mVeinPts.push_back(veinPts[i]);
				//	float dist = tooBrightDist.at<float>(veinPts[i]);
				//printf("    %f\n", dist);
				//		
				
				//if (dist < brightThresh)
				//if (!tooBrightThresh.at<unsigned char>(veinPts[i]))
				if (0)
				{
					veinMap.at<unsigned char>(veinPts[i]) = 0;
					cv::circle(veinOut, veinPts[i], 3, CV_RGB(255, 0, 0), 1);
#if STAND_ALONE_GUI
					cv::circle(veinOut, veinPts[i], 3, CV_RGB(255, 0, 0), 1);
#endif
				}
				else if (veinMap.at<unsigned char>(veinPts[i]))
				{
#if STAND_ALONE_GUI
					cv::circle(veinOut, veinPts[i], 3, CV_RGB(255, 255, 0), 1);
#endif
				}
				else
				{
#if CULL_BAD_COLORS
					unsigned char r = channels[2].at<unsigned char>(veinPts[i]);
					unsigned char g = channels[1].at<unsigned char>(veinPts[i]);
					unsigned char b = channels[0].at<unsigned char>(veinPts[i]);

					unsigned char h = channels2[0].at<unsigned char>(veinPts[i]);
					unsigned char s = channels2[1].at<unsigned char>(veinPts[i]);
					unsigned char v = channels2[2].at<unsigned char>(veinPts[i]);

					//if( (hsv[0] < 30) | (hsv[0] > 150) ) & (hsv[2] >230) & (hsv[1] >20);

					//tune the params
					//if(h < 25 | h > 150)


					//if (r > CULL_R_THRESH &&  r > (float)g*CULL_RG_THRESH)
					//if (r > CULL_R_THRESH && r > g + 5 && r > b + 10)/////////////////////current...
					//if (r > CULL_R_THRESH)
					//if (h < 25 | h > 110)
					if (r > g || r > b + 5)
					//if (r > 80 && (r > g || r > b + 5))
					{
						veinMap.at<unsigned char>(veinPts[i]) = 255;
						//pts.push_back(veinPts[i]);
						mVeinPts.push_back(veinPts[i]);
					}
					else
					{
						cv::circle(veinOut, veinPts[i], 3, CV_RGB(255, 0, 255), 1);
#if STAND_ALONE_GUI
						cv::circle(veinOut, veinPts[i], 3, CV_RGB(255, 0, 255), 1);
#endif
					}

#else
					veinMap.at<unsigned char>(veinPts[i]) = 255;
					pts.push_back(veinPts[i]);
#endif
				}
			}

			veinPts = mVeinPts;
			mOutMutex.leave();

			//mDisplay = channels2[0].clone();
			//mDisplay = veinOut.clone();
			//cvWaitKey(1);


			///	===========================================================	///
			///			Multi resolution correlative scan matching			///
			///	=========================================================== ///

			std::vector<int> idx;
			std::vector<double> dist;

			// Randomly subsample so we don't have too many points
			cv::Mat Rcopy = R.clone(), Tcopy = T.clone();

			std::vector<cv::Point2f> origMapPts;
			for (int i = 0; i < mapPts.size(); i++)
				origMapPts.push_back(mapPts[i]);

			std::vector<cv::Point2f> origVeinPts;
			for (int i = 0; i < veinPts.size(); i++)
				origVeinPts.push_back(veinPts[i]);

			if (1)
			{
				// max point value is limited to 500 samples to enable the program to compute quickly
				//int maxPts = 500;
				int maxPts = 500;
				std::vector<int> sub;
				std::vector<cv::Point> npts;
				/* Debug 1 : no preprocessing
						//pick random int values chosen between 0 and mapPts.size()
						sub = randperm(mapPts.size());
						npts.clear();
						for (int i = 0; i < MIN(mapPts.size(), maxPts); i++)
						{
						// put the corresponding elements (chosen randomly) into npts
						npts.push_back(mapPts[sub[i]]);
						}
						// new map points with 500 points max
						mapPts = npts;
						*/
				sub = randperm(veinPts.size());
				npts.clear();
				for (int i = 0; i < MIN(veinPts.size(), maxPts); i++)
				{
					npts.push_back(veinPts[sub[i]]);
				}
				// new vein points with 500 points max
				veinPts = npts;
			}

			int numInliers = 0;
			int icpStartTime, icpEndTime;

			// save TransMap into a matrix to be used in applyTrans
			cv::Mat TrMap = T.clone();
			TrMap.at<float>(0, 0) = TransMap.x;
			TrMap.at<float>(1, 0) = TransMap.y;

			if (1)
			{
				//	std::vector<cv::Point> &map = mapPts;
				std::vector<cv::Point> &pts = veinPts;

				// create float clones from map and pts
				std::vector<cv::Point2f> mapf;
				std::vector<cv::Point2f> ptsf;

				//	mapf.reserve(map.size());
				ptsf.reserve(pts.size());
				//	for (int i = 0; i < map.size(); i++)
				//		mapf.push_back(map[i]);
				for (int i = 0; i < pts.size(); i++)
					ptsf.push_back(pts[i]);

				

				// translate the vein points into the map basis system
				// rather than using a specific function, the translation is directly added into the apply trans function
			//	TranslationData(ptsf, TransMap.x, TransMap.y); 

				// Transformation parameters
				applyTrans(ptsf, R, T+TrMap);



				// translate the Translation vector
				//Tcopy.at<float>(0, 0) += TransMap.x;
				//Tcopy.at<float>(1, 0) += TransMap.y;

				//mOutMutex.enter();


				//mScanMatching.multiResolution(dispOccMap, ptsf, Tcopy, Rcopy, mFail);

				

				int scanMatchingBegin = timeGetTime();

				// Gaussian blur after update, outside the loop
				if (mapMode == 2)
					mScanMatching.multiResolution(newOccMap, ptsf, Tcopy, Rcopy, Fail);
				// Gaussian blur after update, inside the loop
				else
					mScanMatching.multiResolution(dispOccMap, ptsf, Tcopy, Rcopy, Fail);

				int scanMatchingEnd = timeGetTime();

				// Get the scan matching processing time.
				mScanMatchingAlgo = scanMatchingEnd - scanMatchingBegin;


				//cv::Mat mat = dispOccMap.clone();
				//mScanMatching.lowResolutionMat(mat, mat.cols / 10, mat.rows / 10);

				//cv::imshow("occMap2", occMap);
				//cv::imshow("Low resolution", mat);
				//cvWaitKey(1);
				//mOutMutex.leave();
				// ICP
				//icpStartTime = timeGetTime();
				//icp(mapf, ptsf, Rcopy, Tcopy, numInliers);
				//icpEndTime = timeGetTime();
				//if (veinPts.size() > minInliers && numInliers > minInliers)
				//{
				//Tcopy.at<float>(0, 0) -= TransMap.x;
				//Tcopy.at<float>(1, 0) -= TransMap.y;

				R = Rcopy.clone();
				T = Tcopy.clone();
				//}

			}
			
			/// =======================	///
			///		Kalman Filter		///
			/// ======================= ///
			cv::Mat Rf = R.clone(), Tf = T.clone();
			
			if (RUN_KALMAN_FILTER)
			{
				float theta = atan2(R.at<float>(1, 0), R.at<float>(0, 0));
				float tx = Tf.at<float>(0, 0);
				float ty = Tf.at<float>(1, 0);

				mThetaFilterKF->predict();
				mTxFilterKF->predict();
				mTyFilterKF->predict();

				theta = mThetaFilterKF->correct(theta);
				Tf.at<float>(0, 0) = mTxFilterKF->correct(tx);
				Tf.at<float>(1, 0) = mTyFilterKF->correct(ty);

				Rf.at<float>(0, 0) = cos(theta);
				Rf.at<float>(0, 1) = -sin(theta);
				Rf.at<float>(1, 0) = sin(theta);
				Rf.at<float>(1, 1) = cos(theta);

				R = Rf.clone();
				T = Tf.clone();
			}
			


			/// ===========================	///
			///			Do mapping			///
			/// =========================== ///
			/*
						//warpedVeinPts borders
						int Xmin = 1e6;
						int Ymin = 1e6;
						int Xmax = -1e6;
						int Ymax = -1e6;
						//cv::Mat cloneOccMap=occMap.clone();

						// find the new occMap size
						for (int i = 0; i < warpedVeinPts.size(); i++)
						{
						if (warpedVeinPts.at(i).x < Xmin)
						Xmin = warpedVeinPts.at(i).x;
						if (warpedVeinPts.at(i).x > Xmax)
						Xmax = warpedVeinPts.at(i).x;

						if (warpedVeinPts.at(i).y < Ymin)
						Ymin = warpedVeinPts.at(i).y;
						if (warpedVeinPts.at(i).y > Ymax)
						Ymax = warpedVeinPts.at(i).y;
						}
						*/

			//find image borders : used to resize the occMap
			int Xmin;
			int Ymin;
			int Xmax;
			int Ymax;
			// margin in the detection algorithm
			int margin = 25 - 5;

			std::vector<cv::Point2f> borderPts(4);
			// (0,0)
			borderPts.at(0) = cv::Point2f(margin, margin);
			// (0,max)
			borderPts.at(1) = cv::Point2f(margin, gimg.rows - margin);
			// (max,max)
			borderPts.at(2) = cv::Point2f(gimg.cols - margin, gimg.rows - margin);
			// (max,0)
			borderPts.at(3) = cv::Point2f(gimg.cols - margin, margin);


			// apply translation to center the data into the map
			//TranslationData(borderPts, TransMap.x, TransMap.y);
			// apply transformation model
			applyTrans(borderPts, R, T + TrMap);

			// Find min and max
			FindMinMax(borderPts, Xmin, Xmax, Ymin, Ymax);


			// Resize the occMap
			int XminOccMap = 0;
			int YminOccMap = 0;
			int XmaxOccMap = occMap.cols;
			int YmaxOccMap = occMap.rows;
			
			if (Xmin < XminOccMap)
				XminOccMap = Xmin;
			if (Xmax > XmaxOccMap)
				XmaxOccMap = Xmax;
			if (Ymin < YminOccMap)
				YminOccMap = Ymin;
			if (Ymax > YmaxOccMap)
				YmaxOccMap = Ymax;
			
			// update the translation vector
		//	TransMap.x -= XminOccMap;
		//	TransMap.y -= YminOccMap;
		//	TrMap.at<float>(0, 0) = TransMap.x;
		//	TrMap.at<float>(1, 0) = TransMap.y;

			cv::Mat cloneOccMap(YmaxOccMap - YminOccMap, XmaxOccMap - XminOccMap, occMap.type());
			cloneOccMap.setTo(0);

			for (int i = 0; i < occMap.rows; i++)
			{
				float* PtrCloneOccMap = cloneOccMap.ptr<float>(i - YminOccMap);
				float* PtrOccMap = occMap.ptr<float>(i);

				for (int j = 0; j < occMap.cols; j++)
					PtrCloneOccMap[j - XminOccMap] = PtrOccMap[j];
			}
			occMap = cloneOccMap.clone();


			// create a copy of veinPts vector
			std::vector<cv::Point2f> warpedVeinPts;
			for (int i = 0; i < origVeinPts.size(); i++)
				warpedVeinPts.push_back(origVeinPts[i]);

			// apply translation to center the data into the map
			//TranslationData(warpedVeinPts, TransMap.x, TransMap.y);
			// Apply transformation
			applyTrans(warpedVeinPts, R, T + TrMap);

		//	cv::Mat gblurMap;
			cv::Mat warpedVeinMap = occMap.clone();
			warpedVeinMap.setTo(0);
			// store veinMapVeinPts into warpedVeinMap
			for (int i = 0; i < warpedVeinPts.size(); i++)
			{
				cv::Point pt = warpedVeinPts[i];

				//eliminate the points located outside the matrix border
				if (pt.x < 0 || pt.y < 0 || pt.x >= warpedVeinMap.cols || pt.y >= warpedVeinMap.rows)
					continue;
				//add the point inside the border as a white pixel.
				//warpedVeinMap.at<unsigned char>(warpedVeinPts[i]) = 255;
				warpedVeinMap.at<float>(warpedVeinPts[i]) = 255;
			}

			// output vessel points
			std::vector<cv::Point2f> warpedMapPts;
			for (int i = 0; i < origMapPts.size(); i++)
				warpedMapPts.push_back(origMapPts[i]);

			

			applyTrans(warpedMapPts, Rf.t(), -Rf.t()*(Tf+TrMap));
			//TranslationData(warpedMapPts, -TransMap.x, -TransMap.y);

			mOutMutex.enter();
			mWarpedMapPts = warpedMapPts;

			//added by sungwook...Starts..
#if 0
			Tr2dX = Tr2dC.clone();
			Tr2dC = cv::Mat::eye(3,3,CV_32F);
			Tr2dC(cv::Range(0,2), cv::Range(0,2)) = Rf.t();
			Tr2dC(cv::Range(0,2), cv::Range(2,3)) = -Rf.t()*(2*Tf);
			TR3d = estimate3DTransform();

			//_PrintMat(TR3d);
#endif
			//added by sungwook...Ends..


			mRf = Rf.clone();
			mTf = Tf.clone();


			// store mRf and mTf in vectors 
			mVecRf.push_back(mRf);
			mVecTf.push_back(mTf);

			bFlagTR = true;


			mOutMutex.leave();

			cv::Mat finVein = img.clone();

#if STAND_ALONE_GUI
			for (int i = 0; i < warpedMapPts.size(); i++)
			{
				cv::Point pt = warpedMapPts[i];
				if (pt.x < 0 || pt.y < 0 || pt.x >= veinOut.cols || pt.y >= veinOut.rows)
					continue;
				cv::circle(finVein, pt, 2, CV_RGB(0, 255, 0), -1);
			}
#endif
			// we are applying Gaussian blur on the vein pts for the mode with preprocessing and the mode without but with Gaussian blur on the vein points
			if (mapMode == 1 || mapMode == 4)
				cv::GaussianBlur(warpedVeinMap, warpedVeinMap, cv::Size(9, 9), 2);
			//cv::GaussianBlur(warpedVeinMap, warpedVeinMap, cv::Size(5, 5), 3);
			//cv::GaussianBlur(warpedVeinMap, warpedVeinMap, cv::Size(3, 3), 3);

			//Get the boundaries of the area to update
			cv::Mat subOccMapBound;
			GetSubOccMapBoundaries(gimg, occMap, subOccMapBound, R, T + TrMap, 25);
			//if (subOccMapBound.rows && subOccMapBound.cols)
			//mDisplay = subOccMapBound;

			int i0 = Ymin;
			int j0 = Xmin;
			if (i0 < 0)
				i0 = 0;
			if (j0 < 0)
				j0 = 0;
			
			/*
			if (mapMode == 2)
			{
				double add_value = 100;
				double sub_value = 2;
				double max_value = 200;

				for (int i = i0; i < Ymax; i++)
				{
					for (int j = j0; j < Xmax; j++)
					{
						if (subOccMapBound.at<float>(i, j))
						{
							occMap.at<float>(i, j) += warpedVeinMap.at<float>(i, j)*add_value;
							if (occMap.at<float>(i, j) > max_value)
								occMap.at<float>(i, j) = max_value;
							if (occMap.at<float>(i, j) > sub_value)
								occMap.at<float>(i, j) -= sub_value;
						}
					}
				}
			}
			else
			{*/
				for (int i = i0; i < Ymax; i++)
				{
					for (int j = j0; j < Xmax; j++)
					{
						if (subOccMapBound.at<float>(i, j))
						{
							double value = warpedVeinMap.at<float>(i, j)*blurScale;
							occMap.at<float>(i, j) += value;
							if (value == 0)
							{
								if (occMap.at<float>(i, j) > occDecay)
									occMap.at<float>(i, j) -= occDecay;
							}
							if (occMap.at<float>(i, j) > occMax)
								occMap.at<float>(i, j) = occMax;
						}
					}
				}
			//}
			//ouput display
			//mDisplay = subOccMapBound.clone();

			//if (mapMode == 2)
			//	dispOccMap = occMap.clone();
			//else
				cv::convertScaleAbs(occMap, dispOccMap, 1.0/5.0);


			// to display the updated area
			/*
			if (Ymax-Ymin > 0 && Xmax - Xmin > 0)
			{
				cv::Mat subOccMap(Ymax - Ymin, Xmax - Xmin, occMap.type());
				for (int i = 0; i < subOccMap.rows; i++)
				{
					float* PtrOccMap = dispOccMap.ptr<float>(i + Ymin);
					float* PtrSubOccMap = subOccMap.ptr<float>(i);

					for (int j = 0; j < subOccMap.cols; j++)
						PtrSubOccMap[j] = PtrOccMap[j + Xmin];
				}
				//mDisplay = subOccMap.clone();
			}
			*/
			mOutMutex.enter();
			mOccMap = dispOccMap.clone();
			mOutMutex.leave();

			int endTime = timeGetTime();
			totTime += endTime - startTime;
			mEyeSLAMTime = endTime - startTime;
			ctr++;

			// We are updating the translation of the map
			TransMap.x -= XminOccMap;
			TransMap.y -= YminOccMap;
			TrMap.at<float>(0, 0) = TransMap.x;
			TrMap.at<float>(1, 0) = TransMap.y;

#if STAND_ALONE_GUI
			printf("%0.1f %d (%d trace, %d icp) ms    %d %d   %d\n", (double)totTime/ctr, endTime - startTime, traceEndTime - traceStartTime, icpEndTime - icpStartTime, mapPts.size(), veinPts.size(), numInliers);

			//for (int i = 0; i < veinPts.size(); i++)
			//{
			//	occMap.at<float>(veinPts[i]) += 
			//}
			//} 

			//cv::imshow("occMap", dispOccMap);
			//cv::imshow("skelOut", skelOut);
			//imagesc<float>("skelDist", skelConv);
			//
			cv::imshow("img", img);
			cv::imshow("bimg", tooBrightThresh);
			cv::imshow("occmap", dispOccMap);
			//cv::imshow("vein", veinMap);
			//cv::imshow("map", skelOut);
			//cv::imshow("subvein", gblurMap);
			//cv::imshow("submap", subMap);
			cv::imshow("markedvein", veinOut);
			cv::imshow("finVein", finVein);

			if (first)
			{
				cv::waitKey(-1);
			}
			else
			{
				if (cv::waitKey(30) == 'k')
					mFirst = 1;
			}
			
			if (endTime - startTime > 500)
				Sleep(1);

#endif
			//Find 3d transform..
			
		}

		//Sleep(1)
	}
	bFlagTR = false;
	

	
}

int EyeSLAM::absor(const cv::Mat &map, const cv::Mat &pts, cv::Mat &R, cv::Mat &T)
{
	double sumts = 1;

	return 0;	
}

int EyeSLAM::absor(const std::vector<cv::Point2f> &map, const std::vector<cv::Point2f> &pts, cv::Mat &R, cv::Mat &T)
{
	double sumwts = 1;

	std::vector<cv::Point2f> left;
	std::vector<cv::Point2f> right;

	left.reserve(map.size());
	right.reserve(pts.size());
	for (int i = 0; i < map.size(); i++)
		left.push_back(map[i]);
	for (int i = 0; i < pts.size(); i++)
		right.push_back(pts[i]);

	cv::Point2f lc = calcMeanPt(left);
	cv::Point2f rc = calcMeanPt(right);

	subPt(left, lc);
	subPt(right, rc);

	cv::Mat left32f, right32f;
	//cv::Mat(left).convertTo(left32f, CV_32F);
	//cv::Mat(right).convertTo(right32f, CV_32F);
	//cv::Mat l = cv::Mat(left);
	vecOfPtsToMat(left, left32f);
	vecOfPtsToMat(right, right32f);
	cv::Mat M = left32f*right32f.t();

	double Nxx = M.at<float>(0,0) + M.at<float>(1,1);
	double Nyx = M.at<float>(0,1) - M.at<float>(1,0);

	cv::Mat N(2, 2, CV_32F);
	N.at<float>(0,0) = Nxx;
	N.at<float>(0,1) = Nyx;
	N.at<float>(1,0) = Nyx;
	N.at<float>(1,1) = -Nxx;

	cv::Mat V, D;
	cv::eigen(N, 1, D, V);
	cv::Mat q;
	q.create(1, 2, CV_32F);
	q.at<float>(0,0) = V.at<float>(0,0);
	q.at<float>(0,1) = V.at<float>(0,1);

	float *qd = (float*)q.data;
	float q1 = qd[1] + (qd[1] >= 0);
	float s = 1;
	if (q1 < 0)
		s = -1;

	q = q*s;
	q = q / cv::norm(q);

	float R11 = qd[0]*qd[0] - qd[1]*qd[1];
	float R21 = qd[0]*qd[1]*2;

	if (R.rows != 2 || R.cols != 2 || R.type() != CV_32F)
	{
		R.create(2, 2, CV_32F);
	}
	R.at<float>(0,0) = R11;
	R.at<float>(0,1) = -R21;
	R.at<float>(1,0) = R21;
	R.at<float>(1,1) = R11;

	T = cv::Mat(rc) - R*cv::Mat(lc);

	return 0;
}


void EyeSLAM::subPt(std::vector<cv::Point2f> &pts, cv::Point2f pt)
{
	for (int i = 0; i < pts.size(); i++)
		pts[i] -= pt;
}

cv::Point2f EyeSLAM::calcMeanPt(const std::vector<cv::Point2f> &pts)
{
	cv::Point2f meanPt;

	for (int i = 0; i < pts.size(); i++)
	{
		meanPt += pts[i];
	}

	meanPt = meanPt * (1.0 / pts.size());

	return meanPt;
}

int EyeSLAM::icp(const cv::Mat &map, const cv::Mat &pts, cv::Mat &R, cv::Mat &T)
{
	int deadMan = 40;

	for (int k = 0; k < deadMan; k++)
	{

	}

	return 0;
}

int EyeSLAM::icp(const std::vector<cv::Point2f> &map, const std::vector<cv::Point2f> &pts, cv::Mat &R, cv::Mat &T, int &numInliers)
{
	int deadMan = 40;
	float distThreshold = 25;
	distThreshold *= distThreshold;
	int matchesThreshold = 50;

	//cv::Mat Rg, Tg;
	//Rg.create(2, 2, CV_32F);
	//Tg.create(2, 1, CV_32F);
	//Rg.setTo(0);
	//Tg.setTo(0);
	//Rg.at<float>(0,0) = 1;
	//Rg.at<float>(1,1) = 1;
	//Rg.at<float>(0,0) = 0.999613495205402;
	//Rg.at<float>(0,1) = -0.027800363365255;
	//Rg.at<float>(1,0) = 0.027800363365255;
	//Rg.at<float>(1,1) = 0.999613495205402;
	//Tg.at<float>(0,0) = -37.260455393191464;
	//Tg.at<float>(0,1) = -27.162027777694838;

/*	if (0)
	{
		cv::Point pt;
		std::vector<cv::Point2f> map, pts;
		FILE *fp = 0;

		float x, y;
		fp = fopen("E:\\VirtualFixtures\\pigeyes\\Lobes_02-26-2012\\first cannulation\\mapPts.txt", "rb");
		while (fscanf(fp, "%f %f\n", &x, &y) == 2)
			map.push_back(cv::Point2f(x, y));
		fclose(fp);

		fp = fopen("E:\\VirtualFixtures\\pigeyes\\Lobes_02-26-2012\\first cannulation\\sensPts.txt", "rb");
		while (fscanf(fp, "%f %f\n", &x, &y) == 2)
			pts.push_back(cv::Point2f(x, y));
		fclose(fp);

		absor(map, pts, R, T);
	}*/

	cv::vector<cv::Point2f> warpedPts = pts;

	cv::Mat Rt(2, 2, CV_32F);
	cv::Mat Tt(2, 1, CV_32F);
	Rt.setTo(0);
	Rt.at<float>(0,0) = 1;
	Rt.at<float>(1,1) = 1;
	Tt.setTo(0);

	// associate Rt and Tt with the last transformation value.
	if (Rt.rows == R.rows && Rt.cols == R.cols && Rt.type() == R.type())
		Rt = R.clone();
	if (Tt.rows == T.rows && Tt.cols == T.cols && Tt.type() == T.type())
		Tt = T.clone();

	// apply the last transformation model
	applyTrans(warpedPts, Rt, Tt);

	float curErr = -1, lastErr = -1;
	int ctr = 0;
	numInliers = 0;

	cv::Mat Rf, tf;
	for (int k = 0; k < deadMan; k++, ctr++)
	{
		lastErr = curErr;
		curErr = 0;

		std::vector<cv::Point2f> mapMatchPts, ptsMatchPts;

		// measure the distance between the warped point and the map point
		// try to find matches
		for (int i = 0; i < warpedPts.size(); i++)
		{
			float minDist = 1e9; // ~inf
			int minIdx = -1;

			// find the closest map point to each warped point
			for (int j = 0; j < map.size(); j++)
			{
				int dx = warpedPts[i].x - map[j].x;
				int dy = warpedPts[i].y - map[j].y;
				int d = dx*dx + dy*dy;
				if (d < minDist)
				{
					minDist = d;
					minIdx = j;
				}
			}

			// if the two points are close enough, they match
			if (minDist < distThreshold)
			{
				// memorize the points
				mapMatchPts.push_back(map[minIdx]);
				ptsMatchPts.push_back(warpedPts[i]);

				float dx = map[minIdx].x - warpedPts[i].x;
				float dy = map[minIdx].y - warpedPts[i].y;
				curErr += sqrt(dx*dx + dy*dy);
			}
		}

		if (mapMatchPts.size())
		{
			numInliers = mapMatchPts.size();
			curErr /= numInliers;
			//printf("%d %d  %f\n", ctr, numInliers, curErr);
		}

		// if the difference of error is under a threshold value, there is no need to recalculate the transformation matrix
		if (curErr >= 0 && lastErr >= 0 && fabs(curErr - lastErr) < 1e-2)
			break;

		// if it has enough points, it calculate the new transformation.
		if (mapMatchPts.size() > matchesThreshold)
		{
			cv::Mat r, t;
			absor(ptsMatchPts, mapMatchPts, r, t);
			Rt = Rt*r;
			Tt = Tt + t;
			// if the transformation succeed, we reset the failure counter
			mICPfail = 0;
		}
		// if there is not enough points to register the transformation, we increase the failure counter
		else
		{
			printf("could not icp, not enough points for good solution....\n");
			mICPfail++;
			break;
		}

		warpedPts = pts;
		applyTrans(warpedPts, Rt, Tt);
	}

	R = Rt.clone();
	T = Tt.clone();

	iteration++;

	//FILE *fp = 0;
	//fp = fopen(String().sprintf("out/map%0.4d.txt", iteration), "wb");
	//for (int i = 0; i < map.size(); i++)
	//	fprintf(fp, "%f %f\n", map[i].x, map[i].y);
	//fclose(fp);
	//fp = fopen(String().sprintf("out/pts%0.4d.txt", iteration), "wb");
	//for (int i = 0; i < pts.size(); i++)
	//	fprintf(fp, "%f %f\n", pts[i].x, pts[i].y);
	//fclose(fp);
	//fp = fopen(String().sprintf("out/warpedpts%0.4d.txt", iteration), "wb");
	//for (int i = 0; i < warpedPts.size(); i++)
	//	fprintf(fp, "%f %f\n", warpedPts[i].x, warpedPts[i].y);
	//fclose(fp);

	//fp = fopen(String().sprintf("out/transfirst%0.4d.txt", iteration), "wb");
	//fprintf(fp, "%f %f %f\n", Rt.at<float>(0,0), Rt.at<float>(0,1), Tt.at<float>(0,0));
	//fprintf(fp, "%f %f %f\n", Rt.at<float>(1,0), Rt.at<float>(1,1), Tt.at<float>(1,0));
	//fclose(fp);


	return 0;
}

void EyeSLAM::applyTrans(std::vector<cv::Point2f> &pts, const cv::Mat &R, const cv::Mat &T)
{
	float R00 = R.at<float>(0,0);
	float R01 = R.at<float>(0,1);
	float R10 = R.at<float>(1,0);
	float R11 = R.at<float>(1,1);
	float Tx = T.at<float>(0, 0);
	float Ty = T.at<float>(1, 0);
	//float *Tptr = (float*)T.data;
	//float Tx = Tptr[0];
	//float Ty = Tptr[1];
	for (int i = 0; i < pts.size(); i++)
	{
		float x = pts[i].x*R00 + pts[i].y*R01 + Tx;
		float y = pts[i].x*R10 + pts[i].y*R11 + Ty;
		pts[i].x = x;
		pts[i].y = y;
	}
}

// added By Daniel : export results on .csv file
void EyeSLAM::OutputTransformFile(std::string FileName)
{
	// cut the path
	std::string::size_type PATH = FileName.find_last_of('\\');
	std::string name = FileName.substr(PATH);

	// cut ext
	std::string::size_type EXT = name.find_last_of('.');
	name = name.substr(0, EXT);

	std::ofstream myfile;
	myfile.open(RESULTS_TRANSFORM_PATH + name + ".csv");

	myfile << "video path" << "," << FileName << "\n";
	
	myfile << "Frame,tx,ty,cos(theta),sin(theta)\n";
	for (int i = 0; i < mVecRf.size(); i++)
		myfile << i+1 << "," << mVecTf.at(i).at<float>(0, 0) << "," << mVecTf.at(i).at<float>(1, 0) << "," << mVecRf.at(i).at<float>(0, 0) << "," << mVecRf.at(i).at<float>(1, 0) << "\n";

	myfile.close();
}

int EyeSLAM::knnNaiveCPU(const std::vector<cv::Point> &map, const std::vector<cv::Point> pts, std::vector<int> &idx, std::vector<double> &dist)
{
	idx.clear(); 
	dist.clear();
	idx.reserve(pts.size()); 
	dist.reserve(pts.size());
	for (int i = 0; i < pts.size(); i++)
	{
		int minDist = INT_MAX;
		int minIdx = -1;
		for (int j = 0; j < map.size(); j++)
		{
			int dx = pts[i].x - map[j].x;
			int dy = pts[i].y - map[j].y;
			int d = dx*dx + dy*dy;
			if (d < minDist)
			{
				minDist = d;
				minIdx = j;
			}
		}

		dist.push_back(minDist);
		idx.push_back(minIdx);
	}

	return 0;
}

//int EyeSLAM::knnBruteGPU(const cv::Mat &map, const cv::Mat &pts, std::vector<int> &idx, std::vector<double> &dist)
//{
//	int processFlag = 0;
//	cv::Mat left, right, disp;
//	cv::gpu::GpuMat gpuDisp;
//	cv::gpu::GpuMat gpuLeftWarped, gpuLeftSmall, gpuLeftSmallPadded, gpuRightSmall;
//
//	cv::gpu::GpuMat gpuTipLeftSmall, gpuTipRightSmall, gpuTipLeftSmallBin, gpuTipRightSmallBin, gpuTipLeftSmallBinDilate, gpuTipRightSmallBinDilate;
//	cv::gpu::GpuMat gpuTipLeftBin, gpuTipRightBin, gpuTipLeftBinDilate, gpuTipRightBinDilate;
//
//	cv::gpu::GpuMat mGpuLeft, mGpuRight, gpuGrayLeft, gpuGrayRight, gpuG, gpuR, gpuCombined;
//	cv::gpu::StereoBM_GPU bm;
//
//	cv::Mat prevGray;
//	cv::gpu::GpuMat gpuPrevGray;
//
//	bm.preset = cv::gpu::StereoBM_GPU::PREFILTER_XSOBEL;
//	//bm.preset = cv::gpu::StereoBM_GPU::BASIC_PRESET;
//	bm.ndisp = 32+8;
//	bm.winSize = 51;
//
//
//	cv::gpu::SURF_GPU surf;
//
//	// detecting keypoints & computing descriptors
//	cv::gpu::GpuMat keypoints1GPU, keypoints2GPU;
//	cv::gpu::GpuMat descriptors1GPU, descriptors2GPU;
//
//	// downloading results
//	std::vector<cv::KeyPoint> keypoints1, keypoints2;
//	std::vector<float> descriptors1, descriptors2;
//
//	//hl = hl.inv();
//
//	std::vector<cv::gpu::GpuMat> channels;
//
//
//	cv::Mat gBin, small, gray;
//
//	cv::gpu::PyrLKOpticalFlow gpuLK;
//	cv::gpu::GoodFeaturesToTrackDetector_GPU gpuGoodFeatures(500, 0.01/2, 0.0);
//
//	cv::gpu::GpuMat gpuPrevPts;
//	cv::gpu::GpuMat gpuNextPts;
//	cv::gpu::GpuMat gpuStatus;
//
//	std::vector<cv::Point2f> prevPts;
//	std::vector<cv::Point2f> nextPts;
//	std::vector<unsigned char> status;
//	cv::gpu::GpuMat d_vertex, d_colors;
//
//	cv::gpu::GpuMat gpuColorSmall, gpuPrevColorSmall;
//
//
//	cv::gpu::BruteForceMatcher_GPU< cv::L2<float> > matcher;
//	cv::gpu::GpuMat trainIdx, distance;
//
//	descriptors1GPU.upload(map);
//	descriptors2GPU.upload(pts);
//
//	std::vector<cv::DMatch> matches;
//
//	matcher.matchSingle(descriptors1GPU, descriptors2GPU, trainIdx, distance);
//	cv::gpu::BruteForceMatcher_GPU< cv::L2<float> >::matchDownload(trainIdx, distance, matches);
//
//	return 0;
//}

void EyeSLAM::slamIt(const cv::Mat &img)
{
	mMutex.enter();
	mInput.enqueue(img.clone());
	mMutex.leave();
}
void EyeSLAM::stop(void)
{
	Thread::stopThread(10);
	mSkipped = 0;
	iteration = 0;
	mFirst = 1;
	mTxFilterKF = 0;
	mTyFilterKF = 0;
	mThetaFilterKF = 0;
	mIsDry = false;
	mWarpedMapPts.clear();
	mVeinPts.clear();
	Tr2dC = cv::Mat::eye(3,3,CV_32F);

}
void EyeSLAM::vecOfPtsToMat(const std::vector<cv::Point2f> &pts, cv::Mat &mat)
{
	if (mat.rows != 2 || mat.cols != pts.size() || mat.type() != CV_32F)
	{
		mat.create(2, pts.size(), CV_32F);
	}

	float *ptr = (float*)mat.data;
	for (int i = 0; i < pts.size(); i++)
		*ptr++ = pts[i].x;
	for (int i = 0; i < pts.size(); i++)
		*ptr++ = pts[i].y;
}

cv::Mat EyeSLAM::estimate3DTransform(void)
{
	cv::Mat TR = cv::Mat::eye(4, 4, CV_64FC1);

	int time = timeGetTime();
	std::vector<cv::Point3f> slam3dPts;
	//std::vector<cv::Point2f> slam2dLeftAll;
	std::vector<cv::Point2f> slam2dLeft;
	std::vector<cv::Point2f> slam2dRight;


	std::vector<cv::Point2f> warpedPts;
	getVeinMap(warpedPts); 

	cv::Mat slamPts = 2*cv::Mat(warpedPts).reshape(1); 
	slam2dLeft = cv::Mat_<cv::Point2f>(slamPts);

	if(slam2dLeft.size()<3)
		return	TR;

	int nMaxPts = 200;

	slam2dLeft = m_geom.randSampling(slam2dLeft, nMaxPts);



	cv::perspectiveTransform(slam2dLeft, slam2dRight, H);
	
	/*cv::Mat pt3dMat;
	cv::Mat Ml(m_pMl);  cv::Mat Mr(m_pMr); */
	//_PrintResult("t0 : %d ms", timeGetTime() - time);

	//cv::triangulatePoints(Ml.t(), Mr.t(), cv::Mat(slam2dLeft).reshape(1,2), cv::Mat(slam2dRight).reshape(1,2), pt3dMat);


	//_PrintResult("t1 : %d ms", timeGetTime() - time);
	for (int i =0 ; i < slam2dLeft.size() ; i++)
		slam3dPts.push_back(m_geom.reproject(m_pMl, slam2dLeft[i], m_pMr, slam2dRight[i]));

	//_PrintResult("t2 : %d ms", timeGetTime() - time);
	/////////////////////////////////////////////////////////////////////////


	cv::Mat localTR = Tr2dC*Tr2dX.inv();


	std::vector<cv::Point2f> prevPt2dLeft; 
	cv::perspectiveTransform(slam2dLeft, prevPt2dLeft, localTR.inv());
	//std::vector<cv::Point2f> prevPt2dLeft = m_geom.apply2dTransform(localTR.inv(), slam2dLeft);



	std::vector<cv::Point2f> prevPt2dRight;

	cv::perspectiveTransform(prevPt2dLeft, prevPt2dRight, H);

	std::vector<cv::Point3f> prevPt3d;

	//_PrintResult("H : %d ms", timeGetTime() - time);
	for (int i =0 ; i < prevPt2dLeft.size() ; i++)
		prevPt3d.push_back(m_geom.reproject(m_pMl, prevPt2dLeft[i], m_pMr, prevPt2dRight[i]));

	//_PrintResult("Estimate 3d : %d ms", timeGetTime() - time);
	TR = m_geom.estimate3dTransform(prevPt3d, slam3dPts);
	//_PrintMat(TR);

	//_PrintResult("All : %d ms", timeGetTime() - time);
	TR = TR*TR3d;
	return TR;
}


//void EyeSLAM::getVeinMapfromH(std::vector<cv::Point2f> &map)
//{
//	 mOutMutex.enter();
//	 size_t nVeinPts = mWarpedMapPts.size();
//
//	 if(nVeinPts <=0)
//		 return;
//
//	 
//	 //get right vein maps form left using homography
//	 bool bGetVeinMapfromH = false;
//	 bool bGet3DVeinMap = false;
//
//	/* cv::Mat H;
//	 m_surfModeler.m_planeInfo.H.convertTo(H, CV_32F);*/
//	 if(H.cols ==3 && H.rows ==3)
//		 bGetVeinMapfromH = true;
//
//	/* _PrintResult("EYE SLAM Update\n");
//	 _PrintMat(H);*/
//
//	 if(bGetVeinMapfromH)
//	 {
//
//		 //std::vector<cv::Point2f> slam2dRight;
//		 //cv::perspectiveTransform(slam2dLeft, slam2dRight, H);
//		 
//
//
//		 cv::perspectiveTransform(mWarpedMapPts, map, H); 
//
//
//		 if(bGet3DVeinMap)
//		 {
//			 std::vector<cv::Point3f> slam3dPts;
//			 std::vector<cv::Point2f> slam2dPtsProj;
//
//			 for (int i =0 ; i < mWarpedMapPts.size() ; i++)
//			 {
//				 slam3dPts.push_back(m_geom.reproject(m_pMl, mWarpedMapPts[i], m_pMr, map[i]));
//				 slam2dPtsProj.push_back( m_geom.project(slam3dPts[i], m_pMr));
//			 }
//
//		 }
//		
//		
//	 }
//
//	 
//	 mOutMutex.leave();
//}

void EyeSLAM::getVeinMapfromH(std::vector<cv::Point2f> &map)
{
	 mOutMutex.enter();
	 size_t nVeinPts = mWarpedMapPts.size();

	 if(nVeinPts <=0)
		 return;

	 
	 //get right vein maps form left using homography
	 bool bGetVeinMapfromH = false;
	 bool bGet3DVeinMap = false;

	/* cv::Mat H;
	 m_surfModeler.m_planeInfo.H.convertTo(H, CV_32F);*/
	 if(H.cols ==3 && H.rows ==3)
		 bGetVeinMapfromH = true;

	/* _PrintResult("EYE SLAM Update\n");
	 _PrintMat(H);*/

	 if(bGetVeinMapfromH)
	 {

		 //std::vector<cv::Point2f> slam2dRight;
		 //cv::perspectiveTransform(slam2dLeft, slam2dRight, H);
		 

		 cv::Mat slamPts = 2*cv::Mat(mWarpedMapPts).reshape(1); 
		 std::vector<cv::Point2f> slam2dLeft = cv::Mat_<cv::Point2f>(slamPts);

		 //cv::perspectiveTransform(mWarpedMapPts, map, H); 
		 cv::perspectiveTransform(slam2dLeft, map, H); 

		 if(bGet3DVeinMap)
		 {
			 std::vector<cv::Point3f> slam3dPts;
			 std::vector<cv::Point2f> slam2dPtsProj;

			 for (int i =0 ; i < slam2dLeft.size() ; i++)
			 {
				 slam3dPts.push_back(m_geom.reproject(m_pMl, slam2dLeft[i], m_pMr, map[i]));
				 slam2dPtsProj.push_back( m_geom.project(slam3dPts[i], m_pMr));
			 }

		 }
		
		
	 }

	 
	 mOutMutex.leave();
}

// Added by Daniel : to translate a vector of point in the 2D plan
void EyeSLAM::TranslationData(std::vector<cv::Point2f> &pts, int Tx, int Ty)
{
	for (int i = 0; i < pts.size(); i++)
		pts.at(i) = cv::Point2f(pts.at(i).x + Tx, pts.at(i).y + Ty);
}

void EyeSLAM::TranslationData(std::vector<cv::Point> &pts, int Tx, int Ty)
{
	for (int i = 0; i < pts.size(); i++)
		pts.at(i) = cv::Point(pts.at(i).x + Tx, pts.at(i).y + Ty);
}

// Added by Daniel : get the boundaries matrix to only update the visible area in the map
void EyeSLAM::GetSubOccMapBoundaries(cv::Mat src, cv::Mat Map, cv::Mat &subOccMapBound, cv::Mat R, cv::Mat T, int margin)
{
	// Get the 4 border points of the image
	std::vector<cv::Point2f> borderPts(4);
	// (0,0)
	borderPts.at(0) = cv::Point2f(margin, margin);
	// (0,max)
	borderPts.at(1) = cv::Point2f(margin, src.rows - margin);
	// (max,max)
	borderPts.at(2) = cv::Point2f(src.cols - margin, src.rows - margin);
	// (max,0)
	borderPts.at(3) = cv::Point2f(src.cols - margin, margin);

	//Apply transformation
	applyTrans(borderPts, R, T);
	//TranslationData(borderPts, Trans.x, Trans.y);

	subOccMapBound = Map.clone();
	subOccMapBound.setTo(0);

	//fillConvexPoly require cv::Point, does not handle std::vector<cv::Point>
	cv::Point Pts[4];
	for (int i = 0; i < borderPts.size(); i++)
		Pts[i] = borderPts.at(i);

	if (subOccMapBound.rows*subOccMapBound.cols)
		cv::fillConvexPoly(subOccMapBound, Pts, 4, 255);

	mBorderPts = borderPts;
}

void EyeSLAM::FindMinMax(std::vector<cv::Point2f> Data, int &Xmin, int &Xmax, int &Ymin, int &Ymax)
{
	Xmin =  1e6;
	Ymin =  1e6;
	Xmax = -1e6;
	Ymax = -1e6;

	for (int i = 0; i < Data.size(); i++)
	{
		if (Xmin > Data[i].x)
			Xmin = Data[i].x;
		if (Xmax < Data[i].x)
			Xmax = Data[i].x;
		if (Ymin > Data[i].y)
			Ymin = Data[i].y;
		if (Ymax < Data[i].y)
			Ymax = Data[i].y;
	}
}
