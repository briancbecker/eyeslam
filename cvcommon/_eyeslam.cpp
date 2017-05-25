#include "eyeslam.h"
#include "skeletonize.h"

#define STAND_ALONE_GUI 0
#define CULL_BAD_COLORS 1
#define CULL_R_THRESH 50
#define CULL_RG_THRESH 1.5

Array<cv::Scalar> colors;

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

CEyeSLAM::CEyeSLAM()
{
	mSkipped = 0;
	iteration = 0;
	mFirst = 1;
	mTxFilter = 0;
	mTyFilter = 0;
	mThetaFilter = 0;
	mTxFilterKF = 0;
	mTyFilterKF = 0;
	mThetaFilterKF = 0;
}

CEyeSLAM::~CEyeSLAM()
{
	this->stopThread();

	delete mTxFilter;
	delete mTyFilter;
	delete mThetaFilter;
	delete mTxFilterKF;
	delete mTyFilterKF;
	delete mThetaFilterKF;

	mTxFilter = 0;
	mTyFilter = 0;
	mThetaFilter = 0;
	mTxFilterKF = 0;
	mTyFilterKF = 0;
	mThetaFilterKF = 0;

}

void CEyeSLAM::execute()
{
	cv::Mat veinMap, veinOut;
	std::vector<cv::Point> veinPts;
	cv::Mat inImg;
	cv::Mat occMap;

	float occDecay = 0.01*255;
	float occMax = 5*255;
	float occMin = 0;
	float blurScale = 5;
	int minInliers = 50;
	int skelThresh = 127;

	int ctr = 0;
	int totTime = 0;

	while (!Thread::quitFlag())
	{
		cv::Mat img;
		mMutex.enter();
		int first = 0;
		if (mInput.size())
		{
			mInput.dequeue(img);
			while (mInput.size())
			{
				mSkipped++;
				mInput.dequeue(img);
			}
			first = mFirst;
			mFirst = 0;
		}
		mMutex.leave();

		if (img.rows && img.cols)
		{
			int startTime = timeGetTime();
			std::vector<cv::Mat> channels;

			cv::split(img, channels);
			if (channels.size() <= 1)
				continue;

			cv::Mat gimg = channels[1];

			cv::resize(gimg, inImg, cv::Size(img.cols/2, img.rows/2));

			img.copyTo(veinOut);
			int traceStartTime = timeGetTime();
			mVeinTracer.trace(gimg, veinMap, veinOut, veinPts);
			int traceEndTime = timeGetTime();

			if (occMap.rows != veinMap.rows || occMap.cols != veinMap.cols || first)
			{
				occMap.create(cv::Size(veinMap.cols, veinMap.rows), CV_32F);
				occMap.setTo(0);
				delete mTxFilter;
				delete mTyFilter;
				delete mThetaFilter;
				mTxFilter = new FilterAB("files\linPosFilter2Butter0.1.txt");
				mTyFilter = new FilterAB("files\linPosFilter2Butter0.1.txt");
				mThetaFilter = new FilterAB("files\linPosFilter2Butter0.1.txt");

				delete mTxFilterKF;
				delete mTyFilterKF;
				delete mThetaFilterKF;

				mTxFilterKF = new KalmanPosVel2D(1e0, 1e-2);
				mTyFilterKF = new KalmanPosVel2D(1e0, 1e-2);
				mThetaFilterKF = new KalmanPosVel2D(1e0, 1e-2);

				R.create(2, 2, CV_32F);
				T.create(2, 1, CV_32F);
				R.setTo(0);
				T.setTo(0);
				R.at<float>(0,0) = 1;
				R.at<float>(1,1) = 1;
			}

			cv::Mat dispOccMap, distTrans, skelIn, skelDist, skelLap, skelConv, skelOut;
			cv::convertScaleAbs(occMap, dispOccMap, 1.0/5.0);

			cv::threshold(dispOccMap, skelIn, skelThresh, 255, cv::THRESH_BINARY);
			skelDist.setTo(0);
			cv::distanceTransform(skelIn, skelDist, CV_DIST_L2, 3);

			cv::Laplacian(skelDist, skelLap, CV_32F, 5);
			double minv, maxv;
			cv::minMaxIdx(skelLap, &minv, &maxv);
			cv::convertScaleAbs(skelLap, skelConv, 255/(maxv - minv), minv);
			cv::threshold(skelConv, skelOut, 150, 225, CV_THRESH_BINARY);

			std::vector<cv::Point> mapPts;
			for (int i = 0; i < skelOut.rows; i++)
			{
				unsigned char *skelOutPtr = (unsigned char*)(skelOut.ptr() + i*skelOut.step);
				for (int j = 0; j < skelOut.cols; j++)
				{
					if (*skelOutPtr++)
						mapPts.push_back(cv::Point(j,i));
				}
			}

			// Eliminate fringing effects around light sources
			cv::Mat tooBrightThresh, tooBrightDist;
			cv::threshold(channels[2], tooBrightThresh, 240, 255, CV_THRESH_BINARY_INV);
			cv::distanceTransform(tooBrightThresh, tooBrightDist, CV_DIST_L2, 3);
			std::vector<cv::Point> pts;
			float brightThresh = 15;
			veinMap.setTo(0);
			mOutMutex.enter();
			mVeinPts.clear();
			for (int i = 0; i < veinPts.size(); i++)
			{
				mVeinPts.push_back(veinPts[i]);
				float dist = tooBrightDist.at<float>(veinPts[i]);
				//printf("    %f\n", dist);
				if (dist < brightThresh)
				{
					veinMap.at<unsigned char>(veinPts[i]) = 0;
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

					if (r > CULL_R_THRESH &&  r > (float)g*CULL_RG_THRESH)
					{
						veinMap.at<unsigned char>(veinPts[i]) = 255;
						pts.push_back(veinPts[i]);
					}
					else
					{
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
			veinPts = pts;
			mOutMutex.leave();

			//printf("   *%d\n", veinPts.size());
			//std::vector<cv::Point> pts;
			//for (int i = 0; i < veinMap.rows; i++)
			//{
			//	unsigned char *veinMapPtr = (unsigned char*)(veinMap.ptr() + i*veinMap.step);
			//	for (int j = 0; j < veinMap.cols; j++)
			//	{
			//		if (*veinMapPtr++)
			//		{
			//			cv::Point pt = cv::Point(j,i);
			//			float dist = tooBrightDist.at<float>(pt);
			//			printf("    %f\n", dist);
			//			if (dist > brightThresh)
			//				pts.push_back(pt);
			//			else
			//				cv::circle(veinOut, veinPts[i], 3, CV_RGB(255, 0, 0), 1);
			//		}
			//	}
			//}
			//veinPts = pts;

			std::vector<int> idx;
			std::vector<double> dist;

			// Randomly subsample so we don't have too many points
			cv::Mat Rcopy = R.clone(), Tcopy = T.clone();
			std::vector<cv::Point2f> origMapPts;
			for (int i = 0; i < mapPts.size(); i++)
				origMapPts.push_back(mapPts[i]);
			if (1)
			{
				int maxPts = 500;
				std::vector<int> sub;
				std::vector<cv::Point> npts;

				sub = randperm(mapPts.size());
				npts.clear();
				for (int i = 0; i < MIN(mapPts.size(), maxPts); i++)
				{
					npts.push_back(mapPts[sub[i]]);
				}
				mapPts = npts;

				sub = randperm(veinPts.size());
				npts.clear();
				for (int i = 0; i < MIN(veinPts.size(), maxPts); i++)
				{
					npts.push_back(veinPts[sub[i]]);
				}
				veinPts = npts;
			}

			cv::Mat subVein = veinMap.clone(), subMap = skelOut.clone();
			subVein.setTo(0);
			subMap.setTo(0);
			for (int i = 0; i < veinPts.size(); i++)
				subVein.at<unsigned char>(veinPts[i]) = 255;
			for (int i = 0; i < mapPts.size(); i++)
				subMap.at<unsigned char>(mapPts[i]) = 255;

			int numInliers = 0;
			int icpStartTime, icpEndTime;
			if (1)
			{
				std::vector<cv::Point> &map = mapPts;
				std::vector<cv::Point> &pts = veinPts;
				//cv::Mat m_indices(pts.size(), 1, CV_32S);
				//cv::Mat m_dists(pts.size(), 1, CV_32F);

				//cv::Mat dest_32f(mapPts.size(), 2, CV_32F);
				//for (int i = 0; i < mapPts.size(); i++)
				//{
				//	dest_32f.at<cv::Point>(i,0) = mapPts[i];
				//}

				//cv::Mat obj_32f(veinPts.size(), 2, CV_32F);
				//for (int i = 0; i < veinPts.size(); i++)
				//{
				//	obj_32f.at<cv::Point>(i,0) = veinPts[i];
				//}
				//} 
				//} 
				std::vector<cv::Point2f> mapf;
				std::vector<cv::Point2f> ptsf;

				mapf.reserve(map.size());
				ptsf.reserve(pts.size());
				for (int i = 0; i < map.size(); i++)
					mapf.push_back(map[i]);
				for (int i = 0; i < pts.size(); i++)
					ptsf.push_back(pts[i]);

				if (0)
				{
					FILE *fp = 0;

					float x, y;
					fp = fopen("E:\\VirtualFixtures\\pigeyes\\Lobes_02-26-2012\\first cannulation\\mapPts.txt", "rb");
					mapf.clear();
					while (fscanf(fp, "%f %f\n", &x, &y) == 2)
						mapf.push_back(cv::Point2f(x, y));
					fclose(fp);

					fp = fopen("E:\\VirtualFixtures\\pigeyes\\Lobes_02-26-2012\\first cannulation\\sensPts.txt", "rb");
					ptsf.clear();
					while (fscanf(fp, "%f %f\n", &x, &y) == 2)
						ptsf.push_back(cv::Point2f(x, y));
					fclose(fp);
				}

				if (0)
				{
					cv::Mat R, T;
					R.create(2, 2, CV_32F);
					T.create(2, 1, CV_32F);
					R.setTo(0);
					T.setTo(0);
					R.at<float>(0,0) = 1;
					R.at<float>(1,1) = 1;
					Rcopy = R.clone();
					Tcopy = T.clone();
				}

				icpStartTime = timeGetTime();
				icp(mapf, ptsf, Rcopy, Tcopy, numInliers);
				icpEndTime = timeGetTime();
				if (veinPts.size() > minInliers && numInliers > minInliers)
				{
					R = Rcopy.clone();
					T = Tcopy.clone();
				}
			}

			if (0)
			{
				cv::Mat flow;
				cv::calcOpticalFlowFarneback(veinMap, skelOut, flow, 0.5, 2, 10, 2, 7, 1.5, 0);
			}

			//startTime = timeGetTime();
			if (0)
			{
				for (int i = 0; i < 20; i++)
				{
					knnNaiveCPU(mapPts, veinPts, idx, dist);
				}
			}

			if (0)
			{
				std::vector<cv::Point> &map = mapPts;
				std::vector<cv::Point> &pts = veinPts;
				cv::Mat m_indices(pts.size(), 1, CV_32S);
				cv::Mat m_dists(pts.size(), 1, CV_32F);

				cv::Mat dest_32f(mapPts.size(), 2, CV_32F);
				for (int i = 0; i < mapPts.size(); i++)
				{
					dest_32f.at<cv::Point>(i,0) = mapPts[i];
				}

				cv::Mat obj_32f(veinPts.size(), 2, CV_32F);
				for (int i = 0; i < veinPts.size(); i++)
				{
					obj_32f.at<cv::Point>(i,0) = veinPts[i];
				}

				//if (dest_32f.type() == CV_32F)
				//	printf("bad???");
				cv::flann::Index flann_index(dest_32f, cv::flann::KDTreeIndexParams(2));  // using 4 randomized kdtrees

				for (int i = 0; i < 20; i++)
				{
					flann_index.knnSearch(obj_32f, m_indices, m_dists, 1, cv::flann::SearchParams(32) ); // maximum number of leafs checked

					int* indices_ptr = m_indices.ptr<int>(0);
					//float* dists_ptr = m_dists.ptr<float>(0);
					for (int i=0;i<m_indices.rows;++i) {
						idx.push_back(indices_ptr[i]);
					}

					dist.resize(m_dists.rows);
					m_dists.copyTo(cv::Mat(dist));
				}
			}

			if (0)
			{
				std::vector<cv::Point> &map = mapPts;
				std::vector<cv::Point> &pts = veinPts;
				cv::Mat m_indices(pts.size(), 1, CV_32S);
				cv::Mat m_dists(pts.size(), 1, CV_32F);

				cv::Mat dest_32f(mapPts.size(), 2, CV_32F);
				for (int i = 0; i < mapPts.size(); i++)
				{
					dest_32f.at<cv::Point>(i,0) = mapPts[i];
				}

				cv::Mat obj_32f(veinPts.size(), 2, CV_32F);
				for (int i = 0; i < veinPts.size(); i++)
				{
					obj_32f.at<cv::Point>(i,0) = veinPts[i];
				}

				for (int i = 0; i < 20; i++)
				{
					//knnBruteGPU(dest_32f, obj_32f, idx, dist);
				}
			}

			/// Do mapping
			cv::Mat gblurMap;
			cv::Mat warpedVeinMap = veinMap.clone();
			warpedVeinMap.setTo(0);
			std::vector<cv::Point2f> warpedVeinPts;
			for (int i = 0; i < veinPts.size(); i++)
				warpedVeinPts.push_back(veinPts[i]);

			// Here we use Rcopy and Tcopy to get the best update of the map
			applyTrans(warpedVeinPts, Rcopy, Tcopy);
			for (int i = 0; i < warpedVeinPts.size(); i++)
			{
				cv::Point pt = warpedVeinPts[i];
				if (pt.x < 0 || pt.y < 0 || pt.x >= warpedVeinMap.cols || pt.y >= warpedVeinMap.rows)
					continue;
				warpedVeinMap.at<unsigned char>(warpedVeinPts[i]) = 255;
				//cv::circle(veinOut, pt, 3, CV_RGB(0, 255, 0), 1);
			}

			// Filter!
			cv::Mat Rf = R.clone(), Tf = T.clone();
			if (1)
			{
				float theta = atan2(R.at<float>(1,0), R.at<float>(0,0));
				float tx = Tf.at<float>(0,0);
				float ty = Tf.at<float>(1,0);
				// Low pass
				if (0)
				{
					theta = mThetaFilter->filter(theta);
					Tf.at<float>(0,0) = mTxFilter->filter(tx);
					Tf.at<float>(1,0) = mTyFilter->filter(ty);
					if (first)
					{
						for (int i = 0; i < 100; i++)
						{
							theta = mThetaFilter->filter(theta);
							Tf.at<float>(0,0) = mTxFilter->filter(Tf.at<float>(0,0));
							Tf.at<float>(1,0) = mTyFilter->filter(Tf.at<float>(1,0));					
						}
					}
				}
				else
				{
					mThetaFilterKF->correct(theta);
					mTxFilterKF->correct(tx);
					mTyFilterKF->correct(ty);
					theta = mThetaFilterKF->predict();
					Tf.at<float>(0,0) = mTxFilterKF->predict();
					Tf.at<float>(1,0) = mTyFilterKF->predict();
				}

				Rf.at<float>(0,0) = cos(theta);
				Rf.at<float>(0,1) = -sin(theta);
				Rf.at<float>(1,0) = sin(theta);
				Rf.at<float>(1,1) = cos(theta);
			}

			std::vector<cv::Point2f> warpedMapPts;
			for (int i = 0; i < origMapPts.size(); i++)
				warpedMapPts.push_back(origMapPts[i]);
			applyTrans(warpedMapPts, Rf.t(), -Rf.t()*Tf);

			mOutMutex.enter();
			mWarpedMapPts = warpedMapPts;
			mRf = Rf.clone();;
			mTf = Tf.clone();
			mOutMutex.leave();
			
			//FILE *fp = fopen(String().sprintf("out/warpedmap%0.4d.txt", iteration), "wb");
			//for (int i = 0; i < warpedMapPts.size(); i++)
			//	fprintf(fp, "%f %f\n", warpedMapPts[i].x, warpedMapPts[i].y);
			//fclose(fp);
			//fp = fopen(String().sprintf("out/trans%0.4d.txt", iteration), "wb");
			//fprintf(fp, "%f %f %f\n", Rcopy.at<float>(0,0), Rcopy.at<float>(0,1), Tcopy.at<float>(0,0));
			//fprintf(fp, "%f %f %f\n", Rcopy.at<float>(1,0), Rcopy.at<float>(1,1), Tcopy.at<float>(1,0));
			//fclose(fp);

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

			cv::GaussianBlur(warpedVeinMap, gblurMap, cv::Size(9,9), 3);

			for (int i = 0; i < occMap.rows; i++)
			{
				float *occMapPtr = (float*)(occMap.data + i*occMap.step);
				unsigned char *gblurMapPtr = (unsigned char*)gblurMap.data + i*gblurMap.step;
				for (int j = 0; j < occMap.cols; j++)
				{
					*occMapPtr += *gblurMapPtr*blurScale;
					if (*occMapPtr > occMax)
						*occMapPtr = occMax;
					if (*occMapPtr > occDecay)
						*occMapPtr -= occDecay;
					//if (*occMapPtr < occMin)
					//	*occMapPtr = occMin;

					occMapPtr++; gblurMapPtr++;
				}
			}

			cv::convertScaleAbs(occMap, dispOccMap, 1.0/5.0);

			mOutMutex.enter();
			mOccMap = dispOccMap.clone();
			mOutMutex.leave();

			//}

			//Skeletonize skel;
			//cv::Mat skelSkel = skelIn.clone();
			//IplImage tmp(skelSkel);
			//skel.skeletonize(&tmp);

			int endTime = timeGetTime();
			totTime += endTime - startTime;
			ctr++;

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
		}

		Sleep(1);
	}
}

int CEyeSLAM::absor(const cv::Mat &map, const cv::Mat &pts, cv::Mat &R, cv::Mat &T)
{
	double sumts = 1;

	return 0;	
}

int CEyeSLAM::absor(const std::vector<cv::Point2f> &map, const std::vector<cv::Point2f> &pts, cv::Mat &R, cv::Mat &T)
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


void CEyeSLAM::subPt(std::vector<cv::Point2f> &pts, cv::Point2f pt)
{
	for (int i = 0; i < pts.size(); i++)
		pts[i] -= pt;
}

cv::Point2f CEyeSLAM::calcMeanPt(const std::vector<cv::Point2f> &pts)
{
	cv::Point2f meanPt;

	for (int i = 0; i < pts.size(); i++)
	{
		meanPt += pts[i];
	}

	meanPt = meanPt * (1.0 / pts.size());

	return meanPt;
}

int CEyeSLAM::icp(const cv::Mat &map, const cv::Mat &pts, cv::Mat &R, cv::Mat &T)
{
	int deadMan = 40;

	for (int k = 0; k < deadMan; k++)
	{

	}

	return 0;
}

int CEyeSLAM::icp(const std::vector<cv::Point2f> &map, const std::vector<cv::Point2f> &pts, cv::Mat &R, cv::Mat &T, int &numInliers)
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

	if (0)
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
	}

	cv::vector<cv::Point2f> warpedPts = pts;

	cv::Mat Rt(2, 2, CV_32F);
	cv::Mat Tt(2, 1, CV_32F);
	Rt.setTo(0);
	Rt.at<float>(0,0) = 1;
	Rt.at<float>(1,1) = 1;
	Tt.setTo(0);

	if (Rt.rows == R.rows && Rt.cols == R.cols && Rt.type() == R.type())
		Rt = R.clone();
	if (Tt.rows == T.rows && Tt.cols == T.cols && Tt.type() == T.type())
		Tt = T.clone();

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

		for (int i = 0; i < warpedPts.size(); i++)
		{
			float minDist = 1e9;
			int minIdx = -1;
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

			if (minDist < distThreshold)
			{
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

		if (curErr >= 0 && lastErr >= 0 && fabs(curErr - lastErr) < 1e-2)
			break;

		if (mapMatchPts.size() > matchesThreshold)
		{
			cv::Mat r, t;
			absor(ptsMatchPts, mapMatchPts, r, t);
			Rt = Rt*r;
			Tt = Tt + t;
		}
		else
		{
			//printf("could not icp, not enough points for good solution....\n");
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

void CEyeSLAM::applyTrans(std::vector<cv::Point2f> &pts, const cv::Mat &R, const cv::Mat &T)
{
	float R00 = R.at<float>(0,0);
	float R01 = R.at<float>(0,1);
	float R10 = R.at<float>(1,0);
	float R11 = R.at<float>(1,1);
	float *Tptr = (float*)T.data;
	float Tx = Tptr[0];
	float Ty = Tptr[1];
	for (int i = 0; i < pts.size(); i++)
	{
		float x = pts[i].x*R00 + pts[i].y*R01 + Tx;
		float y = pts[i].x*R10 + pts[i].y*R11 + Ty;
		pts[i].x = x;
		pts[i].y = y;
	}
}

int CEyeSLAM::knnNaiveCPU(const std::vector<cv::Point> &map, const std::vector<cv::Point> pts, std::vector<int> &idx, std::vector<double> &dist)
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

void CEyeSLAM::slamIt(const cv::Mat &img)
{
	mMutex.enter();
	mInput.enqueue(img.clone());
	mMutex.leave();
}

void CEyeSLAM::vecOfPtsToMat(const std::vector<cv::Point2f> &pts, cv::Mat &mat)
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
