#include "scanMatching.h"


scanMatching::scanMatching()
{
	// number of iterations performed by the brute Force algorithm
	mPtsOutx = 0;
	mPtsOuty = 0;
}


scanMatching::~scanMatching()
{
}
/* Old and slow version
void scanMatching::lowResolutionMat(cv::Mat &input, int x, int y)
{
	// to be sure we are not missing details from the reference scan by scaling the image, we are using  a low resolution table, 
	// were each cell is set to the maximum value of the corresponding cells in the high resolution map.

	// input : the rescaled map
	// x and y : are the pixel dimensions. If y is not specified or equal to zero, it is equal to x.

	// pixel size
	if (y <= 0)
		y = x;

	cv::Mat subMat;
	double maxv, minv;
	int xmax, ymax;
	for (int i = 0; i < input.cols; i += x)
	{
		for (int j = 0; j < input.rows; j += y)
		{
			xmax = i + x - 1;
			ymax = j + y - 1;
			if (xmax >= input.cols)
				xmax = input.cols - 1;
			if (ymax >= input.rows)
				ymax = input.rows - 1;

			if (i >= xmax || j >= ymax)
				break;

			// extract a submatrix
			subMat = input.colRange(i, xmax).rowRange(j, ymax);
			// find the max value
			cv::minMaxIdx(subMat, &minv, &maxv);
			subMat = cv::Mat::ones(ymax-j+1, xmax-i+1, subMat.type())*maxv;
			for (int k = 0; k < subMat.rows; k++)
			{
				unsigned char* PtrSubMat = subMat.ptr<unsigned char>(k);
				unsigned char* PtrInput = input.ptr<unsigned char>(k + j);
				for (int l = 0; l < subMat.cols; l++)
					PtrInput[l + i] = PtrSubMat[l];
			}
		}
	}
}
*/

// new low resolution building map
void scanMatching::lowResolutionMat(cv::Mat &input, int x, int y)
{
	// to be sure we are not missing details from the reference scan by scaling the image, we are using  a low resolution table, 
	// were each cell is set to the maximum value of the corresponding cells in the high resolution map.

	// input : the rescaled map
	// x and y : are the pixel dimensions. If y is not specified or equal to zero, it is equal to x.

	// pixel size
	if (y <= 0)
		y = x;

	// Lower resolution map will be smaller by a factor of x and y
	cv::Size lowResSize(input.cols / x + 1, input.rows / y + 1);

	// Create lower resolution map with smaller size
	cv::Mat lowMap(lowResSize, input.type());
	lowMap.setTo(0);

	cv::Mat subMat;
	// store the value of each pixels
	//std::vector<int> pxVal(input.cols/x*input.rows/y+1,0);
	//std::vector<unsigned char> pxVal;
	double maxv, minv;
	int xmax, ymax;
	for (int j = 0; j < input.rows; j += y)
	{
		ymax = j + y - 1;
		if (ymax >= input.rows)
			ymax = input.rows - 1;
		// if there is no positive value on the row, we just skip it
		if (countNonZero(input.rowRange(j, ymax)) < 1)
			continue;

		for (int i = 0; i < input.cols; i += x)
		{
			xmax = i + x - 1;
			if (xmax >= input.cols)
				xmax = input.cols - 1;

			// if there is no positive value on the col, we just skip it
			// This test is not really necessary
		//	if (countNonZero(input.colRange(i, i + x - 1).rowRange(j, ymax) < 1))
		//		continue;

			if (i >= xmax || j >= ymax)
				break;

			// extract a submatrix
			subMat = input.colRange(i, xmax).rowRange(j, ymax);
			// find the max value
			cv::minMaxIdx(subMat, &minv, &maxv);

			lowMap.at<unsigned char>(cv::Point(i / x, j / y)) = maxv;

			//We are storing the max value in the vector. The map will be built later, outside the loop
			//pxVal.push_back(maxv);
			//subMat = cv::Mat::ones(ymax - j + 1, xmax - i + 1, subMat.type())*maxv;
		}
	}
	/*
	// We are building the low resolution Map
	for (int k = 0; k < input.rows; k++)
	{
		//unsigned char* PtrSubMat = subMat.ptr<unsigned char>(k);
		unsigned char* PtrInput = input.ptr<unsigned char>(k);
		for (int l = 0; l < input.cols; l++)
			PtrInput[l] = pxVal[floor(l/x)+floor(k/y)*floor(input.cols/x)];
	}
	*/

	cv::resize(lowMap, input, input.size(),0,0,cv::INTER_NEAREST);
}

// Test all of the configurations
void scanMatching::bruteForce(cv::Mat map, cv::Mat img, std::vector<cv::Point2f> veinPts, cv::Mat &T, cv::Mat &R)
{

	if (!veinPts.size())
	{
		std::cout << "The vector of vein points is empty\n";
		std::cout << "Tx : " << T.at<float>(0, 0) << " px\n";
		std::cout << "Ty : " << T.at<float>(1, 0) << " px\n";
		std::cout << "Theta : " << atan2(R.at<float>(1, 0), R.at<float>(1, 1)) * 180 / M_PI << " deg\n";
		return;
	}

	cv::Mat scoreMap = map;
	// if the map is empty
	if (countNonZero(scoreMap) < 1)
	{
		std::cout << "The map is empty\n";
		std::cout << "Tx : " << T.at<float>(0, 0) << " px\n";
		std::cout << "Ty : " << T.at<float>(1, 0) << " px\n";
		std::cout << "Theta : " << atan2(R.at<float>(1, 0), R.at<float>(1, 1)) * 180 / M_PI << " deg\n";
		return;	
	}
		

	// Transformation parameters
	applyTrans(veinPts, R, T);

	
	/*
	// Rasterization
	// create the 2D map of reference
	cv::Mat scoreMap(map.size(), CV_8U);
	scoreMap.setTo(0);
	// create the 2D map of the detected vein on the current frame
	cv::Mat veinMap = scoreMap.clone();

	
	
	for (int i = 0; i < mapPts.size(); i++)
	{
		cv::circle(scoreMap, mapPts[i], 1, 255);
		cv::circle(veinMap, veinPts[i], 1, 255);
	}
		
	cv::GaussianBlur(scoreMap, scoreMap, cv::Size(9, 9), 3);
	cv::GaussianBlur(veinMap, veinMap, cv::Size(9, 9), 3);

	cv::imshow("scoreMap", scoreMap);
	cv::imshow("vein Map", veinMap);
	cvWaitKey(1);

	// Transformation Matrix
	cv::Mat M(2, 3, CV_32F);
	*/


	float R10 = R.at<float>(1, 0);
	float R11 = R.at<float>(1, 1);
	float theta = atan2(R10, R11) * 180 / M_PI;
	float Tx = T.at<float>(0, 0);
	float Ty = T.at<float>(1, 0);


	std::vector<cv::Point2f> pts = veinPts;

	float bestScore = -1e9;
	float score = 0;
	int matchPts = 0;
	int bestMatch = 0;

	//cv::Mat dispMat = map.clone();



	for (int dtheta = -2; dtheta < 2; dtheta++)
	{
		
		// apply the rotation 
		//for (int i = 0; i < pts.size(); i++)
		//{
		//	pts[i].x = pts[i].x*cos(dtheta*M_PI / 180) - pts[i].y*sin(dtheta*M_PI / 180);
		//	pts[i].y = pts[i].x*sin(dtheta*M_PI / 180) + pts[i].y*cos(dtheta*M_PI / 180);
		//}
		int x0 = map.cols / 2;
		for (int dx = -x0; dx < x0; dx++)
		{
			//for (int i = 0; i < pts.size(); i++)
				//pts[i].x += dx;

			int y0 = map.rows / 2;
			for (int dy = -y0; dy < y0; dy++)
			{	
				for (int i = 0; i < pts.size(); i++)
				{
					int xPt = pts[i].x*cos(dtheta*M_PI / 180) - pts[i].y*sin(dtheta*M_PI / 180) + dx;
					int yPt = pts[i].x*sin(dtheta*M_PI / 180) + pts[i].y*cos(dtheta*M_PI / 180) + dy;

					if (xPt <= 0 || xPt >= scoreMap.cols || yPt <= 0 || yPt >= scoreMap.rows)
						continue;

					if (scoreMap.at<unsigned char>(yPt, xPt))
					{
						score += scoreMap.at<unsigned char>(yPt, xPt);
						matchPts++;
					}
						
				}

				if ((score > bestScore) && (matchPts > pts.size() / 2))
				{
					bestScore = score;
					bestMatch = matchPts;

					T.at<float>(0, 0) = Tx + dx;
					T.at<float>(1, 0) = Ty + dy;
					R.at<float>(0, 0) = cos(dtheta*M_PI/180);
					R.at<float>(0, 1) = -sin(dtheta*M_PI/180);
					R.at<float>(1, 0) = sin(dtheta*M_PI/180);
					R.at<float>(1, 1) = cos(dtheta*M_PI/180);

					//cv::imshow("dispMat", dispMat);
					//cvWaitKey(1);
				}	
				score = 0;
				matchPts = 0;
				//dispMat = dispMat.setTo(0);
			}
		}
	}

	std::cout << "Best score : " << bestScore << "\n";
	std::cout << "Point matched : " << bestMatch << "\n";
	std::cout << "Tx : " << T.at<float>(0, 0) << " px\n";
	std::cout << "Ty : " << T.at<float>(1, 0) << " px\n";
	std::cout << "Theta : " << atan2(R.at<float>(1, 0), R.at<float>(1, 1)) * 180 / M_PI << " deg\n";

	//pts = veinPts;
	//applyTrans(pts, R, T);
	//for (int i = 0; i < pts.size(); i++)
		//cv::circle(dispMat, pts[i], 2, 200);
	//cv::imshow("dispMat", dispMat);
	//cvWaitKey(1);
	//dispMat = map.clone();
	
}


void scanMatching::multiResolution(cv::Mat map, std::vector<cv::Point2f> veinPts, cv::Mat &T, cv::Mat &R, int &fail, int xPxSize, int yPxSize)
{
	cv::Mat Tmr = T.clone();
	cv::Mat Rmr = R.clone();

	mVeinPtsLength = veinPts.size();

	// do not compute if there is not not enough points
	if (veinPts.size() < 40)
	{
		fail++;
		std::cout << "The vector of vein points is too small or empty\n";
		std::cout << "Tx : " << T.at<float>(0, 0) << " px\n";
		std::cout << "Ty : " << T.at<float>(1, 0) << " px\n";
		std::cout << "Theta : " << atan2(R.at<float>(1, 0), R.at<float>(1, 1)) * 180 / M_PI << " deg\n";
		return;
	}

	//cv::Mat scoreMap = map;
	// if the map is empty
	if (countNonZero(map) < 1)
	{
		fail++;
		std::cout << "The map is empty\n";
		std::cout << "Tx : " << T.at<float>(0, 0) << " px\n";
		std::cout << "Ty : " << T.at<float>(1, 0) << " px\n";
		std::cout << "Theta : " << atan2(R.at<float>(1, 0), R.at<float>(1, 1)) * 180 / M_PI << " deg\n";
		return;
	}

	//didn't fail, we restart the counter
	fail = 0;

	// Transformation parameters
	// It is now performed outside the function ( in the EyeSLAM function )
	//applyTrans(veinPts, R, T);


	float R10 = R.at<float>(1, 0);
	float R11 = R.at<float>(1, 1);
	float theta = atan2(R10, R11) * 180 / M_PI;
	float Tx = T.at<float>(0, 0);
	float Ty = T.at<float>(1, 0);

	printf("Original Tx = %.2f, Ty = %.2f, R = %.2f\n", Tx, Ty, theta);

	// Score variables
	float bestScore = -1e9;
	float score = 0;
	int matchPts = 0;
	int bestMatch = 0;
	// best location on the low resolution loop
	int bdx, bdy, bdtheta;

	// ======================== //
	//		LOW RESOLUTION		//
	// ======================== //
	int beginLowResScanMatching = timeGetTime();

	int beginLowResMap = timeGetTime();
	cv::Mat lowMap = map.clone();

	// if the pixel size equal zero, it means that is is set to a default value defined here
	if (xPxSize == 0 && yPxSize == 0)
	{
		//The optimized value to analyse a area of 120x120 in the image
		//xPxSize = 5;
		//yPxSize = 5;

		//if we test half of the points for the low resolution, the new step size is :
		xPxSize = 4;
		yPxSize = 4;

	}
	// if only xPxSize has been defined
	else if (yPxSize == 0)
	{
		yPxSize = xPxSize;
	}

	lowResolutionMat(lowMap, xPxSize, yPxSize);
	//cv::resize(map, lowMap, map.size(), 1/xPxSize, 1/yPxSize, cv::INTER_AREA);
	
	int endLowResMap = timeGetTime();
	mLowResMap = endLowResMap - beginLowResMap;
//	cv::imshow("lowMap", lowMap);
//	cvWaitKey(1);
	// To spare processing time, we want to reduce iteration number.
	// It can be done by not calculating the points if most of them are already outside the image borders.
	// The loops are slip into positive and negative direction and stop if more than 20% of the points are outside the borders.

	//int ptsOut = 0;

	//optimised value
	int dthetaStep = 6;
	std::vector<cv::Point2f> Pts;
	Pts.resize(veinPts.size());
	int trMIN = -60;
	int trMAX = 60;
	int thetaMIN = -6;
	int thetaMAX = 6;
	// percentage of missed points allowed
	double missedMargin = 0.2;

	double delta_movement_cost = 0;

	// the step size of each parameters has been optimized to reduce the processing time at its maximum

//	for (int dtheta = thetaMIN; dtheta < thetaMAX; dtheta += dthetaStep)
//	{
		//we are testing half of the points
	//	for (int i = 0; i < Pts.size(); i++)
	//	{
	//		Pts[i].x = veinPts[i].x*cos(dtheta*M_PI / 180) - veinPts[i].y*sin(dtheta*M_PI / 180);
	//		Pts[i].y = veinPts[i].x*sin(dtheta*M_PI / 180) + veinPts[i].y*cos(dtheta*M_PI / 180);
	//	}

		for (int dx = trMIN; dx < trMAX; dx += xPxSize)
		{
			for (int dy = trMIN; dy < trMAX; dy += yPxSize)
			{
				score = 0;
				matchPts = 0;
				for (int i = 0; i < veinPts.size();i+=2)
				{
					int xPt = veinPts[i].x + dx;
					int yPt = veinPts[i].y + dy;

					if (xPt <= 0 || xPt >= map.cols || yPt <= 0 || yPt >= map.rows)
						continue;

					if (lowMap.at<unsigned char>(yPt, xPt))
					{
						score += lowMap.at<unsigned char>(yPt, xPt);
						matchPts++;
					}

					// if the number of match points is not enough to score
					// if we scan at least 20% of the points and if we missed more than 20% of the points, we can break the loop
					// we are only scanning half of the points, so we have to divide some values by two.
					if (i / 2 - matchPts > veinPts.size()*missedMargin / 2)
						break;
				}

				// Add a slight preference towards not moving, e.g. if there is a close match that doesn't move as much, 
				// it's very possible that is the better choice. 
				double cost_of_moving = abs(bdx)*delta_movement_cost + abs(bdy)*delta_movement_cost;
				//printf("Original cost: %.1f, Moving cost: %.1f, Total cost: %.1f\n", score, cost_of_moving, score - cost_of_moving);
				score -= cost_of_moving;

				if ((score > bestScore) && (matchPts > veinPts.size()*missedMargin/2))
				{
					bestScore = score;
					bestMatch = matchPts;
					/*
					T.at<float>(0, 0) = Tx + dx;
					T.at<float>(1, 0) = Ty + dy;
					R.at<float>(0, 0) = cos(dtheta*M_PI / 180);
					R.at<float>(0, 1) = -sin(dtheta*M_PI / 180);
					R.at<float>(1, 0) = sin(dtheta*M_PI / 180);
					R.at<float>(1, 1) = cos(dtheta*M_PI / 180);
					*/
					bdx = dx;
					bdy = dy;
					bdtheta = 0;

				}
				//	insideLoop(lowMap, Pts, dx, dy, dtheta, bdx, bdy, bdtheta, bestScore, bestMatch);
			}
		}

		/*
		// Positive x direction
		for (int dx = 0; mPtsOutx < 0.2*veinPts.size(); dx += xPxSize)
		{
			// positive y direction
			for (int dy = 0; mPtsOuty < 0.2*veinPts.size(); dy += yPxSize)
			{
				insideLoop(lowMap, Pts, dx, dy, dtheta, bdx, bdy, bdtheta, bestScore, bestMatch);
			}
			mPtsOuty = 0;
			// negative y direction
			for (int dy = -1; mPtsOuty < 0.2*veinPts.size(); dy -= yPxSize)
			{
				insideLoop(lowMap, Pts, dx, dy, dtheta, bdx, bdy, bdtheta, bestScore, bestMatch);
			}
			mPtsOuty = 0;
		}
		mPtsOutx = 0;
		

		// Negative x direction
		for (int dx = -1; mPtsOutx < 0.2*veinPts.size(); dx -= xPxSize)
		{
			// positive y direction
			for (int dy = 0; mPtsOuty < 0.2*veinPts.size(); dy += yPxSize)
			{
				insideLoop(lowMap, Pts, dx, dy, dtheta, bdx, bdy, bdtheta, bestScore, bestMatch);
			}
			mPtsOuty = 0;
			// negative y direction
			for (int dy = -1; mPtsOuty < 0.2*veinPts.size(); dy -= yPxSize)
			{
				insideLoop(lowMap, Pts, dx, dy, dtheta, bdx, bdy, bdtheta, bestScore, bestMatch);
			}
			mPtsOuty = 0;
		}
		mPtsOutx = 0;
		*/
//	}

	//std::cout << "--- Low Resolution ---\n";
	//std::cout << "| Best score : " << bestScore << "\n";
	//std::cout << "| Total of vein points : " << veinPts.size()/2 << "\n";
	//std::cout << "| Total of matched points : " << bestMatch << "\n";
	//std::cout << "| Tx : " << Tx + bdx << " px\n";
	//std::cout << "| Ty : " << Ty + bdy << " px\n";
	//std::cout << "| Theta : " << theta + atan2(sin(bdtheta*M_PI / 180), cos(bdtheta*M_PI / 180)) * 180 / M_PI << " deg\n";

	if (bestScore <= 0)
	{
		return;
	}

	// we reset the best parameters, because obviously the bestscore in high def will be worste than in low def
	bestScore = 0;
	bestMatch = 0;


	int endLowResScanMatching = timeGetTime();
	mLowResScanMatching = endLowResScanMatching - beginLowResScanMatching;

	// ======================== //
	//		HIGH RESOLUTION		//
	// ======================== //
	int beginHighResScanMatching = timeGetTime();

	for (float dtheta = bdtheta - dthetaStep; dtheta < bdtheta + dthetaStep; dtheta++)
	{
		for (int i = 0; i < Pts.size(); i++)
		{
			Pts[i].x = veinPts[i].x*cos(dtheta*M_PI / 180) - veinPts[i].y*sin(dtheta*M_PI / 180);
			Pts[i].y = veinPts[i].x*sin(dtheta*M_PI / 180) + veinPts[i].y*cos(dtheta*M_PI / 180);
		}
		for (int dx = bdx-xPxSize; dx < bdx+xPxSize; dx++)
		{
			for (int dy = bdy-yPxSize; dy < bdy+yPxSize; dy++)
			{
				for (int i = 0; i < veinPts.size(); i++)
				{
					int xPt = Pts[i].x + dx;
					int yPt = Pts[i].y + dy;

					if (xPt <= 0 || xPt >= map.cols || yPt <= 0 || yPt >= map.rows)
						continue;

					if (map.at<unsigned char>(yPt, xPt))
					{
						/*
						double val = 0;
						val += map.at<unsigned char>(yPt - 1, xPt - 1);
						val += 2 * map.at<unsigned char>(yPt - 1, xPt);
						val += map.at<unsigned char>(yPt - 1, xPt + 1);
						val += 2 * map.at<unsigned char>(yPt, xPt - 1);
						val += 4 * map.at<unsigned char>(yPt, xPt);
						val += 2 * map.at<unsigned char>(yPt, xPt + 1);
						val += map.at<unsigned char>(yPt + 1, xPt - 1);
						val += 2 * map.at<unsigned char>(yPt + 1, xPt);
						val += map.at<unsigned char>(yPt + 1, xPt + 1);
						val = val / 16;
						score += val;
						*/
						score += map.at<unsigned char>(yPt, xPt);
						matchPts++;
					}
					// same as low resolution
					//if (i - matchPts > veinPts.size()*missedMargin)
					//	break;
				}

				// Add a slight preference towards not moving, e.g. if there is a close match that doesn't move as much, 
				// it's very possible that is the better choice. 
				double cost_of_moving = abs(bdx)*delta_movement_cost + abs(bdy)*delta_movement_cost + abs(dtheta)*delta_movement_cost;
				//printf("Original cost: %.1f, Moving cost: %.1f, Total cost: %.1f\n", score, cost_of_moving, score - cost_of_moving);
				score -= cost_of_moving;

				if ((score > bestScore) && (matchPts > veinPts.size()*0.8))
				{
					bestScore = score;
					bestMatch = matchPts;

					Tmr.at<float>(0, 0) = dx;
					Tmr.at<float>(1, 0) = dy;
					Rmr.at<float>(0, 0) = cos(dtheta*M_PI / 180);
					Rmr.at<float>(0, 1) = -sin(dtheta*M_PI / 180);
					Rmr.at<float>(1, 0) = sin(dtheta*M_PI / 180);
					Rmr.at<float>(1, 1) = cos(dtheta*M_PI / 180);
					
					//cv::imshow("dispMat", dispMat);
					//cvWaitKey(1);
				}
				score = 0;
				matchPts = 0;
				//dispMat = dispMat.setTo(0);
			}
		}
	}

	//New rigid transformation model
	if (bestScore > 0)
	{
		T = T + R*Tmr;
		R = R*Rmr;
	}

	//std::cout << "--- High Resolution ---\n";
	//std::cout << "| Best score : " << bestScore << "\n";
	//std::cout << "| Total of vein points : " << veinPts.size() << "\n";
	//std::cout << "| Total of matched points : " << bestMatch << "\n";
	//std::cout << "| Tx : " << T.at<float>(0, 0) << " px\n";
	//std::cout << "| Ty : " << T.at<float>(1, 0) << " px\n";
	//std::cout << "| Theta : " << atan2(R.at<float>(1, 0), R.at<float>(1, 1)) * 180 / M_PI << " deg\n";


	int endHighResScanMatching = timeGetTime();
	mHighResScanMatching = endHighResScanMatching - beginHighResScanMatching;
}


// copy of the applyTrans function from EyeSLAM function
void scanMatching::applyTrans(std::vector<cv::Point2f> &pts, const cv::Mat &R, const cv::Mat &T)
{
	float R00 = R.at<float>(0, 0);
	float R01 = R.at<float>(0, 1);
	float R10 = R.at<float>(1, 0);
	float R11 = R.at<float>(1, 1);
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

void scanMatching::insideLoop(cv::Mat map, std::vector<cv::Point2f> veinPts, int dx, int dy, int dtheta, int &bdx, int &bdy, int &bdtheta, float &bestScore, int &bestMatch)
{
	int score = 0;
	int matchPts = 0;
	for (int i = 0; i < veinPts.size(); i++)
	{
		int xPt = veinPts[i].x + dx;
		int yPt = veinPts[i].y + dy;
		
		if (xPt <= 0 || xPt >= map.cols || yPt <= 0 || yPt >= map.rows)
		{
			/*
			if (xPt <= 0 || xPt >= map.cols)
				mPtsOutx++;
			if (yPt <= 0 || yPt >= map.cols)
				mPtsOuty++;
			*/
			continue;
		}
			



		if (map.at<unsigned char>(yPt, xPt))
		{
			/*
			// It's as if we applyed a gaussian blur 3x3 on the vein points
			double val = 0;
			val += map.at<unsigned char>(yPt - 1, xPt - 1);
			val += 2 * map.at<unsigned char>(yPt - 1, xPt);
			val += map.at<unsigned char>(yPt - 1, xPt + 1);
			val += 2 * map.at<unsigned char>(yPt, xPt - 1);
			val += 4 * map.at<unsigned char>(yPt, xPt);
			val += 2 * map.at<unsigned char>(yPt, xPt + 1);
			val += map.at<unsigned char>(yPt + 1, xPt - 1);
			val += 2 * map.at<unsigned char>(yPt + 1, xPt);
			val += map.at<unsigned char>(yPt + 1, xPt + 1);
			val = val / 16;
			score += val;
			*/
			score += map.at<unsigned char>(yPt, xPt);
			matchPts++;
		}

	}

	if ((score > bestScore) && (matchPts > veinPts.size()*0.8))
	{
		bestScore = score;
		bestMatch = matchPts;
		/*
		T.at<float>(0, 0) = Tx + dx;
		T.at<float>(1, 0) = Ty + dy;
		R.at<float>(0, 0) = cos(dtheta*M_PI / 180);
		R.at<float>(0, 1) = -sin(dtheta*M_PI / 180);
		R.at<float>(1, 0) = sin(dtheta*M_PI / 180);
		R.at<float>(1, 1) = cos(dtheta*M_PI / 180);
		*/
		bdx = dx;
		bdy = dy;
		bdtheta = dtheta;

	}
}