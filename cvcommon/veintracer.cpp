#include "veintracer.h"

//#include <WinSock2.h>
#include <Windows.h>

//#include <mmsystem.h>

VeinTracer::VeinTracer()
{
	for (int i = 0; i < 16; i++)
	{
		U[i][0] = cos(2.0*M_PI*i/16.0);
		U[i][1] = -sin(2.0*M_PI*i/16.0);

		Ur[i][0] = cos(2.0*M_PI*i/16.0 + M_PI/2);
		Ur[i][1] = -sin(2.0*M_PI*i/16.0 + M_PI/2);
	}
}

VeinTracer::~VeinTracer()
{
}

int VeinTracer::trace(const cv::Mat &input, const cv::Mat &input2, cv::Mat &map, cv::Mat &out, std::vector<cv::Point> &veinPts)
{
	int beginTime = timeGetTime();
	
	std::vector<cv::Point> outPts, initialPts, seedPts;
	std::vector<int> seedDirs;

	//Get the min and max value of the input to equilize the threshold value.
	double MinInput, MaxInput;
	cv::minMaxLoc(input, &MinInput, &MaxInput);

	//cv::equalizeHist(input, input);

	// Create the map if it doesn't exist already
	if (map.rows != input.rows || map.cols != input.cols)
	{
		map.create(input.size(), CV_8UC1);
	}
	// clear past results
	map.setTo(0);
	veinPts.clear();
	
	// out is not used anymore
	//input.copyTo(out);
	// Set constants
	// footstep between two rows/cols of the grid for the line search of initial points
	int N = 30;
	// Neighborhood size. Used in the 1D sliding window function (SlidingWindowMinimumNaive). 
	// A small value of Ns is needed to detect thin vessels, but leads to detect multiple local minima on thick vessels
	// Ns must be chosen to be >= than the widest expected vessel
	int Ns = 26;
	// border of the analysis in the edge detection image. DO NOT CHANGE THE VALUE : 25
	int minEdgeDist = 25;	
	// sensitive Threshold : should be under the sum of local maxima to validate the seed point
	int sensThresh = 200; // used to be 300
	// equilized threshold
	//int sensThresh = (250 - MinInput) / (MaxInput - MinInput);
	// maximum distance to find maxima around a seed point
	int lineSearchDist = 13;
	// 
	float alpha = 2.0;
	// minmum vessel length
	int minPtsPerVein = 25;
	
	//start the clock
//	timeBeginPeriod(1);
//	int start = timeGetTime();

	// III.E - Step 1: Line search over a coarse grid to find initial points
	std::vector<unsigned char> line, minLine;
	line.resize(input.cols);
	minLine.resize(input.cols); 
	// analysis along rows
	for (int i = minEdgeDist; i < input.rows - minEdgeDist; i += N)
	{
		//fill the entier line with value 255 (visual stuff probably) 
		std::fill(line.begin(), line.end(), 255);

		unsigned char *ptr = (unsigned char*)input.data + input.step*i;
		// line -> 1D Gaussian Kernel
		for (int j = 1; j < line.size() - 1; j++)
			line[j] = (ptr[j-1] >> 2) + (ptr[j] >> 1) + (ptr[j+1] >> 2);
		
		slidingWindowMinimumNaive(line, Ns, minLine);
		
		for (int j = minEdgeDist; j < minLine.size() - minEdgeDist; j++)
		{
			if (line[j] == minLine[j])
			{
				//int start = j;
				for (j++; line[j] == minLine[j] && j < minLine.size() - minEdgeDist; j++);
					initialPts.push_back(cv::Point(j, i));
					//initialPts.push_back(cv::Point((j+start)/2, i));
			}
		}
	}

	line.resize(input.rows);
	minLine.resize(input.rows);
	// analysis along cols
	for (int i = minEdgeDist; i < input.cols - minEdgeDist; i += N)
	{
		std::fill(line.begin(), line.end(), 255);

		unsigned char *ptr = (unsigned char*)input.data + i;
		for (int j = 1; j < line.size() - 1; j++)
			line[j] = (ptr[input.step*(j-1)] >> 2) + (ptr[input.step*j] >> 1) + (ptr[input.step*(j+1)] >> 2);
		
		slidingWindowMinimumNaive(line, Ns, minLine);
		
		for (int j = minEdgeDist; j < minLine.size() - minEdgeDist; j++)
		{
			if (line[j] == minLine[j])
			{
				//int start = j;
				for (j++; line[j] == minLine[j] && j < minLine.size() - minEdgeDist; j++);
					initialPts.push_back(cv::Point(i, j));
				//initialPts.push_back(cv::Point(i, (j+start)/2));
			}
		}
	}

	// III.E - Step 2: Filtering seed points from step 1
	const int len = 16;
	std::vector<float> XR, YR, VR, DR;
	// XR[k] : x coordinate of the maxima in direction k
	XR.resize(len);
	// YR[k] : y coordinate of the maxima in direction k
	YR.resize(len);
	// VR[k] : maxima value in direction k
	VR.resize(len);
	// DR[k] : distance of the maxima from the seed point in the direction k
	DR.resize(len);


	//Get the threshold value 
	unsigned char avgThresh;
	//double Mod = 0.5;
	//GetThresholdValue(input2, avgThresh, Mod);

	//Let's try with a fix value : 0.11*255;
	avgThresh = 0.11 * 255;

// begin debug mode : display filtered pixels from the input image
	// basic threshold
//	cv::Mat ThreshMat;
//	GetThresholdMat(input2, ThreshMat, avgThresh);
	// substraction threshold
	// cv::Mat ThreshMat;
	//cv::Mat imgR, imgG, diffMat;
	//medianBlur(input, imgG, 9);
	//medianBlur(input2, imgR, 9);
	//diffMat = imgR - imgG; //Red - Green
	//cv::threshold(diffMat, ThreshMat, avgThresh, 255, cv::THRESH_BINARY);
	//GetThresholdMat(ThreshMat, ThreshMat, avgThresh);
	/*
	// average threshold mode
	cv::Mat avgInput2, ThreshMat;
	double hdim = 20;
	//average kernel
	cv::Mat h = cv::Mat::ones(hdim, hdim, CV_32FC1) / (hdim*hdim);
	// apply average filter on input2, and save it on avgInput2
	cv::filter2D(input2, avgInput2, -1, h);
	//threshold
	GetThresholdMat(avgInput2, ThreshMat, avgThresh);
*/
	/*	
	// threshold after average filter
	double hdim = 20;
	//average kernel
	cv::Mat h = cv::Mat::ones(hdim, hdim, CV_32FC1) / (hdim*hdim);
	cv::Mat filtValueChan;
	cv::filter2D(input2, filtValueChan, -1, h);
	cv::Mat ThreshValAvg = input2.clone();
	ThreshValAvg.setTo(0);
	for (int i = 0; i < filtValueChan.rows; i++)
		for (int j = 0; j < filtValueChan.cols; j++)
			if (filtValueChan.at<unsigned char>(i, j) < avgValThresh)
				ThreshValAvg.at<unsigned char>(i, j) = 255;
	*/

//	cv::imshow("threshold", ThreshMat);
	//cv::imshow("mean threshold", ThreshValAvg);
	
//	cvWaitKey(1);


	for (int i = 0; i < initialPts.size(); i++)
	{
		//Added by Daniel, black value filter. If the initial seed point is in a black or dark area, it is probably a false positive seed point. based on value channel
		if (input2.at<unsigned char>(initialPts[i]) < avgThresh)
			continue;
			//debug mode : we display the filtered initial seed points, to be sure we do not filter out validate seed points.
		//	cv::circle(out, initialPts[i], 2, 255, 2);
			


		for (int k = 0; k < len; k++)
			this->lineSearchFilter(input, k, initialPts[i].x, initialPts[i].y, Ur[k][0], Ur[k][1], lineSearchDist, XR[k], YR[k], VR[k], DR[k]);

		// local maxima in all the directions
		std::vector<float>::iterator topVR = std::max_element(VR.begin(), VR.end());
		// Added by Daniel : value test, if the top points is under the threshold value, it is not a top point
		//if (*topVR < 0.1 * 255)
		//	continue;
		float combScore = *topVR;



		//return the number of element between the first maxima and the top maxima (to know the orientation)
		int topR = std::distance(VR.begin(), topVR);

		// We are looking for peaks in 16 directions around the center of the point to find the
		// edges. Here we are doing suppression of peaks around the top one so that slightly off
		// angle edges that fall into two or more bins don't throw us off.
		int squashOnEitherSideOfPeak = 3;  // BCB: Used to be 2
		for (int j = -squashOnEitherSideOfPeak; j <= squashOnEitherSideOfPeak; j++)
		{
			int idx = (topR+j+len) % len;
			VR[idx] = -1e6;
		}

		// Get the other top maxima 
		// 1th rule : the outputs of the right templates in all 16 directions must have two local maxima
		std::vector<float>::iterator topVR2 = std::max_element(VR.begin(), VR.end());
		// Added by Daniel : value test, if the top points is under the threshold value, it is not a top point
		//if (*topVR2 < 0.1 * 255)
		//	continue;
		combScore += *topVR2;
		int topR2 = std::distance(VR.begin(), topVR2);

		float top[2] = {topR, topR2};
		if (topR2 > topR)
		{
			top[0] = topR2;
			top[1] = topR;
		}
		  
		// 2nd rule -> The direction between the local maxima must differ by 180 +/- 22.5 degrees
		// 6th rule -> The sum of the local maxima should exceed the sensitivity threshold (sensThresh)
		int diff = top[0] - top[1];
		if (diff < 7 || diff > 9 || combScore < sensThresh)
			continue;

		cv::Point spt = cv::Point((XR[top[0]] + XR[top[1]])/2, (YR[top[0]]+YR[top[1]])/2); // initialPts[i]


		// save the validated seed point and the two directions
		seedPts.push_back(spt);
		seedDirs.push_back(top[0]);
		seedPts.push_back(spt);
		seedDirs.push_back(top[1]);
	}

	// ================================	//
	//									//
	//			Vein Tracing			//
	//									//
	// ================================ //

	// exclude the points if the maximum pixels detected are under the threshold value
	sensThresh = 10;
	// equilized threshold
	//sensThresh = (10 - MinInput) / (MaxInput - MinInput);

	// resize the cmap before entering the vein tracing step
	// cmap will be used to count how much time a pixel has been detected in a row
	cmap = map.clone();

	for (int i = 0; i < (0 ? 0 : seedPts.size()); i++)
	{
		// reset cmap
		cmap.setTo(0);
		// ith seedPts
		cv::Point2f pt = seedPts[i];
		// the corresponding direction calculated on the previous step
		int o = seedDirs[i];
		// vector containing the vein pts detected for the current seed pts
		std::vector<cv::Point2f> curPts;
		// std::vector<cv::Point> curPts;

		// the limit is fixed to 1000 iterations
		for (int ctr = 0; ctr < 1000; ctr++)
		{
			// U[][] is Ur[][] with a dephasing of Pi/2 (displacment in the orthogonal direction)
			// U[i][0] = cos(2.0*M_PI*i/16.0)
			// U[i][0] = sin(2.0*M_PI*i/16.0)
			// by default, alpha = 2.0, so 
			
			// To reduce the number of points, in order to decrease the processing time, we decide to not draw the middle point.
			//	curPts.push_back(pt - cv::Point2f(U[o][0]*alpha/2, U[o][1]*alpha/2));
			curPts.push_back(pt);

			// the new analysed point
			cv::Point2f npt = pt + cv::Point2f(U[o][0]*alpha, U[o][1]*alpha);

			
			float XL[16] = {0};
			float YL[16] = {0};
			float VL[16] = {0};
			float DL[16] = {0};
			int topL = -1;
			float maxValL = -1e6;
			// find the maximum value on the left side to know the new orientation of the point.
			// We are guessing that the orientation is close to the previous one (the orientation is updated at each iteration)
			for (int k = o-1; k <= o+1; k++)
			{
				int kmod = (k + 16) % 16;
				// Line search in left direction
				this->lineSearchFilter(input, kmod, npt.x, npt.y, Ur[kmod][0], Ur[kmod][1], lineSearchDist, XL[kmod], YL[kmod], VL[kmod], DL[kmod]);
				if (VL[kmod] > maxValL)
				{
					maxValL = VL[kmod];
					topL = kmod;
				}
			}

			float XR[16] = {0};
			float YR[16] = {0};
			float VR[16] = {0};
			float DR[16] = {0};
			int topR = -1;
			float maxValR = -1e6;
			int no = o + 8;
			// find the maximum value on the other direction (same procedure as precedently)
			for (int k = no-1; k <= no+1; k++)
			{
				int kmod = (k + 16) % 16;
				// Line search in right direction
				this->lineSearchFilter(input, kmod, npt.x, npt.y, Ur[kmod][0], Ur[kmod][1], lineSearchDist, XR[kmod], YR[kmod], VR[kmod], DR[kmod]);
				if (VR[kmod] > maxValR)
				{
					maxValR = VR[kmod];
					topR = kmod;
				}
			}

			// the brightest side of the vein is considered to be the more accurate 
			// and is used to set the current orientation (co)
			int co = topR;
			if (maxValL > maxValR)
			{
				co = topL;
			}

			// we only looked at the neighbour orientation, meaning that if the difference between o and co > 4, 
			// there is a dephasing of 180 deg that needs to be corrected to continue to evolve on the same direction. 
			/*if (MAX(o, co) - MIN(o, co) > 4)
			{
				co = (co + 8) % 16;
			}*/

			//We are testing in both direction, to avoid any problems around 0 (distance between 15 and 0 is 1, but 15-0=15>4)
			if (abs(co - o) > 4 && abs((co + 8) % 16 - (o + 8) % 16) > 4)
			{
				co = (co + 8) % 16;
			}

			// calculate the centerline current point (cpt)
			cv::Point2f cpt((XL[topL] + XR[topR])/2, (YL[topL] + YR[topR])/2);

			// if the current point (cpt) is equal to the previous point (pt)
			// we are making it equal to the point used to start the iteration to hope to find maybe a new point
			if (cpt.x == pt.x && cpt.y == pt.y)
			{
				cpt = npt;
				//cpt = npt + cv::Point2f(U[co][0] * alpha * 5, U[co][1] * alpha * 5);
			}

			pt = cpt;

			// increment cmap
			cmap.at<unsigned char>(pt)++;

			int deltao = MAX(topL, topR) - MIN(topL, topR);
			// exit loop requirement :
			// -> map.at<unsigned char>(pt) : the seed point has already be detected
			// -> abs(o - co) > 1 : important orientation changes
			// -> deltao < 7 || deltao > 9 : the vein boundaries are not locally parallel
			// -> maxValL < sensThresh ||  maxValR < sensThresh < sensThresh : The maximum detected values are under the threshold value (too dark)
			// -> pt.x < minEdgeDist : x coordinate too close the the image left border
			// -> pt.x > map.cols - minEdgeDist : idem for the right border
			// -> similar tests are performed for the y coordinate
			// -> cmap.at<unsigned char>(pt) > 2 : if the same pixel has been detected more than twice
			//if (map.at<unsigned char>(pt) || abs(o - co) > 1 || deltao < 7 || deltao > 9 || maxValL < avgThresh || maxValR < avgThresh || pt.x < minEdgeDist || pt.x > map.cols - minEdgeDist || pt.y < minEdgeDist || pt.y > map.rows - minEdgeDist || cmap.at<unsigned char>(pt) > 2)
			if (map.at<unsigned char>(pt) || abs(o - co) > 1 || deltao < 6 || deltao > 10 || maxValL < sensThresh || maxValR < sensThresh || pt.x < minEdgeDist || pt.x > map.cols - minEdgeDist || pt.y < minEdgeDist || pt.y > map.rows - minEdgeDist || cmap.at<unsigned char>(pt) > 2)	
				break;

			o = co;
		}

		// if the vessel is long enough, we keep it
		if (curPts.size() > minPtsPerVein)
		{
			// The points are smoothed with an average smooth method
			avgSmooth(curPts, 9);

			
			for (int i = 0; i < curPts.size(); i++)
			{
				// it sometimes append on the same iteration that the same point is registered on a multiple time
				// it will not change anything for the map, but it can drastically reduce veinPts size
				if (!map.at<unsigned char>(curPts[i]))
				{
					veinPts.push_back(curPts[i]);
					map.at<unsigned char>(curPts[i]) = 255;
				}
			}
			
		}
	}

	int end = timeGetTime();
/*  this drawing is not necessary for the algorithm
	// draw the points on the output image.
	for (int k = 0; k < initialPts.size(); k++)
	{
		cv::circle(out, initialPts[k], 1, CV_RGB(200, 200, 200), 1);
	}

	for (int k = 0; k < seedPts.size(); k++)
	{
		cv::circle(out, seedPts[k], 3, CV_RGB(128, 128, 128), 2);
	}

	for (int i = 0; i < veinPts.size(); i++)
	{
		cv::circle(out, veinPts[i], 1, CV_RGB(50, 50, 50), -1);
	}
	*/
	//Added by Daniel : Save the initial pts, seed pts and vessel pts into vector to be saved in a cvs file at the end of the program
	PushBackDataPts(initialPts, seedPts, veinPts);

	int endTime = timeGetTime();
	int time = endTime - beginTime;

	return 0;
}

void VeinTracer::slidingWindowMinimum(std::vector<unsigned char> &ARR, int K, std::vector<unsigned char> &minArr) 
{
	// pair<int, int> represents the pair (ARR[i], i)
	std::deque<std::pair<int, int>> window;
	for (int i = 0; i < ARR.size(); i++) {
		while (!window.empty() && window.back().first >= ARR[i])
			window.pop_back();
		window.push_back(std::make_pair(ARR[i], i));

		while(window.front().second <= i - K)
			window.pop_front();

		//std::cout << (window.front().first) << ' ';
		minArr[i] = window.front().first;
	}
}


void VeinTracer::slidingWindowMinimumNaive(std::vector<unsigned char> &ARR, int K, std::vector<unsigned char> &minArr)
{
	for (int i = 0; i < ARR.size() - K - 1; i++)
	{
		minArr[i+K/2] = *std::min_element(ARR.begin()+i, ARR.begin()+i+K);
		//for (int j = i-K/2; j < i + K/2; j++)
		//{
		//}
	}
}

// Step 2 : Filtering the initial points detected on step 1 through the coarse grid
//		const cv::Mat &input : image src (input)
//		int templ : iteration number (input) -> used to define the angle orientation
//		int xi : x coordinate of the seed point (input)
//		int yi : y coordinate of the seed point (input)
//		float incx : x orientation (tilt angle) ->  cos(2*pi*k/16+pi/2)
//		float incy : y orientation (tilt angle) -> -sin(2*pi*k/16+pi/2)
//		int dist : maximum line search distance 
//		float &xf : x coordinate of the maxima (output)
//		float &yf : y coordinate of the maxima (output)
//		float &val : maxima value (output)
//		float &d : distance from the seed point (output)
void VeinTracer::lineSearchFilter(const cv::Mat &input, int templ, int xi, int yi, float incx, float incy, int dist, float &xf, float &yf, float &val, float &d)
{
	xf = 0;
	yf = 0;
	d = 0;
	val = 1e6;

	float cumx = 0;
	float cumy = 0;
	float maxCumx = 0;
	float maxCumy = 0;
	float maxVal = -1e6;
	float maxDist = 0;
	
	//while we didn't reach the defined distance
	while (cumx*cumx + cumy*cumy < dist*dist)
	{
		// incx -> cos and incy -> sin => d+= 1;
		d += sqrt(incx*incx + incy*incy);

		float x = cvRound(xi+cumx);
		float y = cvRound(yi+cumy);
		val = extractFilterResponse(input, x, y, templ);

		//store the max value and its position and distance
		if (val > maxVal)
		{
			maxCumx = cumx;
			maxCumy = cumy;
			maxVal = val;
			maxDist = d;
		}

		// increase the position in the defined direction.
		cumx += incx;
		cumy += incy;
	}

	// save the infos from the max value in the output variables
	if (maxVal < 1e6)
	{
		xf = cvRound(xi + maxCumx);
		yf = cvRound(yi + maxCumy);
		val = maxVal;
		d = maxDist;
	}
}

//Added by Daniel : Get the mean value of the matrix to modulate the threshold value
void VeinTracer::GetThresholdValue(const cv::Mat &input, unsigned char &avgThresh ,double Mod)
{
	cv::Scalar Mean = mean(input);
	// save the mean value into the avgList vector
	avgList.push_back(Mean(0));
	// To have a smooth result, we are calculating the average value based on the last 10 images
	if (avgList.size() > 10)
		avgList.erase(avgList.begin());

	double avg;
	for (int i = 0; i < avgList.size(); i++)
		avg += avgList[i];

	avg /= avgList.size();

	avgThresh = Mod*avg;
}

void VeinTracer::GetThresholdMat(const cv::Mat &input, cv::Mat &ThreshMat, unsigned char avgThresh)
{
	ThreshMat = input.clone();
	ThreshMat.setTo(0);
	for (int i = 0; i < input.rows; i++)
		for (int j = 0; j < input.cols; j++)
			if (input.at<unsigned char>(i, j) < avgThresh)
				ThreshMat.at<unsigned char>(i, j) = 255;
}

// Smooth the vein points using the average method
void VeinTracer::avgSmooth(std::vector<cv::Point2f> &pts,const int Span)
{
	std::vector<cv::Point2f> Spts;
	for (int i = 0; i < pts.size(); i++)
	{
		int N = floor(Span/2);

		// to manage the border effects
		// left border
		if (i - N < 0)
			N = i;
		// right border
		if (i + N >= pts.size())
			N = pts.size() - i - 1;

		float Sx = 0;
		float Sy = 0;
		// test if N!=0
		for (int j = -N; j <= N; j++)
		{
			Sx += pts[i + j].x;
			Sy += pts[i + j].y;
		}
		Sx /= (2 * N + 1);
		Sy /= (2 * N + 1);

		// save the new points in Spts
		Spts.push_back(cv::Point2f(Sx, Sy));
	}

	pts = Spts;
}