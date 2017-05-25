#ifndef VEINTRACER_H
#define VEINTRACER_H

#include <vector>
#include <deque>

#include <math.h>

//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

#include <cv.h>
#include <highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include <Windows.h>

#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif

class VeinTracer
{
public:
	VeinTracer();
	~VeinTracer();

	// input: grayscale image with veins : green channel
	// input2 : value channel from HSV image or red channel from HSV image
	// map: binary output image where 255 => vein, 0 => background
	// out: draws circles and other crap, ignore
	// veinPts => locations of 255 points in map
	int trace(const cv::Mat &input, const cv::Mat &input2, cv::Mat &map, cv::Mat &out, std::vector<cv::Point> &veinPts);

	// Added by Daniel : Get the data points to be stored in a csv file
	void GetDataPts(std::vector<std::vector<cv::Point>> &SavedInitialPts, std::vector<std::vector<cv::Point>> &SavedSeedPts, std::vector<std::vector<cv::Point>> &SavedVeinPts)
	{
		SavedInitialPts = mSavedInitialPts;
		SavedSeedPts = mSavedSeedPts;
		SavedVeinPts = mSavedVeinPts;
	}

private:
	void slidingWindowMinimum(std::vector<unsigned char> &ARR, int K, std::vector<unsigned char> &minArr);
	void slidingWindowMinimumNaive(std::vector<unsigned char> &ARR, int K, std::vector<unsigned char> &minArr);

	int extractFilterResponse(const cv::Mat &input, int x, int y, int filter);

	void lineSearchFilter(const cv::Mat &input, int templ, int xi, int yi, float incx, float incy, int dist, float &xf, float &yf, float &val, float &d);

	//Added by Daniel : get the threshold value to segmented the input image to filter out the false positive seed points located on dark area
	void GetThresholdValue(const cv::Mat &input, unsigned char &avgThresh, double Mod);
	void GetThresholdMat(const cv::Mat &input, cv::Mat &ThreshMat, unsigned char avgThresh);

	void avgSmooth(std::vector<cv::Point2f> &pts, int span);

	float U[16][2];
	float Ur[16][2];

	//Added by Daniel : list of the mean value of the value channel (or green channel) from HSV image (RGB image)
	std::vector<double> avgList;
	std::vector<double> avgValList;

	//Added by Daniel : extract the data collected on the image -> initial pts, seed pts and vein pts
	std::vector<std::vector<cv::Point>> mSavedInitialPts, mSavedSeedPts, mSavedVeinPts;
	//push back new data into the vector. Used at the end of each frame detection programe
	void PushBackDataPts(std::vector<cv::Point> initialPts, std::vector<cv::Point> seedPts, std::vector<cv::Point> veinPts)
	{
		mSavedInitialPts.push_back(initialPts);
		mSavedSeedPts.push_back(seedPts);
		mSavedVeinPts.push_back(veinPts);
	}

	// Mat used in the vessel tracing step to limit the infinit loop.
	// Each time a pixel is detected, the corresponding value is incremented.
	// Then we test the value to be sure that it does not exceed a threshold value (like 2, 3 or 4)
	cv::Mat cmap;
};


#endif //VEINTRACER_H