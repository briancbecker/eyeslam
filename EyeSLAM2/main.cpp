#include <stdio.h>
#include <direct.h>
#include <stdlib.h>
#include <string>

#include "veintracer.h"
#include "fileio.h"
#include "eyeslam.h"
#include "fpstracker.h"

//#define VEIN_PATH "D:\\EyeSLAM\\Sungwook\\Testing\\Static_With_Fiducials\\Large\\"
//#define VEIN_PATH "D:\\EyeSLAM\\videos\\Thu May 16 17_11_00 2013_cool.mp4"
//#define VEIN_PATH "D:\\EyeSLAM\\05162013_PigEye\\eye2\\Thu May 16 17_16_52 2013__reallygood\\video_xvid_combined.mp4"
//#define VEIN_PATH "D:\\GDrive\\EyeSLAM_shared\\Videos\\\Youtube\\Best\\23G VITRECTOMY - ENDOLASER FOR PROLIFERATIVE DIABETIC RETINOPATHY-01.mp4"

//#define VEIN_PATH "D:\\EyeSLAM\\Sungwook\\dynamic_with_fiducials.mp4"

//#define VEIN_PATH "F:\\Videos\\ours\\dynamic_with_fiducials_Large.mp4"
#define VEIN_PATH "F:\\Videos\\ours\\good\\Thu May 16 17_11_00 2013_cool_snip1.avi"

//#define VIDEO_TIMING "D:\\GDrive\\EyeSLAM_shared\\Results\\2015_08_27_TimingInfo_v4.csv"
#define VIDEO_TIMING "D:\\GDrive\\Journal\\EyeSLAM2016_IJMRCAS\\analysis\\2017_01_28_TimingInfo_v5_highres.csv"

#define READ_VIDEO 1

#define TEST_VEIN_TRACER 0

#define SAVE_VIDEO 0

#define DRAW_TEXT 0

void rotate(cv::Mat& src, double angle, cv::Mat& dst)
{
    int len = std::max(src.cols, src.rows);
    cv::Point2f pt(len/2., len/2.);
    cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);

    cv::warpAffine(src, dst, r, cv::Size(len, len));
}

void set_label(cv::Mat& im, cv::Rect r, const std::string label)
{
#if DRAW_TEXT
    int fontface = cv::FONT_HERSHEY_PLAIN;
    double scale = 1;
    int thickness = 1;
    int baseline = 0;

    cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
    cv::Point pt(r.x + (r.width-text.width)/2, r.y + (r.height+text.height)/2);

    cv::rectangle(
        im, 
        pt + cv::Point(0, baseline), 
        pt + cv::Point(text.width, -text.height), 
        CV_RGB(0,0,0), CV_FILLED
    );

    cv::putText(im, label, pt + cv::Point(0, 2), fontface, scale, CV_RGB(255,255,255), thickness, 8);
#endif
}

void TimingInfoCSVFile(std::vector<TimingInfo> vecTimingInfo)
{
	printf("Writing timing info to csv file: %s!\n", VIDEO_TIMING);
	// Date
	// current date/time based on current system
	time_t now = time(0);

	// convert now to string form
	std::string dt = ctime(&now);

	std::ofstream myfile;
	myfile.open(VIDEO_TIMING);

	myfile << "Timing info from the video " << VEIN_PATH << "\n";
	std::cout << "Timing info from the video " << VEIN_PATH << "\n";
	myfile << "EyeSLAMTime, veinDetectionAlgo, scanMatchingAlgo, lowResScanMatching, highResScanMatching\n";
	// for each frame
	for (int i = 0; i < vecTimingInfo.size(); i++)
	{ 
		myfile << vecTimingInfo[i].eyeSLAMTime << "," << vecTimingInfo[i].veinDetectionAlgo << "," << vecTimingInfo[i].ScanMatchingAlgo << ",";
		myfile << vecTimingInfo[i].lowResScanMatching << "," << vecTimingInfo[i].highResScanMatching << "\n";
	}

	myfile.close();
}

void SplitFilename(const std::string& str, std::string &path, std::string &filename)
{
  std::size_t found = str.find_last_of("/\\");
  path = str.substr(0,found);
  filename = str.substr(found+1);
}

int main(int argc, char *argv[])
{
	timeBeginPeriod(1);

	std::string filename = VEIN_PATH;
	bool saveVideo = SAVE_VIDEO;
	std::string outputFilename = "";
	std::string postfix = "";
	bool wholeVideo = false;
	if (argc == 3)
	{
		printf("%s %s\n", argv[1], argv[2]);
		postfix = argv[1];
		filename = argv[2];
		FILE *fp = fopen(filename.c_str(), "rb");
		if (!fp)
		{
			printf("Error, could not open %s!\n", filename.c_str());
			exit(-1);
		}
		fclose(fp);
		saveVideo = true; // Always save video when providing a video to run. 
		std::string path, file;
		SplitFilename(filename, path, file);
		std::string outputPath = path + "/" + argv[1];
		_mkdir(outputPath.c_str());
		outputFilename = path + "/" + argv[1] + "/" + file + ".output.avi";
		if (argv[1][0] == 'y')
		{
			wholeVideo = true;
		}
	}	

	std::vector<cv::Point> veinPts;
	VeinTracer tracer;
	std::vector<TimingInfo> all_timing_info;

	cv::VideoWriter outputVideo;

#if TEST_VEIN_TRACER
	cv::Mat veinMap, veinOut;
	std::vector<cv::Mat> channels;

	cv::Mat img;
#if 0
	cv::Mat oimg = cv::imread("D:\\EyeSLAM\\images\\test_45_gray.jpg");
	rotate(oimg, 00, img);
#else
	cv::Mat oimg = cv::imread("D:\\EyeSLAM\\images\\test_img.jpg");
	rotate(oimg, 00, img);
#endif
	cv::split(img, channels);
	cv::Mat gimg = channels[1];
	//cv::equalizeHist(gimg, gimg);
	veinOut = gimg;
	tracer.trace(gimg, veinMap, veinOut, veinPts);
	cv::imshow("veinOut", veinOut);
	cv::imshow("gimg", gimg);
	cv::imshow("inImg", img);
	cv::waitKey(-1);
	return 0;
#endif

#if READ_VIDEO
	cv::VideoCapture inputVideo;
	inputVideo.open(filename);
#else
	Array<String> files;
	FileIO::scanDirForFiles("*.jpg", files, filename);
#endif
	EyeSLAM eyeSLAM;

	eyeSLAM.start();

#if READ_VIDEO
	int counter = 0;
	bool openedOutputVideo = false;
	while (1) 
	{
		counter++;
		if (counter > 1e6)
			break;

		cv::Mat imgBig2X;
		inputVideo >> imgBig2X;

		cv::Rect roi(0, 0, imgBig2X.cols / 2, imgBig2X.rows);
		//fprintf(stderr, "Img size: %d %d\n"

		cv::Mat imgBig = imgBig2X(roi);

		if (imgBig.rows == 0)
			break;
#else
	for (int i = 0; i < files.size(); i++)
	{
		printf("Processing image %s...\n", files[i].c_str());
		//cv::Mat imgBig = cv::imread("testimg1g.jpg", 0), img;
		//cv::Mat dispBig = cv::imread("testimg1.jpg"), disp;
		cv::Mat imgBig2X = cv::imread(files[i]);
		cv::Mat imgBig = imgBig2X(cv::Rect(0, 0, imgBig2X.cols / 2, imgBig2X.rows));
#endif
		cv::Mat img, occMap;

		//printf("Size: %d %d\n", imgBig2X.cols, imgBig2X.rows);
		cv::resize(imgBig, img, cv::Size(imgBig.cols/2, imgBig.rows/2));
		//img = imgBig;
		//cv::resize(dispBig, disp, cv::Size(imgBig.cols/2, imgBig.rows/2));

		//cv::Mat map(img.size(), CV_8UC1);
		//tracer.trace(img, map, disp, veinPts);
		//cv::imshow("vein", disp);
		//cv::waitKey(1000);
		// 

		if (wholeVideo)
		{
			img = imgBig2X;
			//cv::resize(imgBig2X, img, cv::Size(imgBig2X.cols/2, imgBig2X.rows/2));
		}

		eyeSLAM.slamIt(img);

		while (eyeSLAM.getQueueLength() || eyeSLAM.isDry() == false)
			Sleep(1);

		eyeSLAM.getGaussianOccMap(occMap);
		if (occMap.rows)
		{
			cv::Mat finalImage(cv::Size(img.cols*2, img.rows*2), CV_8UC3);
			finalImage.setTo(0);

			cv::Mat colorOccMap;
			cv::cvtColor(occMap, colorOccMap, CV_GRAY2BGR);

			std::vector<cv::Point2f> borderPoints = eyeSLAM.getBorderPoints();
			for (int i = 0; i < borderPoints.size(); i++)
			{
				cv::line(colorOccMap, borderPoints[i], borderPoints[(i + 1) % borderPoints.size()], CV_RGB(255, 255, 0), 2);
			}

			cv::imshow("occMap", colorOccMap);
			cv::imshow("img", img);
			
			// draw detected vessels points from the map 
			cv::Mat veinMap;
			std::vector<cv::Point> veinPts;
			std::vector<cv::Point2f> mapPts;

			img.copyTo(veinMap);
			eyeSLAM.getVeinPts(veinPts);
			if (veinPts.size())
			{
				for (int i = 0; i < veinPts.size(); i++)
					cv::circle(veinMap, veinPts.at(i), 1, CV_RGB(0, 255, 255), -1);

				cv::imshow("Output vessel detection", veinMap);
			}

			cv::Mat slamMap;
			eyeSLAM.getVeinMap(mapPts);
			img.copyTo(slamMap);
			if (mapPts.size())
			{
				for (int i = 0; i < mapPts.size(); i++)
					cv::circle(slamMap, mapPts.at(i), 1, CV_RGB(0, 255, 0), -1);

				cv::imshow("slamMap", slamMap);
			}

			cv::Mat Display;
			eyeSLAM.GetmDisplay(Display);
			if (Display.rows*Display.cols)
				cv::imshow("mDispMap", Display);

			double scale = std::max((double)occMap.cols / img.cols, (double)occMap.rows / img.rows);

			cv::Mat colorOccMapSq(cv::Size(ceil(img.cols*scale), ceil(img.rows*scale)), CV_8UC3);
			colorOccMapSq.setTo(0);
			cv::Mat colorOccMapSqROI(colorOccMapSq, cv::Rect(0, 0, occMap.cols, occMap.rows));
			printf("%d %d copying into %d %d\n", occMap.cols, occMap.rows, colorOccMapSq.cols, colorOccMapSq.rows);
			colorOccMap.copyTo(colorOccMapSqROI);
			cv::imshow("colorOccMapSq", colorOccMapSq);
			cv::Mat colorOccMapDisp(cv::Size(img.cols, img.rows), CV_8UC3);
			cv::resize(colorOccMapSq, colorOccMapDisp, cv::Size(img.cols, img.rows));

			cv::Mat upperLeftImg(finalImage, cv::Rect(0, 0, img.cols, img.rows));
			cv::Mat upperRightImg(finalImage, cv::Rect(img.cols, 0, img.cols, img.rows));
			cv::Mat lowerLeftImg(finalImage, cv::Rect(0, img.rows, img.cols, img.rows));
			cv::Mat lowerRightImg(finalImage, cv::Rect(img.cols, img.rows, img.cols, img.rows));
			img.copyTo(upperLeftImg);
			veinMap.copyTo(upperRightImg);
			colorOccMapDisp.copyTo(lowerLeftImg);
			slamMap.copyTo(lowerRightImg);


			static FPSTracker fps(100);
			char text[10240];
			sprintf(text, "Frame: %d: %.1f fps", counter, fps.click());
			int x = 160+img.cols;
			int y = 260+img.rows;
			set_label(finalImage, cv::Rect(x, y, img.cols*2 - x, img.rows*2 - y), text);

			set_label(upperLeftImg, cv::Rect(img.cols / 2 - 50, 5, 100, 20), "Raw Video");
			set_label(upperRightImg, cv::Rect(img.cols / 2 - 50, 5, 100, 20), "Vessel Oservations");
			set_label(lowerLeftImg, cv::Rect(img.cols / 2 - 50, 5, 100, 20), "Occupancy Map");
			set_label(lowerRightImg, cv::Rect(img.cols / 2 - 50, 5, 100, 20), "EyeSLAM Output");

			cv::imshow("finalImage", finalImage);

			if (saveVideo)
			{
				if (!openedOutputVideo)
				{
					openedOutputVideo = true;
					printf("output: %s\n", outputFilename.c_str());
					outputVideo.open(outputFilename, CV_FOURCC('X', 'V', 'I', 'D'), 30, cv::Size(finalImage.cols, finalImage.rows), true);
				}
				outputVideo.write(finalImage);
			}
		}

		if (counter > 100e10)
		{
			cv::waitKey(-1);
		}
		else
		{
			cv::waitKey(1);
		}

		TimingInfo oneFrameTimingInfo;
		eyeSLAM.GetTiming(oneFrameTimingInfo);

		all_timing_info.push_back(oneFrameTimingInfo);
	}

	TimingInfoCSVFile(all_timing_info);

	return 0;
}

#if _DEBUG
#pragma comment(lib, "opencv_calib3d249d.lib")
#pragma comment(lib, "opencv_contrib249d.lib")
#pragma comment(lib, "opencv_core249d.lib")
#pragma comment(lib, "opencv_features2d249d.lib")
#pragma comment(lib, "opencv_flann249d.lib")
//#pragma comment(lib, "opencv_gpu249d.lib")
#pragma comment(lib, "opencv_highgui249d.lib")
#pragma comment(lib, "opencv_imgproc249d.lib")
#pragma comment(lib, "opencv_legacy249d.lib")
#pragma comment(lib, "opencv_ml249d.lib")
#pragma comment(lib, "opencv_nonfree249d.lib")
#pragma comment(lib, "opencv_objdetect249d.lib")
#pragma comment(lib, "opencv_photo249d.lib")
#pragma comment(lib, "opencv_stitching249d.lib")
#pragma comment(lib, "opencv_ts249d.lib")
#pragma comment(lib, "opencv_video249d.lib")
#pragma comment(lib, "opencv_videostab249d.lib")
#else
#pragma comment(lib, "opencv_calib3d249.lib")
#pragma comment(lib, "opencv_contrib249.lib")
#pragma comment(lib, "opencv_core249.lib")
#pragma comment(lib, "opencv_features2d249.lib")
#pragma comment(lib, "opencv_flann249.lib")
#pragma comment(lib, "opencv_gpu249.lib")
#pragma comment(lib, "opencv_highgui249.lib")
#pragma comment(lib, "opencv_imgproc249.lib")
#pragma comment(lib, "opencv_legacy249.lib")
#pragma comment(lib, "opencv_ml249.lib")
#pragma comment(lib, "opencv_nonfree249.lib")
#pragma comment(lib, "opencv_objdetect249.lib")
#pragma comment(lib, "opencv_photo249.lib")
#pragma comment(lib, "opencv_stitching249.lib")
#pragma comment(lib, "opencv_ts249.lib")
#pragma comment(lib, "opencv_video249.lib")
#pragma comment(lib, "opencv_videostab249.lib")
#endif