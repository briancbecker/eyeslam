//#pragma once
//#ifndef WIN32_LEAN_AND_MEAN
//#define WIN32_LEAN_AND_MEAN 1 // Don't include Winsock v1
//#endif

//#include <windows.h>
//#include <stdlib.h>




#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "winmm.lib")

#ifdef _DEBUG
#pragma comment(lib, "opencv_calib3d248d.lib")
#pragma comment(lib, "opencv_contrib248d.lib")
#pragma comment(lib, "opencv_core248d.lib")
#pragma comment(lib, "opencv_features2d248d.lib")
//#pragma comment(lib, "opencv_ffmpeg248.lib")
#pragma comment(lib, "opencv_flann248d.lib")
//#pragma comment(lib, "opencv_haartraining_engine.lib")
#pragma comment(lib, "opencv_highgui248d.lib")
#pragma comment(lib, "opencv_imgproc248d.lib")
#pragma comment(lib, "opencv_legacy248d.lib")
#pragma comment(lib, "opencv_ml248d.lib")
#pragma comment(lib, "opencv_objdetect248d.lib")
#pragma comment(lib, "opencv_ts248d.lib")
#pragma comment(lib, "opencv_video248d.lib")
#pragma comment(lib, "opencv_gpu248d.lib")
#pragma comment(lib, "opencv_nonfree248d.lib")
#pragma comment(lib, "zlibd.lib")
#pragma comment(lib, "libtiffd.lib")
#pragma comment(lib, "libjpegd.lib")
#pragma comment(lib, "libpngd.lib")
#pragma comment(lib, "libjasperd.lib")
#pragma comment(lib, "FlyCapture2.lib")

//#pragma comment(lib, "FlyCapture2GUI_Cd.lib")
#else
#pragma comment(lib, "opencv_nonfree248.lib")
#pragma comment(lib, "opencv_calib3d248.lib")
#pragma comment(lib, "opencv_contrib248.lib")
#pragma comment(lib, "opencv_core248.lib")
#pragma comment(lib, "opencv_features2d248.lib")
//#pragma comment(lib, "opencv_ffmpeg248.lib")
#pragma comment(lib, "opencv_flann248.lib")
//#pragma comment(lib, "opencv_haartraining_engine.lib")
#pragma comment(lib, "opencv_highgui248.lib")
#pragma comment(lib, "opencv_imgproc248.lib")
#pragma comment(lib, "opencv_legacy248.lib")
#pragma comment(lib, "opencv_ml248.lib")
#pragma comment(lib, "opencv_objdetect248.lib")
#pragma comment(lib, "opencv_ts248.lib")
#pragma comment(lib, "opencv_video248.lib")
#pragma comment(lib, "opencv_gpu248.lib")
#pragma comment(lib, "zlib.lib")
#pragma comment(lib, "libtiff.lib")
#pragma comment(lib, "libjpeg.lib")
#pragma comment(lib, "libpng.lib")
#pragma comment(lib, "libjasper.lib")
#pragma comment(lib, "FlyCapture2.lib")

//#pragma comment(lib, "FlyCapture2GUI.lib")
#endif

