#pragma once
#include <cv.hpp>
#include <opencv2/core/core.hpp>
#include <queue>
#include <deque>

#include <macros.h>
#include <MMSystem.h>

#define SC_CAMERA 0
#define SC_VIDEO 1
#define SC_IMAGE 2

#define THREAD_UPDATE (WM_USER + 1)
#define MY_DEBUG 0
#define WND_MAIN "Main Frame"

#define USE_PCL 0
#define MAX_BLOB_SIZE 500
#define MIN_BLOB_SIZE 250
#define MAX_DEPTH_LIMIT 2000
#define DEFAULT_DEPTH_LIMIT 500


#define MAX_LINE_PRINT 30



#define CALIB_LEFT_FILE                 "files\\Mln.txt"
#define CALIB_RIGHT_FILE                "files\\Mrn.txt"

//namespace globals
//{
//extern std::deque<CString> *appStrResult;
//extern void _PrintResult(CString str);
extern void _PrintResult (const char * format, ... );
extern void _PrintMat (cv::Mat mat);

//}





struct _FRAME{
	cv::Mat img;
	UINT nFrameNum;

};

struct _FRAME_INFO{

	long nTotalFrame;
	long nCurFrame;
	bool bEoF;
	int nSourceType; /*SC_CAMERA = 0, SC_VIDEO = 1, and SC_IMAGE = 2*/
	double fps;

};


//struct _FRAMEInfo{
//	UINT nTotalFrame;
//	UINT curFrame;
//	bool bEoF;
//
//	int sourceType; /*SC_CAMERA = 0, SC_IMAGE = 1, and SC_VIDEO = 2*/
//	double fps;
//
//};
//
//
//
//
//class CFrameInfo
//{
//public:
//	_FRAMEInfo *m_pFrameInfo;
//	int m_nSourceType;
//	bool m_genInfo;
//
//public:
//	CFrameInfo(void){
//		m_pFrameInfo = NULL;
//		m_nSourceType = -1;
//		m_genInfo = false;
//	};
//	
//	CFrameInfo(_FRAMEInfo *pInfo){
//		m_pFrameInfo = pInfo;
//		m_genInfo = false;
//	
//	};
//	void gen(void){
//		m_pFrameInfo = new _FRAMEInfo;
//		m_pFrameInfo->bEoF = false;
//		m_pFrameInfo->nTotalFrame = 0;
//		m_pFrameInfo->curFrame = -1;
//		m_pFrameInfo->sourceType = -1;
//		m_pFrameInfo->fps = 0.0;
//		m_genInfo = true;
//	}
//
//	~CFrameInfo(void)
//	{
//		if(m_genInfo)
//			delete m_pFrameInfo;
//	};
//
//};
//
