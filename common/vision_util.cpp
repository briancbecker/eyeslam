//#include "def_MicronVision.h"

#include "stdafx.h"
#include "vision_util.h"
#include "def_MicronVision.h"


//#define MAX_LINE_PRINT 30

void _PrintResult(CString str){
	
	gMutex.enter();
	appStrResult->push_back(str);

	while(appStrResult->size() > MAX_LINE_PRINT)
		appStrResult->pop_front();

	updateStrResult = true;
		
	gMutex.leave();
}



void _PrintResult (const char * format,...)
{
	gMutex.enter();
	va_list args;
	va_start(args, format);

	char str[256];
	vsprintf (str,format, args);
	CString cstr(str);

	gMutex.leave();

	 _PrintResult(cstr);


}

void _PrintMat (cv::Mat mat)
{
	gMutex.enter();
	int nCols = mat.cols;
	int nRows = mat.rows;
	cv::Mat matC;
	mat.convertTo(matC, CV_64F);

	gMutex.leave();

	for(int i=0 ; i < nRows ; i++)
	{
		CString str;
		str.Format(_T("%d: "),i); 
		for(int j=0; j < nCols; j++)
		{
			CString str2;
			str2.Format(_T("%.6f "), matC.at<double>(i,j));
			str.Append(str2);
		}
		 _PrintResult(str);
	}



}