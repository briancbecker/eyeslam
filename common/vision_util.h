#pragma once
#include "stdafx.h"
#include <deque>
#include <cv.hpp>
#include "mutex.h"
//extern std::deque<CString> *appStrResult;
//extern bool updateStrResult;
//
//extern void _PrintResult(CString str);
//extern void _PrintResult (const char * format, ... );
//extern void _PrintMat (cv::Mat mat);


std::deque<CString> *appStrResult;
bool updateStrResult;
Mutex gMutex;

void _PrintResult(CString str);
void _PrintResult (const char * format, ... );
void _PrintMat (cv::Mat mat);

