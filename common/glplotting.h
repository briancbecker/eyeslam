////////////////////////////////////////////////////////////////////////////////
///
///   Filename: glplotting.h
/// 
///   Copyright (C) 2011-2012   Brian C. Becker             www.BrianCBecker.com
///   License: LGPL             RI @ CMU                          www.ri.cmu.edu
///                             Medical Instrumentation Lab       Micron Project
///
///   Description: A quick & dirty OpenGL class to do MATLAB-style plotting
///   --------------------------------------------------------------------------
///	  Sometimes in real-time C++ systems you want to do plotting on the fly
///	  without having to save the data out to a file and fire up MATLAB. This
///	  class aims to provide a quick drop-in replacement for exactly that
///	  scenario. 
///	  
///	  Note: we use freeglut instead of glut because is more thread safe
///
////////////////////////////////////////////////////////////////////////////////

#ifndef GLPLOTTING_H
#define GLPLOTTING_H

// We want to use static libraries as much as possible
//#define FREEGLUT_STATIC

#include <gl/freeglut.h>
#include <stdio.h>
#include <math.h>
#include "thread.h"
#include "queue.h"
#include "mutex.h"
#include "macros.h"
#include "zstring.h"

////////////////////////////////////////////////////////////////////////////////
///   \def GLP_MAX_WINDOWS
///   \brief Maximum number of supported windows (figures in MATLAB)
////////////////////////////////////////////////////////////////////////////////
#define GLP_MAX_WINDOWS 64

////////////////////////////////////////////////////////////////////////////////
///   \def GLP_MAX_STRLEN
///   \brief Maximum length for strings like titles and axis labels
////////////////////////////////////////////////////////////////////////////////
#define GLP_MAX_STRLEN 1024

////////////////////////////////////////////////////////////////////////////////
///   \def GLP_MAX_GRAPHS
///   \brief Maximum number of individual graphs in a figure (series in MATLAB)
////////////////////////////////////////////////////////////////////////////////
#define GLP_MAX_GRAPHS 16

// Various color macro defines
#define TO_RGB(r,g,b) ( ((r << (32-8*1)) & 0xFF000000) | ((g << (32-8*2)) & 0x00FF0000) | ((b << (32-8*3)) & 0x0000FF00) )
#define RR(x) (((x & 0xFF000000) >> (32-8*1)) / 255.0f)
#define GG(x) (((x & 0x00FF0000) >> (32-8*2)) / 255.0f)
#define BB(x) (((x & 0x0000FF00) >> (32-8*3)) / 255.0f)

// If you want to customize the windows, here is where you would do so
#define GLP_WINDOW_SIZE                 1000
#define GLP_TITLE_POS                   50
#define GLP_XOFF                        120
#define GLP_COLOR_BACK                  0xCCCCCC00
#define GLP_COLOR_GRAPH                 0xFFFFFF00


////////////////////////////////////////////////////////////////////////////////
///   
///   \class GlutWindow
///   \brief The OpenGL window for each figure
///   
///   GlutWindow serves as the OpenGL basis for each figure. It contains
///   low-level data for axis, title, and series data 
///   
////////////////////////////////////////////////////////////////////////////////
class GlutWindow
{
public:
	GlutWindow() { width = height = x = y = 0; num = id = -1; *windowName = 0; *title = 0; xstart = 0; xstop = 0; ylow = yhigh = 0; xinc = yinc = 0; paused = 0;}
	int width, height, x, y;
	int num, id;
	char windowName[GLP_MAX_STRLEN];
	char title[GLP_MAX_STRLEN];
	Queue<float> data[GLP_MAX_GRAPHS];
	int colors[GLP_MAX_GRAPHS];
	int xstart, xstop;
	float ylow, yhigh;
	float xinc, yinc;
	bool paused;

	GlutWindow &operator =(const GlutWindow &rhs)
	{
		if (this != &rhs)
		{
			this->width = rhs.width;
			this->height = rhs.height;
			this->num = rhs.num;
			strcpy(this->windowName, rhs.windowName);
			strcpy(this->title, rhs.title);
			this->id = rhs.id;
			for (int i = 0; i < GLP_MAX_GRAPHS; i++)
			{
				this->data[i] = rhs.data[i];
				this->colors[i] = rhs.colors[i];
			}
			this->xstart = rhs.xstart;
			this->xstop = rhs.xstop;
			this->ylow = rhs.ylow;
			this->yhigh = rhs.yhigh;
			this->xinc = rhs.xinc;
			this->yinc = rhs.yinc;
			this->paused = rhs.paused;
		}
		return *this;
	}
};

////////////////////////////////////////////////////////////////////////////////
///   
///   \class GLPlotting
///   \brief A MATLAB like interface to creating figures, axis, and plots
///   
///   There should be only one GLPlotting object, which runs in its own thread
///   and can serve up to GLP_MAX_WINDOWS individual figures, each with
///   GLP_MAX_GRAPHS data series. It has very MATLAB-like syntax. The figure(X)
///   function creates or selects a specific figure. The functions title, ylim,
///   xlim, and grid should be very familiar to any MATLAB users. Since this is
///   a real-time plot, new data is added and plotted by calling addPoint. You
///   can achieve a MATLAB's hold on functional by specifying individual values
///   for graph parameters of color and addPoint. 
///   
///   You can pause real-time plotting by hitting the space bar. GLPlotting will
///   adaptively find y min/max if both parameters to ylim are the same
///   
///   An example:
///   
///   %% MATLAB:
///   figure(1); 
///   plot(sin(0:0.01:10), 'Color', [1 0 0])
///   hold on
///   plot(cos(0:0.01:10), 'Color', [0 1 0])
///   xlim([0 1000]);
///   ylim([-1 1]);
///   
///   // GLPlotting:
///   plot.figure(1);
///   plot.xlim(0.0f, 1000.0f);
///   plot.ylim(-1.0f, 1.0f);
///   plot.color(1, TO_RGB(255, 0, 0));
///   for (float val = 0.0f; val < 10.0f; val += 0.01f) 
///	      plot.addPoint(1, sin(val));
///   plot.color(2, TO_RGB(0, 255, 0));
///   for (float val = 0.0f; val < 10.0f; val += 0.01f) 
///	      plot.addPoint(2, cos(val));
///   
////////////////////////////////////////////////////////////////////////////////
class GLPlotting : protected Thread
{
public:
	GLPlotting();
	~GLPlotting();

	static int figure(int num, int width = 800, int height = 600, int x = 10, int y = 50);
	static int title(const char *str);
	static int ylim(float ylow = 0, float yhigh = 0);
	static int xlim(int length);
	static int xlim(int xstart, int xstop);
	static int color(int graph, int color);
	static int grid(float yinc);

	int addPoint(int graph, float point);

protected:
	void init();
	void destroy();

	virtual void execute();
	virtual void cleanup();

	static void draw();
	static void keyboard(unsigned char key, int x, int y);

	static int mInit;
	static int mRun;
	static Queue<GlutWindow> mCreateWindows;
	static Mutex mMutex;
	static Mutex mQMutex;

	static GlutWindow mWindows[GLP_MAX_WINDOWS];
	static int mWinIDs[GLP_MAX_WINDOWS];

	static int mCurrent;
	static int mMax;

	Array<float> mTmpQueue[GLP_MAX_WINDOWS][GLP_MAX_GRAPHS];
	static bool mPaused[GLP_MAX_WINDOWS];
};

#endif