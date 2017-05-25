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

#include "glplotting.h"

int GLPlotting::mInit = 0;
int GLPlotting::mRun = 0;
int GLPlotting::mCurrent = -1;
int GLPlotting::mMax = -1;
int GLPlotting::mWinIDs[GLP_MAX_WINDOWS] = {-1};
GlutWindow GLPlotting::mWindows[GLP_MAX_WINDOWS];
bool GLPlotting::mPaused[GLP_MAX_WINDOWS] = {0};
Queue<GlutWindow> GLPlotting::mCreateWindows;
Mutex GLPlotting::mMutex;
Mutex GLPlotting::mQMutex;

GLPlotting::GLPlotting()
{
	init();
}

GLPlotting::~GLPlotting()
{
	destroy();
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Glut function invoked on keypress, used to pause/resume plotting
///
///   If the space bar is pressed on a figure, the figure is paused. This means
///   that all new data is discarded and only the current data is replotted.
///   Resuming a paused figure causes a discontinuity in the plot.
///   
////////////////////////////////////////////////////////////////////////////////
void GLPlotting::keyboard(unsigned char key, int x, int y)
{
	if (key != 32)
		return;

	int id = glutGetWindow();
	GlutWindow *win = 0;
	for (int i = 0; i <= mMax; i++)
	{
		if (mWindows[i].id == id)
		{
			win = &mWindows[i];
			break;
		}
	}

	// If we didn't find an associated glut window, then exit
	if (!win)
		return;

	win->paused = !win->paused;
	mPaused[win->num] = win->paused;
}

void GLPlotting::draw()
{

	// Get associated glut window that is being updated
	int id = glutGetWindow();
	GlutWindow *win = 0;
	for (int i = 0; i <= mMax; i++)
	{
		if (mWindows[i].id == id)
		{
			win = &mWindows[i];
			break;
		}
	}

	// If we didn't find an associated glut window, then exit
	if (!win)
		return;

	// Setup the graph for the data
	glClear(GL_COLOR_BUFFER_BIT);

	glBegin(GL_QUADS);
	glColor3f(RR(GLP_COLOR_GRAPH),GG(GLP_COLOR_GRAPH),BB(GLP_COLOR_GRAPH));
	glVertex2f(GLP_XOFF,GLP_TITLE_POS*2);
	glVertex2f(GLP_XOFF,GLP_WINDOW_SIZE-GLP_TITLE_POS);
	glVertex2f(GLP_WINDOW_SIZE-GLP_TITLE_POS,GLP_WINDOW_SIZE-GLP_TITLE_POS);
	glVertex2f(GLP_WINDOW_SIZE-GLP_TITLE_POS,GLP_TITLE_POS*2);
	glVertex2f(GLP_XOFF,GLP_TITLE_POS*2);
	glEnd();

	glBegin(GL_LINE_STRIP);
	glColor3f(0,0,0);
	glVertex2f(GLP_XOFF,GLP_TITLE_POS*2);
	glVertex2f(GLP_XOFF,GLP_WINDOW_SIZE-GLP_TITLE_POS);
	glVertex2f(GLP_WINDOW_SIZE-GLP_TITLE_POS,GLP_WINDOW_SIZE-GLP_TITLE_POS);
	glVertex2f(GLP_WINDOW_SIZE-GLP_TITLE_POS,GLP_TITLE_POS*2);
	glVertex2f(GLP_XOFF,GLP_TITLE_POS*2);
	glEnd();

	// Setup offsets
	float x = GLP_XOFF;
	float y = GLP_TITLE_POS*2;
	float height = GLP_WINDOW_SIZE - GLP_TITLE_POS*2 - GLP_TITLE_POS;
	float width = GLP_WINDOW_SIZE - GLP_TITLE_POS - x;

	int j = 0;
	float xlen = win->xstop - win->xstart;
	float minVal = 1e9, maxVal = -1e9;

	float ylow = win->ylow;
	float yhigh = win->yhigh;

	// Adaptively find the min & max of the y axis
	if (ylow == yhigh)
	{
		for (int j = 0; j < GLP_MAX_GRAPHS; j++)
		{
			for (int k = win->xstart; k < win->data[j].size(); k++)
			{
				float val = 0 ;
				win->data[j].get(k, val);
				if (val < minVal)
					minVal = val;
				if (val > maxVal)
					maxVal = val;
			}
		}

		ylow = minVal;
		yhigh = maxVal;
	}

	// Plot y axis labels
	float yinc = win->yinc;
	if (yinc > 0)
	{
		for (float hline = ylow + yinc; hline < yhigh; hline += yinc)
		{
			float ypt = hline;
			ypt = (ypt - ylow) / (yhigh - ylow);
			ypt = height - (ypt * height) + y;

			glEnable (GL_LINE_STIPPLE);
			glLineWidth (1.0);
			glColor3f(0.8f, 0.8f, 0.8f); 
			glLineStipple(5, 0xAAAA);
			glBegin(GL_LINES);
			glVertex2f(x, ypt);
			glVertex2f(x+width, ypt);
			glEnd();
			glDisable(GL_LINE_STIPPLE);

			char text[256];
			float length, stand;
			//sprintf(text, "%0.4e", hline);
			sprintf(text, "%.2f", hline);
			length = glutBitmapLength(GLUT_BITMAP_HELVETICA_12, (const unsigned char *)text);
			stand = glutBitmapHeight(GLUT_BITMAP_HELVETICA_12);
			glColor3f(0,0,0);
			glRasterPos2f(GLP_XOFF - length - 30, ypt + stand/2);
			for (int i = 0; i < strlen(text); i++)
				glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, text[i]);
		}
	}

	// Plot data for each graph with the associated color
	if (xlen > 0)
	{
		for (int j = 0; j < GLP_MAX_GRAPHS; j++)
		{
			if (win->xstart < win->data[j].size())
			{
				glBegin(GL_LINE_STRIP);
				glColor3f(RR(win->colors[j]),GG(win->colors[j]),BB(win->colors[j])); 
				float ctr = 0;
				for (int k = win->xstart; k < win->data[j].size(); k++)
				{
					float ypt = 0;
					win->data[j].get(k, ypt);
					ypt = (ypt - ylow) / (yhigh - ylow);
					ypt = height - (ypt * height) + y;
					glVertex2f(x + ctr/xlen*width, ypt);
					ctr += 1.0f;
				}
				glEnd();
			}
		}
	}

	// Title y-axis with min & max
	char text[256];
	float length, stand;
	//sprintf(text, "%0.4e", yhigh);
	sprintf(text, "%.2f", yhigh);
	length = glutBitmapLength(GLUT_BITMAP_HELVETICA_12, (const unsigned char *)text);
	stand = glutBitmapHeight(GLUT_BITMAP_HELVETICA_12);
	glColor3f(0,0,0);
	glRasterPos2f(GLP_XOFF - length - 30, GLP_TITLE_POS*2 + stand/2);
	for (int i = 0; i < strlen(text); i++)
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, text[i]);

	//sprintf(text, "%0.4e", ylow);
	sprintf(text, "%.2f", ylow);
	length = glutBitmapLength(GLUT_BITMAP_HELVETICA_12, (const unsigned char *)text);
	stand = glutBitmapHeight(GLUT_BITMAP_HELVETICA_12);
	glColor3f(0,0,0);
	glRasterPos2f(GLP_XOFF - length - 30, GLP_TITLE_POS*2 + height + stand/2);
	for (int i = 0; i < strlen(text); i++)
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, text[i]);

	// Title x-axis
	sprintf(text, "%d", win->xstart);
	length = glutBitmapLength(GLUT_BITMAP_HELVETICA_12, (const unsigned char *)text);
	stand = glutBitmapHeight(GLUT_BITMAP_HELVETICA_12);
	glColor3f(0,0,0);
	glRasterPos2f(GLP_XOFF - length/2, GLP_TITLE_POS*2 + height + stand + 10);
	for (int i = 0; i < strlen(text); i++)
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, text[i]);

	sprintf(text, "%d", win->xstop);
	length = glutBitmapLength(GLUT_BITMAP_HELVETICA_12, (const unsigned char *)text);
	stand = glutBitmapHeight(GLUT_BITMAP_HELVETICA_12);
	glColor3f(0,0,0);
	glRasterPos2f(GLP_XOFF - length/2 + width, GLP_TITLE_POS*2 + height + stand + 10);
	for (int i = 0; i < strlen(text); i++)
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, text[i]);

	// Title the graph
	length = glutBitmapLength(GLUT_BITMAP_TIMES_ROMAN_24, (const unsigned char *)win->title);
	stand = glutBitmapHeight(GLUT_BITMAP_HELVETICA_12);
	glColor3f(0,0,0);
	glRasterPos2f(GLP_WINDOW_SIZE/2 - length/2, GLP_TITLE_POS + stand / 2);
	for (int i = 0; i < strlen(win->title); i++)
		glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, win->title[i]);

	// There we go, done!
	glutSwapBuffers();
	glutPostRedisplay();
	return;	
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Initializes the GLPLotting object and runs plotting thread
///
///   Only lets one GLPlotting object actually run
///
////////////////////////////////////////////////////////////////////////////////
void GLPlotting::init()
{
	bool init = false;

	mMutex.enter();
	init = mInit;
	mInit = true;
	mMutex.leave();

	if (!init)
	{
		// Create freeglut thread
		int argc = 1;
		char* argv[1] = {"run"};
		glutInit(&argc, argv);
		createThread();
	}
}

void GLPlotting::destroy()
{
	
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Creates or selects a new figure (up to GLP_MAX_WINDOWS)
///
///   \param num The figure number to create/select
///   \param width On creation, the width of the figure
///   \param height On creation, the height of the figure
///   \param x Reserved for future, do not use
///   \param y Reserved for future, do not use
///
///   \return Returns figure number of the figure created/selected
///
////////////////////////////////////////////////////////////////////////////////
int GLPlotting::figure(int num, int width /*= 800*/, int height /*= 600*/, int x /*= 10*/, int y /*= 50*/)
{
	int result = 0;
	if (num < GLP_MAX_WINDOWS)
	{
		if (num > mMax)
			mMax = num;

		if (mWindows[num].num == num)
		{
			mQMutex.enter();
			mCurrent = num;
			mQMutex.leave();
		}
		else
		{
			GlutWindow win;
			win.num = num;
			win.width = width;
			win.height = height;
			win.x = x;
			win.y = y;
			mMutex.enter();
			mCreateWindows.enqueue(win);
			mRun = 1;
			result = 1;
			mCurrent = num;
			mMutex.leave();
			SLEEP(10);
		}
	}
	return result;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Main thread of the GLPlotting object, draws new data
///
///   Retreives new data from the 
///
///   \return 
///
////////////////////////////////////////////////////////////////////////////////
void GLPlotting::execute()
{
	while (!quitFlag())
	{
		mMutex.enter();

		// Transfer data accumulated in the temporary buffer by calls to addPoint to our official data buffer running in the thread for plotting
		mQMutex.enter();
		for (int i = 0; i <= mMax; i++)
		{
			for (int j = 0; j < GLP_MAX_GRAPHS; j++)
			{
				Queue<float> &q = mWindows[i].data[j];
				Array<float> &a = mTmpQueue[i][j];
				for (int k = 0; k < a.size(); k++)
				{
					q.enqueue(a[k]);
				}
				a.clear();
				while (q.size() > mWindows[i].xstop)
				{
					q.dequeue();
				}
			}
		}
		mQMutex.leave();

		// mCreateWindows is a queue of each new window to create added by another thread
		while (mCreateWindows.size())
		{
			GlutWindow win = mCreateWindows.dequeue();
			sprintf(win.windowName, "Figure %d", win.num);
			glutInitDisplayMode(GLUT_RGBA | GLUT_SINGLE);
			glutInitWindowSize(win.width,win.height);
			glutInitWindowPosition(win.x,win.y);
			win.id = glutCreateWindow(win.windowName);
			glutDisplayFunc(GLPlotting::draw);
			glutKeyboardFunc(GLPlotting::keyboard);
			float r = RR(GLP_COLOR_BACK);
			float g = GG(GLP_COLOR_BACK);
			float b = BB(GLP_COLOR_BACK);
			glClearColor(r,g,b,0.0);
			gluOrtho2D(0, 1000, 1000, 0);

			mWindows[win.num].num = win.num;
			mWindows[win.num].id = win.id;
			mWindows[win.num].width = win.width;
			mWindows[win.num].height = win.height;
			mWindows[win.num].x = win.x;
			mWindows[win.num].y = win.y;
			strcpy(win.windowName, mWindows[win.num].windowName);
		}
		
		glutMainLoopEvent();
		mMutex.leave();

		// Let's not refresh the data too fast, after all this is supposed to be low CPU
		SLEEP(10);
	}
}

void GLPlotting::cleanup()
{
	
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Set a title for the selected figure
///
///   \param str New title of the window
///
///   \return True if successful, false otherwise
///
////////////////////////////////////////////////////////////////////////////////
int GLPlotting::title(const char *str)
{
	int result = 0;
	mMutex.enter();
	if (mCurrent >= 0 && mCurrent <= GLP_MAX_WINDOWS && strlen(str) < GLP_MAX_STRLEN)
	{
		strcpy(mWindows[mCurrent].title, str);
		result = 1;
	}
	mMutex.leave();
	return result;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Set the x limits for the selected figure
///
///   \param xstart Start of the series
///   \param xstop End of the series
///
///   \return True if successful, false otherwise
///
////////////////////////////////////////////////////////////////////////////////
int GLPlotting::xlim(int xstart, int xstop)
{
	int result = 0;
	
	mMutex.enter();
	if (mCurrent >= 0 && mCurrent <= GLP_MAX_WINDOWS && xstart >= 0 && xstart < xstop)
	{
		mWindows[mCurrent].xstart = xstart;
		mWindows[mCurrent].xstop = xstop;
		result = 1;
	}
	mMutex.leave();
	return result;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Set the x limits for the selected figure, using 0 as the start 
///
///   \param length How many samples to show
///
///   \return True if successful, false otherwise
///
////////////////////////////////////////////////////////////////////////////////
int GLPlotting::xlim(int length)
{
	return xlim(0, length);
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets the y limits for the selected figure
///   
///   If ylow == yhigh, then the y-limits are adaptively calculated for each new
///   data point added to accomodate the min/max of the y-values
///
///   \param ylow The bottom y-axis limits
///   \param yhigh The top y-axis limits
///
///   \return True if successful, false otherwise
///
////////////////////////////////////////////////////////////////////////////////
int GLPlotting::ylim(float ylow /*= 0*/, float yhigh /*= 0*/)
{
	int result = 0;

	mMutex.enter();
	if (mCurrent >= 0 && mCurrent <= GLP_MAX_WINDOWS && ylow <= yhigh)
	{
		mWindows[mCurrent].ylow = ylow;
		mWindows[mCurrent].yhigh = yhigh;
		result = 1;
	}
	mMutex.leave();
	return result;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets the color for the specified data series on the graph
///
///   \param graph The data series to set the color for
///   \param color Use the TO_RGB(R, G, B) macro to create a color
///
///   \return True if successful, false otherwise
///
////////////////////////////////////////////////////////////////////////////////
int GLPlotting::color(int graph, int color)
{
	int result = 0;

	mMutex.enter();
	if (mCurrent >= 0 && mCurrent <= GLP_MAX_WINDOWS && graph >= 0 && graph < GLP_MAX_GRAPHS)
	{
		mWindows[mCurrent].colors[graph] = color;
		result = 1;
	}
	mMutex.leave();
	return result;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets the y-increment for the grid on the selected figure
///
///   \param yinc How often to graph the gridlines on the y-axis
///
///   \return True if successful, false otherwise
///
////////////////////////////////////////////////////////////////////////////////
int GLPlotting::grid(float yinc)
{
	int result = 0;

	mMutex.enter();
	if (mCurrent >= 0 && mCurrent <= GLP_MAX_WINDOWS)
	{
		mWindows[mCurrent].yinc = yinc;
		result = 1;
	}
	mMutex.leave();
	return result;
}

////////////////////////////////////////////////////////////////////////////////
///
///   \brief Adds a point to the graph, shifting graph over by one data point
///
///   \param graph The data series to add the point to on the selected figure
///   \param point The y-value of the point
///
///   \return True if successful, false otherwise
///
////////////////////////////////////////////////////////////////////////////////
int GLPlotting::addPoint(int graph, float point)
{
	int result = 0;

	mQMutex.enter();
	if (mCurrent >= 0 && mCurrent <= GLP_MAX_WINDOWS && graph >= 0 && graph < GLP_MAX_GRAPHS && !mPaused[mCurrent])
	{
		mTmpQueue[mCurrent][graph].add(point);
	}
	mQMutex.leave();

	return result;
}