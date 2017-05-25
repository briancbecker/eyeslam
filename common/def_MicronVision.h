#pragma once
#define BASE_PATH0			"C:\\Micron_Vision_Data\\"
#define BASE_PATH			"E:\\Micron_Vision_Data\\"
//#define BASE_PATH			"D:\\sungwook\\DATA\\Micron_Vision_Data\\"
#define CURRENT_PLANE		"files\\surf_info.txt"
#define CURRENT_TIP_OFFST	"files\\tip_offset_val.txt"
#define CALIB_LEFT_FILE     "files\\Mln.txt"
#define CALIB_RIGHT_FILE	"files\\Mrn.txt"
#define FILTER_FILENAME     "files\\LowPass1.5HzButterOrder1.txt"
#define FILTER_FILENAME2    "files\\LowPass5.0HzButterOrder1.txt"

////////////////////////////////////////////////////////////////////////////////////////
#define LOG_VER 1.00 //define specification of data ex) tip..

/*define drawing spec.*/
#define NF 0 /*not defined*/
#define TIP 1 //Tip
#define TIP_ASAP 2 //TIP given ASAP
#define STATUS 3
#define VC 4 //Visual Cue

#define CAM_LEFT 0
#define CAM_RIGHT 1



#define LASER_AIM_BEAM 10 //Laser Aiming Beam
#define LASER_TARGET 11 //Laser Targets
#define LASER_TARGET_M 12 //Laser Moving Targets


#define VEIN_MAP 20

//#define DRAW_TIP 1
////////////////////////////////////////////////////////////////////////////////////////

#define CRAZY_PROJ_MULT 1

#define REMOVE_PURPLE_FRAC 0
#define DEL_PX 10
#define LASER_TARGET_NUMBER 10


////////////////////////////////////////////////////////////////////////////////////////
#define SC_CAMERA 0
#define SC_VIDEO 1
#define SC_IMAGE 2

#ifndef CAMERA_FRAME_WIDTH
	#define CAMERA_FRAME_WIDTH 800
#endif
#ifndef CAMERA_FRAME_HEIGHT
	#define CAMERA_FRAME_HEIGHT 608
#endif
////////////////////////////////////////////////////////////////////////////////////////

#define USE_PCL 0
#define MAX_BLOB_SIZE 500
#define MIN_BLOB_SIZE 20
#define MORPH_ELEMENT_SIZE 0
#define MAX_DEPTH_LIMIT 2000
#define DEFAULT_DEPTH_LIMIT 1000

#define MAX_PATH_DIA 4000
#define PATH_DIA 4000
#define PATH_DURATION 20.0
#define PATH_TRANS_TIME 1000
#define LASER_SPOT_SIZE 600
#define TIP_OFFSET 370 //370
#define TIP_DIAMETER 700
#define TIP_SCALE 8.516709 //for 1.0 x mag.. 


#define BEAM_AREA 100.0;

#define NUM_CALIB_SAMPLES               (54*15)
//#define NUM_CALIB_SAMPLES               500
#define NUM_CALIB_TIP_SAMPLES			1000
#define NUM_FIT_POINTS 500
////////////////////////////////////////////////////////////////////////////////////////
//for Blue
#define COLOR_TH_H_MIN 75
#define COLOR_TH_H_MAX 110
//for Green
//#define COLOR_TH_H_MIN 45
//#define COLOR_TH_H_MAX 75

#define MICROSCOPE_CALIB_A 1.029900
#define MICROSCOPE_CALIB_B -0.015600
//#define MICROSCOPE_CALIB_C 1.07482993
#define MICROSCOPE_CALIB_C 1.0084033613

#define MICROSCOPE_CALIB_0 8.638498
#define MICROSCOPE_CALIB_EYE 2.215


////////////////////////////////////////////////////////////////////////////////////////

#define MAX_LINE_PRINT 100


//..Shortcut Keys...
#define RECORD 'r'
#define REC_IMAGE 'q'
#define PROJECTOR 'p'
#define TASK 't'
#define VF 'v'
#define CALIB_CAM 'c'
#define CALIB_TIP 'o'
#define CALIB_LASER 'l'

#define DRAW 'd'
#define EYE_SLAM 'i'
#define TEST 'w'
#define SURF_UPDATE 'u'
#define SURF_CONST 's'
#define FIXTURE 'f'
#define SATURATE 'S'

////////////////////////////////////////////////////////////////////////////////////////

#define WND_MAIN "Main Frame"


////////////////////////////////////////////////////////////////////////////////////////

#include <cv.hpp>
extern void _PrintResult (const char * format, ... );
extern void _PrintMat (cv::Mat mat);



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
////////////////////////////////////////////////////////////////////////////////////////

struct _PATH_INFO{
	double duration;
	double dia;
	cv::Point center;
	cv::Point3f center3d;
	cv::Point3f center3dOffset;

	bool bSetCenter;
	int nMultiPathNum;
	bool bUpdate;
	bool bUpdateThread;
	UINT transTime;
	bool bSimPath;
	float tipOffset;
	bool bAxialLimit;
	bool bRelLimit;

	float patternPhase;
	float patternPhaseMatching;

	void init(void){
		dia = PATH_DIA;
		duration = PATH_DURATION;
		center = cv::Point(CAMERA_FRAME_WIDTH/2, CAMERA_FRAME_HEIGHT/2);
		center3d = cv::Point3f(0.0, 0.0, 0.0);
		center3dOffset = cv::Point3f(0.0, 0.0, 0.0);
		bSetCenter = false;
		bUpdate = false;
		bUpdateThread = false;
		bSimPath = false;
		nMultiPathNum = 0;
		transTime = PATH_TRANS_TIME; 
		tipOffset = TIP_OFFSET;
		
		patternPhase = 0.0;
		patternPhaseMatching = 0.0;

		bAxialLimit = true;
		bRelLimit = true;

	};

	bool checkUpdate()
	{	
		bool curStatus = bUpdate;
		if(curStatus)
			bUpdate = !curStatus;
		return curStatus;
	};
	void setAxialLimit(void)
	{
		bAxialLimit = true;
	}

};
typedef struct _PATH_INFO_SUB{
	float dia;
	cv::Point center;

}_PATH_INFO_SUB;

typedef struct _DRAWING_OPT{
	bool bTipTraj;
	bool bLaserTraj;
	bool bLaserTrajUpdate;

	bool bTargetTraj;
	bool bNullTip;
	bool bAnalysisContour;


	bool bOnDraw;
	bool bRightVein;
	bool bTipProj;
	bool bDrawTip;
	bool bTipCue;

	bool bDrawBeam;

	void init(void){
		bTipTraj = false;
		bLaserTraj = false;
		bLaserTrajUpdate = true;
		bTargetTraj = false;
		bNullTip = false;
		bAnalysisContour = true;

		bRightVein = false;
		bOnDraw = true;
		bTipProj = false;
		bDrawTip = true;
		bDrawBeam = true;
		bTipCue = false;

	}


}_DRAWING_OPT;

typedef struct _LASER_PATH
{
	UINT totalTime; //ms
	UINT curTime; //ms

	cv::Point3f pt3d;
	int fireLaser; //either 0 or 1
	int type; //Init : -1. Transition:0 or Target 1

}_LASER_PATH;

typedef struct _PROJ_TIP_CUES
{
	float radius;
	float rScale;
	cv::Point oTip;
	cv::Point oNull;
	cv::Point oTarget;

	cv::Point xyTip;
	cv::Point xyNull;

	float zTip;
	float zNull;
	


	void init(float r, float scale, cv::Point target)
	{
		radius = r;
		rScale = scale;
		oTip = cv::Point(80, 70);
		oNull = cv::Point(80, 180);
		oTarget = target;
		xyTip = cv::Point(0,0);
		xyNull = cv::Point(0,0);
		zTip = 50.0/2;
		zNull = 50.0/2;
	}

}_PROJ_TIP_CUES;

typedef struct _TARGET_QUEUE
{
	UINT id;
	cv::Point2f pt;

}_TARGET_QUEUE;

#include "kf.h"
class CLaserFixtureControl
{
public:
	CLaserFixtureControl() {
		init();
	};
	~CLaserFixtureControl() {}; 



	void init(void){
		cntLastReachTarget = 0;
		cntEnable = false;
		cntPrev = 0;
		cntLaserFire = 0;

		laserValid = false;
		laserFired = false;
	

		beam2d = cv::Point2f(0.0f, 0.0f);
		beam2dLast = cv::Point2f(0.0f, 0.0f);
		target2dEnabled = cv::Point2f(0.0f, 0.0f);


		beam3d = cv::Point3f(0.0f, 0.0f, 0.0f);
		beam3dLast = cv::Point3f(0.0f, 0.0f, 0.0f);

		tip3d0 = cv::Point3f(0.0f, 0.0f, 0.0f);
		err2d0 = cv::Point2f(0.0f, 0.0f);
		beam2d0 = cv::Point2f(0.0f, 0.0f);

		curTip = cv::Point3f(0.0f, 0.0f, 0.0f);
		curOrigin = cv::Point3f(0.0f, 0.0f, 0.0f);
		curRCM = cv::Point3f(0.0f, 0.0f, 0.0f);

		tipLast = cv::Point3f(0.0f, 0.0f, 0.0f);
		goalLast = cv::Point3f(0.0f, 0.0f, 0.0f);

		prevErr =  cv::Point2f(0.0f, 0.0f);
		errSum = cv::Point2f(0.0f, 0.0f);
		errDiff = cv::Point2f(0.0f, 0.0f);

		err3d0 = cv::Point3f(0.0f, 0.0f, 0.0f);
		prevErr3d =  cv::Point3f(0.0f, 0.0f, 0.0f);
		errSum3d = cv::Point3f(0.0f, 0.0f, 0.0f);
		errDiff3d = cv::Point3f(0.0f, 0.0f, 0.0f);


		kfErrDiff.resetNoise(1e3, 1e-4);
		kfErrDiff3d.resetNoise(1e3, 1e-4);
		kfPlaneDist.resetNoise(1e3, 1e-4);

		tipVel = cv::Point3f(0.0f, 0.0f, 0.0f);
		prevTip = cv::Point3f(0.0f, 0.0f, 0.0f);
		kfTipVel3d.resetNoise(1e2, 1e-4);


		taskDone = false;
		targetSize = 0;
		nLaserBurns = 0;
		initTime = 0;
		endTime = 0;
		laserBoostDuration = 0;
		curDistPlane = 0.0;
		correctRatio = 1.0;
		targetQueue.clear();
		targetID = 0;
		bFindTarget = false;
		

		//_PrintResult("Initialize 'CLaserFixtureControl'");
		

	};
	void reset(void)
	{
		init();
		kfErrDiff.initState(cv::Point2f(0.0f, 0.0f), cv::Point2f(0.0f, 0.0f));
		kfErrDiff3d.initState(cv::Point3f(0.0f, 0.0f,0.0f), cv::Point3f(0.0f, 0.0f,0.0f));
		kfPlaneDist.initState(0.0f,0.0f);

	};
	/*void reset(UINT nTargets)
	{
		init();
		kfErrDiff.initState(cv::Point2f(0.0f, 0.0f), cv::Point2f(0.0f, 0.0f));
		kfErrDiff3d.initState(cv::Point3f(0.0f, 0.0f,0.0f), cv::Point3f(0.0f, 0.0f,0.0f));
		kfPlaneDist.initState(0.0f,0.0f);

		targetSize = nTargets;
	};*/
	void resetResult(void)
	{
		burnTarget.resize(0);
	}
	void reset(DWORD counter)
	{
		if(counter == 0)
			init();
	};
	void initkfErrDiff(cv::Point2f initPos)
	{
		kfErrDiff.initState(initPos, cv::Point2f(0.0f, 0.0f));
	};
	void initkfErrDiff(cv::Point3f initPos)
	{
		kfErrDiff3d.initState(initPos, cv::Point3f(0.0f, 0.0f,0.0f));
	};
	UINT getSequenceNum(void)
	{
		UINT nTargetLeft = (targetSize-nLaserBurns);
		return (targetSize - nTargetLeft);
	}
	int getTargetPointer(void)
	{
		UINT num = getSequenceNum();
		if(num>=0 && num < targetSize)
			return (int)pTargetSeq->at(num);
		else
			return 0;
	}
	bool isTaskDone(void)
	{
		/*UINT nTargetLeft = (targetSize-nLaserBurns);
		return (nTargetLeft == 0);*/

		if(targetQueue.size() == 0)
			return true;
		else
			return false;
	}

	void kfErrDiffFilter(cv::Point2f &errDiff)
	{
		kfErrDiff.filter(errDiff);
	};
	void kfErrDiffFilter(cv::Point3f &errDiff3d)
	{
		kfErrDiff3d.filter(errDiff3d);
	};
	void kfPlaneDistFilter(float &dist)
	{
		kfPlaneDist.filter(dist);
	};
	void kfTipVelFilter(cv::Point3f &tipVel)
	{
		kfTipVel3d.filter(tipVel);
	};



	void setError2d(cv::Point2f error2d)
	{
		//errDiff = error2d -prevErr;
		errDiff = error2d -err2d;

		err2d = error2d;

		kfErrDiffFilter(errDiff);
		errDiff = 1000.*errDiff;

		errSum += error2d*0.001;

	};
	void setError(cv::Point3f error3d)
	{
		errDiff3d = error3d -prevErr3d;
		kfErrDiffFilter(errDiff3d);
		errDiff3d = 1000.*errDiff3d;

		errSum3d += error3d*0.001;

	};
	void setPrevError(cv::Point2f error2d)
	{
		prevErr = error2d;
	};
	void setPrevError(cv::Point3f error3d)
	{
		prevErr3d = error3d;
	};

	void setInitError(cv::Point2f error2d)
	{
		err2d0 = error2d;
		prevErr = error2d;
		beam2d0 = beam2d;
		tip3d0 = curTip;

	};
	void setInitError2(cv::Point2f error2d)
	{
		err2d0 = error2d;
		//prevErr = error2d;
		//err2d  = error2d;

		beam2d0 = beam2d;
		tip3d0 = curTip;

		curTip0 = curTip;

		target2d0 = beam2d0;
		//pTarget2d0 = &beam2d0;



		//if(targetQueue.size()> 0)
		//	target2d = targetQueue.at(0);
		//else
		//	target2d = beam2d0;

	};
	void setInitError(cv::Point3f error3d)
	{
		err3d0 = error3d;
		prevErr3d = error3d;

		beam3d0 = beam3d;
		tip3d0 = curTip;

	};
	void setTipVel(void)
	{
		cv::Point3f diffPos = curTip - prevTip;
		//_PrintResult("TipVel: %f %f", curTip.x, prevTip.x);
		kfTipVel3d.filter(diffPos);
		tipVel = 1000.*diffPos;

		prevTip = curTip;
		

	};
	





	std::vector<cv::Point2f> burnTarget;

	DWORD cntLastReachTarget; //last timerCount when laser is fired in order to discrimate succsesive fire flag
	bool cntEnable; //enable convergence checking procedure once tip reaches target..
	DWORD cntPrev; //counter to keep laser flag within threshold
	UINT cntLaserFire;
	
	bool laserValid;
	bool laserFired;
	bool taskDone;
	bool bGoalFix;

	cv::Point3f tip3d0;

	cv::Point2f beam2d0;
	cv::Point2f beam2d;
	cv::Point2f beam2dLast;
	cv::Point2f target2dEnabled;
	cv::Point3f goal3dEnabled;
	
	cv::Point3f beam3d0;
	cv::Point3f beam3d;
	cv::Point3f beam3dLast;

	cv::Point2f err2d0;
	cv::Point2f prevErr;

	cv::Point2f err2d;
	cv::Point2f errDiff;
	cv::Point2f errSum;


	cv::Point3f err3d0;
	cv::Point3f prevErr3d;
	cv::Point3f errDiff3d;
	cv::Point3f errSum3d;

	cv::Point3f curRCM;
	cv::Point3f curTip;
	cv::Point3f curTip0; //either set when laser is fired or m_timerCount == 0
	cv::Point3f curOrigin;
	cv::Point3f tipLast;
	cv::Point3f goalLast;

	KalmanConstVel2D kfErrDiff;
	KalmanConstVel3D kfErrDiff3d;
	KalmanPosVel1D kfPlaneDist;

	cv::Point3f tipVel;
	cv::Point3f prevTip;
	KalmanConstVel3D kfTipVel3d;

	UINT targetSize;
	UINT nLaserBurns;

	DWORD initTime;
	DWORD endTime;

	DWORD prevBurnTime; //time when the previous burn is completed.
	UINT targetInterval; //interval used for rate fix..
	UINT laserBoostDuration; //defined by last boost duration..for rate fix..
	UINT openLoopCount;

	std::vector<UINT> *pTargetSeq; //target sequences from avoidance checking, Otherwise it is from 0 to n-1.
	//std::vector<cv::Point2f> targetQueue;
	std::vector<_TARGET_QUEUE> targetQueue; 

	float curDistPlane;
	float correctRatio;
	
	
	cv::Point2f target2d;

	cv::Point2f target2d0;
	//cv::Point2f *pTarget2d0; //for taking into account moving targets..

	UINT targetID;
	bool bFindTarget;
	

};

typedef struct _LASER_SETTING
{
	UINT minFireCnt;
	bool bOn;
	bool bPathCorrect;
	bool bAvoidance;
	bool bFeedForward;
	bool bGoalFix;
	bool bRateFix;
	bool bAdaptive;
	bool bStackTRs;
	bool bSemiControl;
	bool bTipFilterOverride;


	UINT nOpenLoopDelay;

	float avoidTh;

	float Pk, Pi, Pd;

	float Pz[3];
	float convTh;
	bool bHCorrect;
	float fireFreq;
	float semiThreshold;

	void init(void){
		bOn = false;
		bPathCorrect = true;
		minFireCnt = 200;
		fireFreq = 1.0f;
		avoidTh = 150;

		bAvoidance = false;
		bFeedForward = true;

		//Pk = 2.0f;
		//Pi = 0.02f;
		//Pd = 0.0f;

		//Pk = 3.0f;
		//Pi = 0.05f;
		//Pd = 0.15f;

		
		Pk = 1.0f;
		Pi = 0.2f;
		Pd = 0.0f;
		
		Pz[0] = 0.5f;
		Pz[1] = 0.0;
		Pz[2] = 0.0;

		convTh = 100.0f; //20 um
		bHCorrect = true;
		bGoalFix = false;
		bRateFix = false;
		bAdaptive = false;
		bStackTRs = false;
		bSemiControl = false;
		bTipFilterOverride = false;
		nOpenLoopDelay = 170;

		semiThreshold = 250;

//////////////////////////////////////////////////
		//convTh = 100.0f; //20 um
		//convTh = 50.0f; //20 um
		//bHCorrect = true;
		//bGoalFix = false;
		//bRateFix = true;
		//bPathCorrect = false;
		//bAdaptive = true;
		//bStackTRs = false;
		//bSemiControl = true;
		//bTipFilterOverride = true;
		bOn = true;
//////////////////////////////////////////////////



	};


}_LASER_SETTING;

typedef struct _SUB_PATH
{
	UINT duration;
	double traj;
	float radius;
	cv::Point3d center3d;
	bool bTarget;

}_SUB_PATH;

typedef struct _BURN_ANALYSIS
{
	float tipScale;
	bool bKMeans;
	int selImage;
	int nTargets;
	float executionTime;

	std::vector<cv::Point2f> targetPts;
	std::vector<cv::Point2f> beamPtsBurn;
	std::vector<cv::Point2f> beamPtsContour;
	std::vector<cv::Point2f> beamPtsKMeans;

	std::vector<double> errorsBurn;
	std::vector<double> errorsContour;
	std::vector<double> errorskMeans;

	double avgBurnError;
	double avgContourError;
	double avgKMeansError;


	void init(int selImg, int nTargetSize, bool kmeans, float scale, float runTime)
	{
		selImage = selImg;
		nTargets = nTargetSize;
		bKMeans = kmeans;
		tipScale = scale;
		executionTime = runTime;

		targetPts.clear();
		beamPtsBurn.clear();
		beamPtsContour.clear();
		beamPtsKMeans.clear();

		errorsBurn.clear();
		errorsContour.clear();
		errorskMeans.clear();
		avgBurnError = avgContourError = avgKMeansError = 0.0;
		
	}
	

}_BURN_ANALYSIS;

typedef struct _TIP_OFFSET
{
	bool bTipOffset;
	float valTipOffset;

	void init(void){
		bTipOffset = false;
		valTipOffset = TIP_OFFSET;
	};
	void setVal(float val){	valTipOffset = val;	};
	void onoff(bool flag){	bTipOffset = flag;	};

}_TIP_OFFSET;

typedef struct _TIP_OFFSET_ADJ
{
	float x;
	float y;
	float z;
	bool bUpdate;
	
	void init(void){
		x = y = 0.0;
		z = 55000;
	};

}_TIP_OFFSET_ADJ;


typedef struct _KF_NOISE
{
	double procNoise;
	double measNoise;
	bool bUpdate;
	bool bDraw;

	void init(void){
		procNoise = measNoise = 1e2;
		bDraw = false;
		bUpdate = false;
	};

}_KF_NOISE;


typedef struct _PROJ_CONTROL
{
	char * pProjOn;

	bool bProjCalib;
	bool bRunProjCalib;
	



	void init(void){
		pProjOn = NULL;
		bProjCalib = false;
		bRunProjCalib = false;
	};
	void init(char *pKeyProjOn){
		pProjOn = pKeyProjOn;
		bProjCalib = false;
		bRunProjCalib = false;
	};

	bool isOn(void)
	{
		return (!!(*pProjOn));
	}; 
	/*
	void setVal(float val){	valTipOffset = val;	};
	void onoff(bool flag){	bTipOffset = flag;	};
*/
}_PROJ_CONTROL;



// If we are using quicksync, we should be able to save out full-res videos (hopefully!)
#if USE_QUICKSYNC
//#define RESZE_VIDEO_TO cv::Size(640, 480)
#define RESZE_VIDEO_TO cv::Size(800, 608)
#else
//#define RESZE_VIDEO_TO cv::Size(640, 480)
#define RESZE_VIDEO_TO cv::Size(800, 608)
#endif