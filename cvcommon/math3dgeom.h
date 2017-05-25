

#ifndef MATH3DGEOM_H
#define MATH3DGEOM_H

#include "array.h"
#include <cv.h>


#define MATH3D_NUM3DPT2 12


class Math3dGeom
{
public:
	Math3dGeom();
	~Math3dGeom();

	bool project(const CvPoint3D32f &ptASAP, const CvMat *Ml, CvPoint2D32f &pl, const CvMat *Mr, CvPoint2D32f &pr);
	CvPoint2D32f project(const CvPoint3D32f &ptASAP, const CvMat *M);
	cv::Point2d project(cv::Point3f &pt3D, cv::Mat &M);

	CvPoint3D32f P_from_Mp(const CvMat *Ml, const CvPoint2D32f &pl, const CvMat *Mr, const CvPoint2D32f &pr);
	CvPoint3D32f reproject(const CvMat *Ml, const CvPoint2D32f &pl, const CvMat *Mr, const CvPoint2D32f &pr) { return P_from_Mp(Ml, pl, Mr, pr); }
	CvPoint3D32f reprojectCV(CvMat *Ml, const CvPoint2D32f &pl, CvMat *Mr, const CvPoint2D32f &pr);

	CvPoint3D32f getPointAlongPose(const CvPoint3D32f &pt, CvMat *pose, double length);
	CvPoint3D32f getPointAlongPose(const CvPoint3D32f &pt, CvMat *pose, const CvPoint3D32f &vec);
	CvPoint3D32f intersectLinePlane(const CvPoint3D32f &ptLine1, const CvPoint3D32f &ptLine2, const CvPoint3D32f &ptPlane1, const CvPoint3D32f &ptPlane2, const CvPoint3D32f &ptPlane3);
	CvPoint3D32f intersectLinePlane(const CvPoint3D32f &ptLine1, const CvPoint3D32f &ptLine2, const CvPoint3D32f &ptPlane1, const CvPoint3D32f &vecPlaneNormal);

	static float distancePointToPlane(const cv::Point3f planePt, const cv::Point3f planeNormal, const cv::Point3f pt);
	static float signedDistancePointToPlane(const cv::Point3f planePt, const cv::Point3f planeNormal, const cv::Point3f pt);
	
	static void estimatePlane(const Array<cv::Point3f> &pts, cv::Point3f &normal, double &offset, cv::Point3f &planePt);
	static void estimatePlane(const Array<cv::Point3f> &points, cv::Point3f &normal, double &offset, cv::Point3f &planePt, cv::Mat &eigenVectors);
	void estimatePlane(std::vector<cv::Point3f> &points, cv::Point3d &normal, cv::Point3f &planePt, cv::Mat &eigenVectors);

	static cv::Point3f fastCross(const cv::Point3f a, const cv::Point3f b);
	static float fastDot(const cv::Point3f a, const cv::Point3f b);

	cv::Point3f cross(cv::Point3f a, cv::Point3f b);
	static double dot(cv::Point3f a, cv::Point3f b);
	static double dot(cv::Point2f a, cv::Point2f b);

	cv::Point3f getPointAlongDirection(cv::Point3f base, cv::Point3f direction, double amount);
	cv::Point3f getCameraDirection(CvMat *m);
	cv::Mat axisToRoation(cv::Point3f from, cv::Point3f to);
	static cv::Point3f norm(cv::Point3f pt);
	double mag(cv::Point3f pt);
	double mag(cv::Point2f pt);
	double mag(cv::Point pt);
	double magSqr(cv::Point3f pt);
	double magSqr(cv::Point pt);
	cv::Mat pt2mat(cv::Point3f pt);
	cv::Mat pt2mat(cv::Point2f pt);
	cv::Point3f mat2pt(cv::Mat &m);
	cv::Point2f mat2pt2(cv::Mat &m);
	double distanceLinePlane(cv::Point3f ptLine1, cv::Point3f ptLine2, cv::Point3f pt);
	cv::Point3f intersectPointLine(cv::Point3f p1, cv::Point3f p2, cv::Point3f pX);
	//cv::Point3f rotate(cv::Point3f pt, cv::Point3f from, cv::Point3f to);

	static double sq(double x) { return x*x; }

	static CvPoint3D32f sub(const CvPoint3D32f &p, const CvPoint3D32f &q);
	static CvPoint3D32f add(const CvPoint3D32f &p, const CvPoint3D32f &q);
	static CvPoint3D32f mul(const CvPoint3D32f &p, float multiplier);
	static CvPoint3D32f div(const CvPoint3D32f &p, float multiplier);

	static CvPoint2D32f sub(const CvPoint2D32f &p, const CvPoint2D32f &q);
	static CvPoint2D32f add(const CvPoint2D32f &p, const CvPoint2D32f &q);

	static int fitSphereFrom4Pts(cv::Point3f p0, cv::Point3f p1, cv::Point3f p2, cv::Point3f p3, cv::Point3f &center, double &radius);
	static int fitSphereFromNPts(const Array<cv::Point3f> &pts, cv::Point3f &center, double &radius);

	static int ransacSphereFit(const Array<cv::Point3f> &pts, int iterations, double minDist, cv::Point3f &center, double &radius);

	static int intersectLineSphere(cv::Point3f center, double radius, cv::Point3f linePt, cv::Point3f lineUnitDirection, double &d1, double &d2);
	static int intersectLineSphere(cv::Point3f center, double radius, cv::Point3f linePt, cv::Point3f lineUnitDirection, cv::Point3f &pt1, cv::Point3f &pt2);

	static cv::Point3d calcVecMean(const std::vector<cv::Point3f> &points);
	static cv::Point2d calcVecMean(const std::vector<cv::Point2f>& points); 
	static double calcVecMean(const std::vector<double>& vals); 
	static double calcVecStd(const std::vector<double>& vals); 


	static cv::Mat estimate3dTransform(std::vector<cv::Point3f> srcArray, std::vector<cv::Point3f> dstArray);
	static cv::Mat estimate3dTransform(std::vector<cv::Point3f> srcArray, std::vector<cv::Point3f> dstArray, int nMaxPts);

	static std::vector<cv::Point3f> apply3dTransform(cv::Mat TR, std::vector<cv::Point3f> srcArray);
	static std::vector<cv::Point2f> apply2dTransform(cv::Mat TR, std::vector<cv::Point2f> srcArray);
	cv::Point2f Math3dGeom::apply2dTransform(cv::Mat TR, cv::Point2f src);

	static std::vector<int> randperm(int n);

	static std::vector<cv::Point2f> randSampling(std::vector<cv::Point2f> &srcArray, int maxPts);

	//cv::Point2f perspectiveTransform(cv::Point2f pt,  cv::Mat H);
	//cv::Point3f perspectiveTransform(cv::Point3f pt,  cv::Mat H);
	void perspectiveTransform(cv::Point2f &pt,  cv::Point2f &dst, cv::Mat H);
	void perspectiveTransform(cv::Point3f &pt,  cv::Point3f &dst, cv::Mat H);

	double calcDistVec(cv::Point2f v0, cv::Point2f v1);
	cv::Point3f projPtonPlane(cv::Point3f pt, cv::Point3f normal, cv::Point3f ptPlane);
	double axialDistancefromPlane(cv::Point3f tip, cv::Point3f origin, cv::Point3f planeCenter, cv::Point3f planeNormal);


	//template <typename T> std::vector<T> randSampling(std::vector<T> srcArray, int maxPts)
	//{
	//	//random sampling to reduce the number of points 
	//	std::vector<int> idx = randperm(maxPts);

	//	std::vector<T> sampledSrcArray;

	//	if(srcArray.size() > nMaxPts)
	//	{
	//		for(int i =0 ; i < idx.size(); i++)
	//		{
	//			sampledSrcArray.push_back(srcArray[idx[i]]);

	//		}
	//	}
	//	else
	//		sampledSrcArray = srcArray;

	//	return sampledSrcArray;
	//};
	cv::Mat_<double> Math3dGeom::LinearLSTriangulation(cv::Point3d u,       //homogenous image point (u,v,1)
                   cv::Matx34d P,       //camera 1 matrix
                   cv::Point3d u1,      //homogenous image point in 2nd camera
                   cv::Matx34d P1       //camera 2 matrix
                                   );

	cv::Mat_<double> IterativeLinearLSTriangulation(cv::Point3d u,    //homogenous image point (u,v,1)
                                            cv::Matx34d P,          //camera 1 matrix
                                            cv::Point3d u1,         //homogenous image point in 2nd camera
                                            cv::Matx34d P1          //camera 2 matrix
                                            ) ;

private:
	void zero();
	void init();
	void destroy();

	CvMat *pt2d, *pt3d, *pt2dLeft, *pt2dRight;
	CvMat *W, *WtW;
	CvMat *V, *D;

	CvMat *ptLen, *ptPt;

	CvMat *planeNormal;
	CvMat *p3ds[MATH3D_NUM3DPT2];
	CvMat *p2ds[MATH3D_NUM3DPT2];
};

#endif // MATH3DGEOM_H