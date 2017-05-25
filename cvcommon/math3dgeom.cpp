
#include "math3dgeom.h"
//extern void _PrintResult (const char * format,...);

using namespace cv;


Math3dGeom::Math3dGeom()
{
	init();
}

Math3dGeom::~Math3dGeom()
{
	destroy();
}

void Math3dGeom::zero()
{
	W = 0;
	WtW = 0;
	V = 0;
	D = 0;
	pt2d = 0;
	pt3d = 0;
}

void Math3dGeom::init()
{
	zero();

	W = cvCreateMat(4, 4, CV_64F);
	WtW = cvCreateMat(4, 4, CV_64F);
	V = cvCreateMat(4, 4, CV_64F);
	D = cvCreateMat(4, 4, CV_64F);
	pt2d = cvCreateMat(3, 1, CV_64F);
	pt3d = cvCreateMat(4, 1, CV_64F);

	ptPt = cvCreateMat(1, 3, CV_64F);
	ptLen = cvCreateMat(1, 3, CV_64F);
	
	planeNormal = cvCreateMat(1, 3, CV_64F);
	for (int i = 0; i < MATH3D_NUM3DPT2; i++)
		p3ds[i] = cvCreateMat(1, 3, CV_64F);
}

void Math3dGeom::destroy()
{
	cvReleaseMat(&W);
	cvReleaseMat(&WtW);
	cvReleaseMat(&V);
	cvReleaseMat(&D);
	cvReleaseMat(&pt2d);
	cvReleaseMat(&pt3d);
	cvReleaseMat(&ptPt);
	cvReleaseMat(&ptLen);
	cvReleaseMat(&planeNormal);

	for (int i = 0; i < MATH3D_NUM3DPT2; i++)
		cvReleaseMat(&p3ds[i]);
}

CvPoint3D32f Math3dGeom::P_from_Mp(const CvMat *Ml, const CvPoint2D32f &pl, const CvMat *Mr, const CvPoint2D32f &pr)
{
	CvPoint3D32f result;
	result.x = result.y = result.z = 0;

	cvmSet(W, 0, 0, cvmGet(Ml, 0, 0) - pl.x*cvmGet(Ml, 0, 2));
	cvmSet(W, 0, 1, cvmGet(Ml, 1, 0) - pl.x*cvmGet(Ml, 1, 2));
	cvmSet(W, 0, 2, cvmGet(Ml, 2, 0) - pl.x*cvmGet(Ml, 2, 2));
	cvmSet(W, 0, 3, cvmGet(Ml, 3, 0) - pl.x*cvmGet(Ml, 3, 2));

	cvmSet(W, 1, 0, cvmGet(Ml, 0, 1) - pl.y*cvmGet(Ml, 0, 2));
	cvmSet(W, 1, 1, cvmGet(Ml, 1, 1) - pl.y*cvmGet(Ml, 1, 2));
	cvmSet(W, 1, 2, cvmGet(Ml, 2, 1) - pl.y*cvmGet(Ml, 2, 2));
	cvmSet(W, 1, 3, cvmGet(Ml, 3, 1) - pl.y*cvmGet(Ml, 3, 2));

	cvmSet(W, 2, 0, cvmGet(Mr, 0, 0) - pr.x*cvmGet(Mr, 0, 2));
	cvmSet(W, 2, 1, cvmGet(Mr, 1, 0) - pr.x*cvmGet(Mr, 1, 2));
	cvmSet(W, 2, 2, cvmGet(Mr, 2, 0) - pr.x*cvmGet(Mr, 2, 2));
	cvmSet(W, 2, 3, cvmGet(Mr, 3, 0) - pr.x*cvmGet(Mr, 3, 2));

	cvmSet(W, 3, 0, cvmGet(Mr, 0, 1) - pr.y*cvmGet(Mr, 0, 2));
	cvmSet(W, 3, 1, cvmGet(Mr, 1, 1) - pr.y*cvmGet(Mr, 1, 2));
	cvmSet(W, 3, 2, cvmGet(Mr, 2, 1) - pr.y*cvmGet(Mr, 2, 2));
	cvmSet(W, 3, 3, cvmGet(Mr, 3, 1) - pr.y*cvmGet(Mr, 3, 2));

	cvGEMM(W, W, 1, 0, 0, WtW, CV_GEMM_A_T);

	cvSVD(WtW, D, V, 0, CV_SVD_U_T | CV_SVD_MODIFY_A);

	// Singular values D are sorted (descending)
	double den = cvmGet(V, 3, 3);
	if (den == 0)
		den = 1e-6;
	result.x = (float)(cvmGet(V, 3, 0) / den);
	result.y = (float)(cvmGet(V, 3, 1) / den);
	result.z = (float)(cvmGet(V, 3, 2) / den);

	return result;
}

CvPoint2D32f Math3dGeom::project(const CvPoint3D32f &ptASAP, const CvMat *M)
{

	CvPoint2D32f p;
	p.x = p.y = 0;

	cvmSet(pt3d, 0, 0, ptASAP.x);
	cvmSet(pt3d, 1, 0, ptASAP.y);
	cvmSet(pt3d, 2, 0, ptASAP.z);
	cvmSet(pt3d, 3, 0, 1.0f);
	cvGEMM(M, pt3d, 1, 0, 0, pt2d, CV_GEMM_A_T);

	p.x = (float)(cvmGet(pt2d, 0, 0));
	p.y = (float)(cvmGet(pt2d, 1, 0));
	double homo = cvmGet(pt2d, 2, 0);
	if (homo == 0)
		homo = 1e-6;
	p.x /= (float)(homo);
	p.y /= (float)(homo);

	return p;

}

cv::Point2d Math3dGeom::project(cv::Point3f &pt3D, cv::Mat &M)
{

	cv::Mat ptMat = cv::Mat(1,4,CV_64FC1);
	ptMat.at<double>(0,0) = pt3D.x;
	ptMat.at<double>(1,0) = pt3D.y;
	ptMat.at<double>(2,0) = pt3D.z;
	ptMat.at<double>(3,0) = 1.0;

	cv::Mat P(M);

	cv::Mat res = P*ptMat;

	cv::Point2f pt2d;
	pt2d.x = res.at<double>(0,0);
	pt2d.y =  res.at<double>(1,0);

	double homo = res.at<double>(2,0);
	if (homo == 0)
		homo = 1e-6;
	pt2d.x /= homo;
	pt2d.y /= homo;

	return pt2d;

}

CvPoint3D32f Math3dGeom::reprojectCV(CvMat *Ml, const CvPoint2D32f &pl, CvMat *Mr,  const CvPoint2D32f &pr)
{

		CvMat *Poin4d = cvCreateMat(4, 1, CV_64FC1);;

		CvMat *ptL = cvCreateMat(2, 1, CV_64FC1);
		cvmSet(ptL, 0, 0, pl.x);  cvmSet(ptL, 1, 0, pl.y); 

		CvMat *ptR = cvCreateMat(2, 1, CV_64FC1);
		cvmSet(ptR, 0, 0, pr.x);  cvmSet(ptR, 1, 0, pr.y); 

		CvMat *Mlt = cvCreateMat(3, 4, CV_64FC1); CvMat *Mrt = cvCreateMat(3, 4, CV_64FC1);
		cvTranspose(Ml, Mlt);
		cvTranspose(Mr, Mrt);


		cvTriangulatePoints(Mlt, Mrt, ptL, ptR, Poin4d);



		CvPoint3D32f result;

		result.x = cvmGet(Poin4d, 0, 0)/cvmGet(Poin4d, 3, 0);
		result.y = cvmGet(Poin4d, 1, 0)/cvmGet(Poin4d, 3, 0);
		result.z = cvmGet(Poin4d, 2, 0)/cvmGet(Poin4d, 3, 0);

		cvReleaseMat(&Poin4d); cvReleaseMat(&Mlt); cvReleaseMat(&Mrt);

		return result;
}


bool Math3dGeom::project(const CvPoint3D32f &ptASAP, const CvMat *Ml, CvPoint2D32f &pl, const CvMat *Mr, CvPoint2D32f &pr)
{
	pl = project(ptASAP, Ml);
	pr = project(ptASAP, Mr);
	return true;
}

CvPoint3D32f Math3dGeom::getPointAlongPose(const CvPoint3D32f &pt, CvMat *pose, double length)
{
	CvPoint3D32f result;

	cvmSet(ptLen, 0, 0, 0);
	cvmSet(ptLen, 0, 1, 0);
	cvmSet(ptLen, 0, 2, length);

	cvGEMM(ptLen, pose, 1, 0, 0, ptPt);

	result.x = (float)(pt.x + cvmGet(ptPt, 0, 0));
	result.y = (float)(pt.y + cvmGet(ptPt, 0, 1));
	result.z = (float)(pt.z + cvmGet(ptPt, 0, 2));

	return result;
}

CvPoint3D32f Math3dGeom::getPointAlongPose(const CvPoint3D32f &pt, CvMat *pose, const CvPoint3D32f &vec)
{
	CvPoint3D32f result;

	cvmSet(ptLen, 0, 0, vec.x);
	cvmSet(ptLen, 0, 1, vec.y);
	cvmSet(ptLen, 0, 2, vec.z);

	cvGEMM(ptLen, pose, 1, 0, 0, ptPt);

	result.x = (float)(pt.x + cvmGet(ptPt, 0, 0));
	result.y = (float)(pt.y + cvmGet(ptPt, 0, 1));
	result.z = (float)(pt.z + cvmGet(ptPt, 0, 2));

	return result;
}

double Math3dGeom::axialDistancefromPlane(cv::Point3f tip, cv::Point3f origin, cv::Point3f planeCenter, cv::Point3f planeNormal)
{

	cv::Point3f nn = origin - tip; //upward vector...
	if(mag(nn) > 0)
		nn = (1.0/mag(nn))*nn;

	cv::Point3f pt= intersectLinePlane(tip, origin, planeCenter, planeNormal);
	double distance  = mag(tip - pt);
	if(dot((tip-pt), nn) < 0)
		distance = -1*distance;

	return distance;
}
//http://local.wasp.uwa.edu.au/~pbourke/geometry/planeline/
CvPoint3D32f Math3dGeom::intersectLinePlane(const CvPoint3D32f &ptLine1, const CvPoint3D32f &ptLine2, const CvPoint3D32f &ptPlane1, const CvPoint3D32f &ptPlane2, const CvPoint3D32f &ptPlane3)
{
	CvPoint3D32f result = ptLine1;

	// p3ds: 0 = ptLine1, 1 = ptLine2, 2 = ptPlane1, 3 = ptPlane2, 4 = ptPlane3
	cvmSet(p3ds[0], 0, 0, ptLine1.x);
	cvmSet(p3ds[0], 0, 1, ptLine1.y);
	cvmSet(p3ds[0], 0, 2, ptLine1.z);

	cvmSet(p3ds[1], 0, 0, ptLine2.x);
	cvmSet(p3ds[1], 0, 1, ptLine2.y);
	cvmSet(p3ds[1], 0, 2, ptLine2.z);

	cvmSet(p3ds[2], 0, 0, ptPlane1.x);
	cvmSet(p3ds[2], 0, 1, ptPlane1.y);
	cvmSet(p3ds[2], 0, 2, ptPlane1.z);

	cvmSet(p3ds[3], 0, 0, ptPlane2.x);
	cvmSet(p3ds[3], 0, 1, ptPlane2.y);
	cvmSet(p3ds[3], 0, 2, ptPlane2.z);

	cvmSet(p3ds[4], 0, 0, ptPlane3.x);
	cvmSet(p3ds[4], 0, 1, ptPlane3.y);
	cvmSet(p3ds[4], 0, 2, ptPlane3.z);

	// 5 = ptPlane1 - ptPlane2
	// 6 = ptPlane1 - ptPlane3
	cvSub(p3ds[2], p3ds[3], p3ds[5]);
	cvSub(p3ds[2], p3ds[4], p3ds[6]);

	// 7 = normal = (ptPlane1 - ptPlane2) cross (ptPlane1 - ptPlane3)
	cvCrossProduct(p3ds[5], p3ds[6], p3ds[7]);

	// 8 = ptPlane1 - ptLine1
	// 9 = ptLine2 - ptLine1
	cvSub(p3ds[2], p3ds[0], p3ds[8]);
	cvSub(p3ds[1], p3ds[0], p3ds[9]);

	// u = [(normal) dot (ptPlane1 - ptLine1)] / [(normal) dot (ptLine2 - ptLine1)]
	double u = cvDotProduct(p3ds[7], p3ds[8]) / cvDotProduct(p3ds[7], p3ds[9]);

	// point on plane = ptLine1 + u * (ptLine2 - ptLine1)
	result.x += u * cvmGet(p3ds[9], 0, 0);
	result.y += u * cvmGet(p3ds[9], 0, 1);
	result.z += u * cvmGet(p3ds[9], 0, 2);
	
	return result;
}

CvPoint3D32f Math3dGeom::intersectLinePlane(const CvPoint3D32f &ptLine1, const CvPoint3D32f &ptLine2, const CvPoint3D32f &ptPlane1, const CvPoint3D32f &vecPlaneNormal)
{
	CvPoint3D32f result = ptLine1;

	// p3ds: 0 = ptLine1, 1 = ptLine2, 2 = ptPlane1
	cvmSet(p3ds[0], 0, 0, ptLine1.x);
	cvmSet(p3ds[0], 0, 1, ptLine1.y);
	cvmSet(p3ds[0], 0, 2, ptLine1.z);

	cvmSet(p3ds[1], 0, 0, ptLine2.x);
	cvmSet(p3ds[1], 0, 1, ptLine2.y);
	cvmSet(p3ds[1], 0, 2, ptLine2.z);

	cvmSet(p3ds[2], 0, 0, ptPlane1.x);
	cvmSet(p3ds[2], 0, 1, ptPlane1.y);
	cvmSet(p3ds[2], 0, 2, ptPlane1.z);

	// 7 = plane normal
	cvmSet(p3ds[7], 0, 0, vecPlaneNormal.x);
	cvmSet(p3ds[7], 0, 1, vecPlaneNormal.y);
	cvmSet(p3ds[7], 0, 2, vecPlaneNormal.z);

	// 8 = ptPlane1 - ptLine1
	// 9 = ptLine2 - ptLine1
	cvSub(p3ds[2], p3ds[0], p3ds[8]);
	cvSub(p3ds[1], p3ds[0], p3ds[9]);

	// u = [(normal) dot (ptPlane1 - ptLine1)] / [(normal) dot (ptLine2 - ptLine1)]
	double u = cvDotProduct(p3ds[7], p3ds[8]) / cvDotProduct(p3ds[7], p3ds[9]);

	// point on plane = ptLine1 + u * (ptLine2 - ptLine1)
	result.x += u * cvmGet(p3ds[9], 0, 0);
	result.y += u * cvmGet(p3ds[9], 0, 1);
	result.z += u * cvmGet(p3ds[9], 0, 2);

	return result;
}

CvPoint3D32f Math3dGeom::sub(const CvPoint3D32f &p, const CvPoint3D32f &q)
{
	return cvPoint3D32f(p.x - q.x, p.y - q.y, p.z - q.z);
}

CvPoint2D32f Math3dGeom::sub(const CvPoint2D32f &p, const CvPoint2D32f &q)
{
	return cvPoint2D32f(p.x - q.x, p.y - q.y);
}
CvPoint3D32f Math3dGeom::add(const CvPoint3D32f &p, const CvPoint3D32f &q)
{
	return cvPoint3D32f(p.x + q.x, p.y + q.y, p.z + q.z);
}

CvPoint2D32f Math3dGeom::add(const CvPoint2D32f &p, const CvPoint2D32f &q)
{
	return cvPoint2D32f(p.x + q.x, p.y + q.y);
}

CvPoint3D32f Math3dGeom::mul(const CvPoint3D32f &p, float multiplier)
{
	return cvPoint3D32f(p.x*multiplier, p.y*multiplier, p.z*multiplier);
}

CvPoint3D32f Math3dGeom::div(const CvPoint3D32f &p, float multiplier)
{
	return cvPoint3D32f(p.x/multiplier, p.y/multiplier, p.z/multiplier);
}

cv::Point3f Math3dGeom::cross(cv::Point3f a, cv::Point3f b)
{
	cv::Point3f result;

	cvmSet(p3ds[0], 0, 0, a.x);
	cvmSet(p3ds[0], 0, 1, a.y);
	cvmSet(p3ds[0], 0, 2, a.z);

	cvmSet(p3ds[1], 0, 0, b.x);
	cvmSet(p3ds[1], 0, 1, b.y);
	cvmSet(p3ds[1], 0, 2, b.z);

	cvCrossProduct(p3ds[0], p3ds[1], p3ds[2]);

	result.x = cvmGet(p3ds[2], 0, 0);
	result.y = cvmGet(p3ds[2], 0, 1);
	result.z = cvmGet(p3ds[2], 0, 2);

	return result;
}

// Takes the base 3D point, and adds a vector in the specified direction by the specified amount (direction doesn't have to be normalized)
cv::Point3f Math3dGeom::getPointAlongDirection(cv::Point3f base, cv::Point3f direction, double amount)
{
	cv::Point3f result;

	cvmSet(p3ds[0], 0, 0, direction.x);
	cvmSet(p3ds[0], 0, 1, direction.y);
	cvmSet(p3ds[0], 0, 2, direction.z);

	cvNormalize(p3ds[0], p3ds[2], 1, 0, CV_L2);

	result.x = base.x + cvmGet(p3ds[2], 0, 0)*amount;
	result.y = base.y + cvmGet(p3ds[2], 0, 1)*amount;
	result.z = base.z + cvmGet(p3ds[2], 0, 2)*amount;

	return result;
}

cv::Point3f Math3dGeom::getCameraDirection(CvMat *m)
{
	static FILE *fp = 0;//fopen("camdir.txt", "wb");
	cv::Mat mat(3, 3, CV_64F);
	cv::Point3f vec;

	cv::Mat mct(m);
	cv::Mat mc = mct.t();

	// From Hartley & Zisserman, section 6.2.4 under "Finding the camera centre"
	double dir[4] = {0};
	double *ptr = (double*)mat.data;
	// Each direction
	for (int i = 0; i < 4; i++)
	{
		int ctr = 0;
		// Take all the cols except the direction we are currently on
		for (int j = 0; j < 4; j++)
		{
			if (i == j)
				continue;

			// Copy the cols to make the 3x4 into a 3x3
			for (int k = 0; k < 3; k++)
			{
				//mat.at<float>(k, ctr) = cvmGet(m, k, j);
				mat.at<double>(k, ctr) = mc.at<double>(k, j);
			}

			ctr++;
		}

		// Alternate signs on the direction, starting with positive
		dir[i] = ((i%2) ? -1 : 1) * cv::determinant(mat);
	}

	//printf("B:%f %f %f\n", dir[0]/dir[3], dir[1]/dir[3], dir[2]/dir[3]);
	//edited by sungwook...start..

	cv::Mat mat2(m);
	mat2 =mat2.t();

	cv::Mat subM = mat2.colRange(cv::Range(0,3));
	cv::Mat tM = mat2.col(3);
	cv::Mat tvec = -subM.inv()*tM;
	//printf("S:%f %f %f\n", tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0));
	//edited by sungwook...end..

	if (fp)
	{
		for (int j = 0; j < 4; j++)
		{
			// Copy the cols to make the 3x4 into a 3x3
			for (int k = 0; k < 3; k++)
			{
				fprintf(fp, "%f ", mc.at<double>(k, j));
			}
		}
		fprintf(fp, "  %f %f %f %f", dir[0], dir[1], dir[2], dir[3]);
		fprintf(fp, "\n");
		fflush(fp);
	}

	// Take out of homogenous coordinates
	int w = dir[3];
	if (w == 0)
		w = 1;
	return cv::Point3f(dir[0] / w, dir[1] / w, dir[2] / w);
	//return cv::Point3f(tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0));
}

//cv::Point3f Math3dGeom::rotate(cv::Point3f pt, CvMat *r)
//{
//	cv::Point3f result;
//
//	cvmSet(p3ds[1], 0, 0, pt.x);
//	cvmSet(p3ds[1], 0, 1, pt.y);
//	cvmSet(p3ds[1], 0, 2, pt.z);
//
//	cvGEMM(p3ds[1], r, 1, 0, 0, p3ds[0]);
//
//	result.x = cvmGet(p3ds[0], 0, 0);
//	result.y = cvmGet(p3ds[0], 0, 1);
//	result.z = cvmGet(p3ds[0], 0, 2);
//
//	return result;
//}

cv::Mat Math3dGeom::axisToRoation(cv::Point3f from, cv::Point3f to)
{
	from = norm(from);
	to = norm(to);
	double angle = acos(dot(from,to));
	cv::Point3f axis = cross(from, to);
	axis = norm(axis);

	cv::Mat axis_skewed(3,3,CV_64F);

	axis_skewed.at<double>(0, 0) = 0;
	axis_skewed.at<double>(0, 1) = -axis.z;
	axis_skewed.at<double>(0, 2) = axis.y;

	axis_skewed.at<double>(1, 0) = axis.z;
	axis_skewed.at<double>(1, 1) = 0;
	axis_skewed.at<double>(1, 2) = -axis.x;

	axis_skewed.at<double>(2, 0) = -axis.y;
	axis_skewed.at<double>(2, 1) = axis.x;
	axis_skewed.at<double>(2, 2) = 0;

	cv::Mat eye(3,3,CV_64F);
	eye = eye.eye(cv::Size(3,3), CV_64F);

	cv::Mat R = eye + sin(angle)*axis_skewed + (1-cos(angle))*axis_skewed*axis_skewed;
	double *ptr = (double*)R.data;

	return R;
}

double Math3dGeom::dot(cv::Point3f a, cv::Point3f b)
{
	return a.x*b.x + a.y*b.y + a.z*b.z;
}

double Math3dGeom::dot(const cv::Point2f a, const cv::Point2f b)
{
	return a.x*b.x + a.y*b.y;
}

cv::Point3f Math3dGeom::norm(cv::Point3f pt)
{
	return div(pt, sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z));
}

cv::Mat Math3dGeom::pt2mat(cv::Point3f pt)
{
	cv::Mat mat(1,3, CV_64F);
	mat.at<double>(0, 0) = pt.x;
	mat.at<double>(0, 1) = pt.y;
	mat.at<double>(0, 2) = pt.z;
	return mat;
}

cv::Mat Math3dGeom::pt2mat(cv::Point2f pt)
{
	cv::Mat mat(1,2, CV_64F);
	mat.at<double>(0, 0) = pt.x;
	mat.at<double>(0, 1) = pt.y;
	return mat;
}


cv::Point3f Math3dGeom::mat2pt(cv::Mat &m)
{
	if (m.rows == 3 && m.cols == 1)
		return cv::Point3f(m.at<double>(0,0), m.at<double>(1,0), m.at<double>(2,0));
	else if (m.cols == 3 && m.rows == 1)
		return cv::Point3f(m.at<double>(0,0), m.at<double>(0, 1), m.at<double>(0,2));
	return cv::Point3f(0,0,0);
}

cv::Point2f Math3dGeom::mat2pt2(cv::Mat &m)
{
	if (m.cols == 3)
		return cv::Point2f(m.at<double>(0,0) / m.at<double>(0,2), m.at<double>(0, 1) / m.at<double>(0,2));
	else
		return cv::Point2f(m.at<double>(0,0), m.at<double>(0, 1));
}

double Math3dGeom::mag(cv::Point3f pt)
{
	return sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
}

double Math3dGeom::magSqr(cv::Point pt)
{
	return ((double)pt.x*pt.x + pt.y*pt.y);
}

double Math3dGeom::magSqr(cv::Point3f pt)
{
	return (pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
}

double Math3dGeom::mag(cv::Point pt)
{
	return sqrt((double)pt.x*pt.x + pt.y*pt.y);
}

double Math3dGeom::mag(cv::Point2f pt)
{
	return sqrt(pt.x*pt.x + pt.y*pt.y);
}


double Math3dGeom::distanceLinePlane(cv::Point3f ptLine1, cv::Point3f ptLine2, cv::Point3f pt)
{
	return mag(cross(ptLine2 - ptLine1, ptLine1 - pt)) / mag(ptLine2-ptLine1);
}

cv::Point3f Math3dGeom::intersectPointLine(cv::Point3f ptLine1, cv::Point3f ptLine2, cv::Point3f pt)
{
	double u = ((pt.x - ptLine1.x) * (ptLine2.x - ptLine1.x)) + ((pt.y - ptLine1.y) * (ptLine2.y - ptLine1.y)) + ((pt.z - ptLine1.z) * (ptLine2.z - ptLine1.z));
	double dist = mag(ptLine1-ptLine2);
	u = u/(dist*dist);

	cv::Point3f t(0, 0, 0);
	t.x = ptLine1.x + u * (ptLine2.x - ptLine1.x);
	t.y = ptLine1.y + u * (ptLine2.y - ptLine1.y);
	t.z = ptLine1.z + u * (ptLine2.z - ptLine1.z);
	return t;
}

// Taken from: http://paulbourke.net/geometry/spherefrom4/
// Actually seems slightly slower than fitSphereFromNPoints (with N = 4 and LU instead of SVD), but this one seems to give more accurate results for N = 4
int Math3dGeom::fitSphereFrom4Pts(cv::Point3f p0, cv::Point3f p1, cv::Point3f p2, cv::Point3f p3, cv::Point3f &center, double &radius)
{
	cv::Point3f p[4] = {p0, p1, p2, p3};
	cv::Mat a(4, 4, CV_64F);
	double m11 = 0, m12 = 0, m13 = 0, m14 = 0, m15 = 0;
	int i = 0;

	radius = -1;
	center = cv::Point3f(0, 0, 0);

	/* Find determinant M11 */
	for (i=0;i<4;i++) {
		a.at<double>(i,0) = p[i].x;
		a.at<double>(i,1) = p[i].y;
		a.at<double>(i,2) = p[i].z;
		a.at<double>(i,3) = 1;
	}
	m11 = cv::determinant(a);

	/* Find determinant M12 */    
	for (i=0;i<4;i++) {
		a.at<double>(i,0) = p[i].x*p[i].x + p[i].y*p[i].y + p[i].z*p[i].z;
		a.at<double>(i,1) = p[i].y;
		a.at<double>(i,2) = p[i].z;
		a.at<double>(i,3) = 1;
	}
	m12 = cv::determinant(a);

	/* Find determinant M13 */    
	for (i=0;i<4;i++) {
		a.at<double>(i,0) = p[i].x;
		a.at<double>(i,1) = p[i].x*p[i].x + p[i].y*p[i].y + p[i].z*p[i].z;
		a.at<double>(i,2) = p[i].z;
		a.at<double>(i,3) = 1;
	}
	m13 = cv::determinant(a);

	/* Find determinant M14 */    
	for (i=0;i<4;i++) {
		a.at<double>(i,0) = p[i].x;
		a.at<double>(i,1) = p[i].y;
		a.at<double>(i,2) = p[i].x*p[i].x + p[i].y*p[i].y + p[i].z*p[i].z;
		a.at<double>(i,3) = 1;
	}
	m14 = cv::determinant(a);

	/* Find determinant M15 */    
	for (i=0;i<4;i++) {
		a.at<double>(i,0) = p[i].x*p[i].x + p[i].y*p[i].y + p[i].z*p[i].z;
		a.at<double>(i,1) = p[i].x;
		a.at<double>(i,2) = p[i].y;
		a.at<double>(i,3) = p[i].z;
	}
	m15 = cv::determinant(a);

	if (m11 == 0) {
		//fprintf(stderr,"The points don't define a sphere!\n");
		return 0;
	}

	center.x = 0.5 * m12 / m11;
	center.y = 0.5 * m13 / m11;
	center.z = 0.5 * m14 / m11;
	radius = sqrt(center.x*center.x + center.y*center.y + center.z*center.z - m15/m11);

	return 1;
}

int Math3dGeom::fitSphereFromNPts(const Array<cv::Point3f> &pts, cv::Point3f &center, double &radius)
{
	cv::Mat A(pts.size(), 4, CV_64F);
	cv::Mat b(pts.size(), 1, CV_64F);
	cv::Mat a(4, 1, CV_64F);

	for (int i = 0; i < pts.size(); i++)
	{
		double x = pts[i].x, y = pts[i].y, z = pts[i].z;
		A.at<double>(i, 0) = x;
		A.at<double>(i, 1) = y;
		A.at<double>(i, 2) = z;
		A.at<double>(i, 3) = 1;

		b.at<double>(i, 0) = -(x*x + y*y + z*z);
	}

	cv::solve(A, b, a, cv::DECOMP_SVD);

	center.x = -a.at<double>(0, 0) / 2;
	center.y = -a.at<double>(1, 0) / 2;
	center.z = -a.at<double>(2, 0) / 2;

	double x = center.x, y = center.y, z = center.z;
	radius = sqrt(x*x + y*y + z*z - a.at<double>(3,0));

	return 0;
}

int Math3dGeom::ransacSphereFit(const Array<cv::Point3f> &pts, int iterations, double minDist, cv::Point3f &center, double &radius)
{
	cv::Point3f c, bestc;
	double r, bestr;
	int bestScore = 0;
	int inliers = 0;
	int size = pts.size();

	center = cv::Point3f(0, 0, 0);
	radius = -1;

	if (size < 4)
		return 0;
	
	for (int i = 0; i < iterations; i++)
	{
		fitSphereFrom4Pts(pts[rand() % size], pts[rand() % size], pts[rand() % size], pts[rand() % size], c, r);

		inliers = 0;
		for (int j = 0; j < size; j++)
		{
			double x = c.x - pts[j].x, y = c.y - pts[j].y, z = c.z - pts[j].z;
			double dist = fabs(r - sqrt(x*x + y*y + z*z));
			if (dist < minDist)
				inliers++;
		}

		if (inliers > bestScore)
		{
			bestScore = inliers;
			bestc = c;
			bestr = r;
		}
	}

	c = bestc;
	r = bestr;
	Array<cv::Point3f> inlierPts;
	for (int j = 0; j < size; j++)
	{	
		double x = c.x - pts[j].x, y = c.y - pts[j].y, z = c.z - pts[j].z;
		double dist = fabs(r - sqrt(x*x + y*y + z*z));
		if (dist < minDist)
			inlierPts.add(pts[j]);
	}

	fitSphereFromNPts(inlierPts, center, radius);

	return inlierPts.size();
}

int Math3dGeom::intersectLineSphere(cv::Point3f center, double radius, cv::Point3f linePt, cv::Point3f lineUnitDirection, double &d1, double &d2)
{
	static FILE *fp = 0;//fopen("intersectLineSphere.txt", "wb");

#if 1
	cv::Point3f &D = lineUnitDirection;
	cv::Point3f &O = linePt;
	cv::Point3f &C = center;
	double r = radius;

    //Compute A, B and C coefficients
    double a = dot(D, D);//l.x*l.x + l.y*l.y* + l.z*l.z;//dot(ray.d, ray.d);
    double b = 2*dot(O-C, D);//2*(l.x*p.x + l.y*p.y + l.z*p.z);//2 * dot(ray.d, ray.o);
    double c = dot(O-C, O-C) - r*r;//p.x*p.x + p.y*p.y + p.;//dot(ray.o, ray.o) - (r * r);

    //Find discriminant
    double disc = b * b - 4 * a * c;
    
    // if discriminant is negative there are no real roots, so return 
    // false as ray misses sphere
    if (disc < 0)
        return 0;

    // compute q as described above
    double distSqrt = sqrtf(disc);
    double q;
    if (b < 0)
        q = (-b - distSqrt)/2.0;
    else
        q = (-b + distSqrt)/2.0;

    // compute t0 and t1
    float t0 = q / a;
    float t1 = c / q;

    // make sure t0 is smaller than t1
    if (t0 > t1)
    {
        // if t0 is bigger than t1 swap them around
        float temp = t0;
        t0 = t1;
        t1 = temp;
    }

    // if t1 is less than zero, the object is in the ray's negative direction
    // and consequently the ray misses the sphere
    if (t1 < 0)
        return 0;

    // if t0 is less than zero, the intersection point is at t1
    //if (t0 < 0)
    //{
    //    d1 = t1;
    //    return 1;
    //}
    //// else the intersection point is at t0
    //else
    //{
    //    d1 = t0;
    //    return 1;
    //}

	//if (fp) fprintf(fp, "%f %f %f  %f %f %f  %f %f %f  %f  %f\n", 
	//	l.x, l.y, l.z, p.x, p.y, p.z, c.x, c.y, c.z, radius, SQRT);
	//if (fp) fflush(fp);

	d1 = t0;
	d2 = t1;
	return 2;

#else
	cv::Point3f &l = lineUnitDirection;
	cv::Point3f &p = linePt;
	cv::Point3f &c = center;

	double LC = c.x*l.x + c.y*l.y + c.z*l.z;
	double CSQR = c.x*c.x + c.y*c.y + c.z*c.z;
	double R = radius;
	double SQRT = LC*LC - CSQR + R*R;
	if (fp) fprintf(fp, "%f %f %f  %f %f %f  %f %f %f  %f  %f\n", 
		l.x, l.y, l.z, p.x, p.y, p.z, c.x, c.y, c.z, radius, SQRT);
	if (fp) fflush(fp);
	if (SQRT < 0)
	{
		return 0;
	}
	else if (SQRT == 0)
	{
		d1 = LC;
		return 1;
	}
	else
	{
		d1 = LC + sqrt(SQRT);
		d2 = LC - sqrt(SQRT);
		return 2;
	}
#endif
}

int Math3dGeom::intersectLineSphere(cv::Point3f center, double radius, cv::Point3f linePt, cv::Point3f lineUnitDirection, cv::Point3f &pt1, cv::Point3f &pt2)
{
	cv::Point3f &l = lineUnitDirection;
	cv::Point3f &p = linePt;
	cv::Point3f &c = center;
	double LC = c.x*l.x + c.y*l.y + c.z*l.z;
	double C = c.x*c.x + c.y*c.y + c.z*c.z;
	double R = radius;
	double SQRT = LC*LC - C*C + R*R;
	if (SQRT < 0)
	{
		return 0;
	}
	else if (SQRT == 0)
	{
		double d1 = LC;
		pt1 = linePt + lineUnitDirection*d1;
		return 1;
	}
	else
	{
		double d1 = LC + sqrt(SQRT);
		double d2 = LC - sqrt(SQRT);
		pt1 = linePt + lineUnitDirection*d1;
		pt2 = linePt + lineUnitDirection*d2;
		return 2;
	}
}


float Math3dGeom::fastDot(const cv::Point3f a, const cv::Point3f b)
{
	return a.x*b.x + a.y*b.y + a.z*b.z;
}

cv::Point3f Math3dGeom::fastCross(const cv::Point3f a, const cv::Point3f b)
{
	return cv::Point3f(a.y*b.z - a.z*b.y, -(a.x*b.z - a.z*b.x), a.x*b.y - a.y*b.x);
}

float Math3dGeom::distancePointToPlane(const cv::Point3f planePt, const cv::Point3f planeNormal, const cv::Point3f pt)
{
	//return (fastDot(planeNormal, planePt) - fastDot(planeNormal, pt));
	return fabs(fastDot(planeNormal, pt - planePt));
}

float Math3dGeom::signedDistancePointToPlane(const cv::Point3f planePt, const cv::Point3f planeNormal, const cv::Point3f pt)
{
	//return (fastDot(planeNormal, planePt) - fastDot(planeNormal, pt));
	return fastDot(planeNormal, pt - planePt);
}

void Math3dGeom::estimatePlane(const Array<cv::Point3f> &points, cv::Point3f &normal, double &offset, cv::Point3f &planePt)
{
	cv::Mat m, eigenVectors;
	cv::Mat eigenValues;
  
	cv::Point3f p0(0.0,0.0,0.0), p;
	double n = double(points.size());
  
	double sum_xx = 0.0, sum_yy = 0.0, sum_zz = 0.0, sum_xy = 0.0, sum_xz = 0.0, sum_yz = 0.0;
  
	for (int i = 0; i < points.size(); i++)
	{
		p0 += points[i];
	}

	p0 = div(p0, n);

	// Compute statistics required to construct the MI tensor
	for(int i=0; i<points.size(); i++){
		//p0 += vector3d(V3COMP(points[i]));
		p = points[i];//-p0;
		sum_xx += sq(p.x);
		sum_yy += sq(p.y);
		sum_zz += sq(p.z);
		sum_xy += p.x*p.y;
		sum_xz += p.x*p.z;
		sum_yz += p.y*p.z;
	}
  
	double invN = 1.0/n;

	m.create(3, 3, CV_64F);
  
	m.at<double>(0,0) = invN*sum_xx - p0.x*p0.x;
	m.at<double>(0,1) = invN*sum_xy - p0.x*p0.y;
	m.at<double>(0,2) = invN*sum_xz - p0.x*p0.z;
	m.at<double>(1,0) = invN*sum_xy - p0.x*p0.y;
	m.at<double>(1,1) = invN*sum_yy - p0.y*p0.y;
	m.at<double>(1,2) = invN*sum_yz - p0.y*p0.z;
	m.at<double>(2,0) = invN*sum_xz - p0.x*p0.z;
	m.at<double>(2,1) = invN*sum_yz - p0.y*p0.z;
	m.at<double>(2,2) = invN*sum_zz - p0.z*p0.z;

	cv::eigen(m, eigenValues, eigenVectors);
	cv::SVD svd;
	svd(m);

	normal.x = svd.u.at<double>(0,2);
	normal.y = svd.u.at<double>(1,2);
	normal.z = svd.u.at<double>(2,2);

	planePt = p0;
	return;
}

void Math3dGeom::estimatePlane(const Array<cv::Point3f> &points, cv::Point3f &normal, double &offset, cv::Point3f &planePt, cv::Mat &eigenVectors)
{
	cv::Mat m;
	cv::Mat eigenValues;
  
	cv::Point3f p0(0.0,0.0,0.0), p;
	double n = double(points.size());
  
	double sum_xx = 0.0, sum_yy = 0.0, sum_zz = 0.0, sum_xy = 0.0, sum_xz = 0.0, sum_yz = 0.0;
  
	for (int i = 0; i < points.size(); i++)
	{
		p0 += points[i];
	}

	p0 = div(p0, n);

	// Compute statistics required to construct the MI tensor
	for(int i=0; i<points.size(); i++){
		//p0 += vector3d(V3COMP(points[i]));
		p = points[i];//-p0;
		sum_xx += sq(p.x);
		sum_yy += sq(p.y);
		sum_zz += sq(p.z);
		sum_xy += p.x*p.y;
		sum_xz += p.x*p.z;
		sum_yz += p.y*p.z;
	}
  
	double invN = 1.0/n;

	m.create(3, 3, CV_64F);
  
	m.at<double>(0,0) = invN*sum_xx - p0.x*p0.x;
	m.at<double>(0,1) = invN*sum_xy - p0.x*p0.y;
	m.at<double>(0,2) = invN*sum_xz - p0.x*p0.z;
	m.at<double>(1,0) = invN*sum_xy - p0.x*p0.y;
	m.at<double>(1,1) = invN*sum_yy - p0.y*p0.y;
	m.at<double>(1,2) = invN*sum_yz - p0.y*p0.z;
	m.at<double>(2,0) = invN*sum_xz - p0.x*p0.z;
	m.at<double>(2,1) = invN*sum_yz - p0.y*p0.z;
	m.at<double>(2,2) = invN*sum_zz - p0.z*p0.z;

	//cv::eigen(m, eigenValues, eigenVectors);
	cv::SVD svd;
	svd(m);

	normal.x = svd.u.at<double>(0,2);
	normal.y = svd.u.at<double>(1,2);
	normal.z = svd.u.at<double>(2,2);

	svd.u.copyTo(eigenVectors);

	
	planePt = p0;
	return;
}

void Math3dGeom::estimatePlane(std::vector<cv::Point3f> &points, cv::Point3d &normal, cv::Point3f &planePt, cv::Mat &eigenVectors)
{
	cv::Mat m;
	cv::Mat eigenValues;
  
	cv::Point3d p0(0.0,0.0,0.0), p;
	

	double n = double(points.size());
	double invN = 1.0/n;
	
	double sum_xx = 0.0, sum_yy = 0.0, sum_zz = 0.0, sum_xy = 0.0, sum_xz = 0.0, sum_yz = 0.0;
  
	for (int i = 0; i < points.size(); i++)
	{
		//p0 += invN*points[i];
		p0.x += invN*points[i].x;
		p0.y += invN*points[i].y;
		p0.z += invN*points[i].z;
	}

	//p0 = div(p0, n);


	// Compute statistics required to construct the MI tensor
	for(int i=0; i<points.size(); i++){
		//p0 += vector3d(V3COMP(points[i]));
		p = points[i];//-p0;
		sum_xx += sq(p.x);
		sum_yy += sq(p.y);
		sum_zz += sq(p.z);
		sum_xy += p.x*p.y;
		sum_xz += p.x*p.z;
		sum_yz += p.y*p.z;
	}
  
	

	m.create(3, 3, CV_64F);
  
	m.at<double>(0,0) = invN*sum_xx - p0.x*p0.x;
	m.at<double>(0,1) = invN*sum_xy - p0.x*p0.y;
	m.at<double>(0,2) = invN*sum_xz - p0.x*p0.z;
	m.at<double>(1,0) = invN*sum_xy - p0.x*p0.y;
	m.at<double>(1,1) = invN*sum_yy - p0.y*p0.y;
	m.at<double>(1,2) = invN*sum_yz - p0.y*p0.z;
	m.at<double>(2,0) = invN*sum_xz - p0.x*p0.z;
	m.at<double>(2,1) = invN*sum_yz - p0.y*p0.z;
	m.at<double>(2,2) = invN*sum_zz - p0.z*p0.z;

	//cv::eigen(m, eigenValues, eigenVectors);
	cv::SVD svd;
	svd(m);

	normal.x = svd.u.at<double>(0,2);
	normal.y = svd.u.at<double>(1,2);
	normal.z = svd.u.at<double>(2,2);
	if(normal.z < 0) 
		normal = -normal;

	svd.u.copyTo(eigenVectors);

	
	planePt = p0;
	return;
}

cv::Point3d Math3dGeom::calcVecMean(const std::vector<cv::Point3f>& points) 
{ 
	cv::Point3d mean(0.0,0.0,0.0);

	int nPts = points.size();
	for(int i = 0; i < nPts; i++) 
	{ 
		mean += (cv::Point3d) points[i]; 
		//mean += points[i]; 
		//mean.x += points[i].x;
		//mean.y += points[i].y;
		//mean.z += points[i].z;

	} 

	mean = mean*double(1.0/nPts); 
	return  mean; 
} 
cv::Point2d Math3dGeom::calcVecMean(const std::vector<cv::Point2f>& points) 
{ 
	cv::Point2d mean(0.0,0.0);
	int nPts = points.size();
	for(int i = 0; i < nPts; i++) 
	{ 
		mean += (cv::Point2d) points[i]; 
		//mean += points[i]; 
		//mean.x += points[i].x;
		//mean.y += points[i].y;
	} 

	mean = mean*double(1.0/nPts); 
	return mean; 
} 

double Math3dGeom::calcVecMean(const std::vector<double>& vals) 
{ 
	double mean = 0.0;

	int nPts = vals.size();
	for(int i = 0; i < nPts; i++) 
	{ 
		mean += vals[i]; 

	} 

	mean = mean*double(1.0/nPts); 
	return mean; 
	
	//double sum = std::accumulate(v.begin(), v.end(), 0.0);
	//double mean = sum / v.size();
} 
double Math3dGeom::calcVecStd(const std::vector<double>& vals) 
{ 
	double mean = 0.0;

	int nPts = vals.size();
	for(int i = 0; i < nPts; i++) 
	{ 
		mean += vals[i]; 

	} 

	mean = mean*double(1.0/nPts); 

	double sqSum = 0.0;
	for(int i = 0; i < nPts; i++) 
	{ 
		sqSum += (mean - vals[i])*(mean - vals[i]); 

	} 

	sqSum = sqrt(sqSum*double(1.0/(nPts-1))); 
	return sqSum; 

} 

std::vector<int> Math3dGeom::randperm(int n)
{
	int i, j, t;
	std::vector<int> perm;

	perm.reserve(n);
	for(i=0; i<n; i++)
		perm.push_back(i);

	for(i=0; i<n; i++)
	{
		j = rand()%(n-i)+i;
		t = perm[j];
		perm[j] = perm[i];
		perm[i] = t;
	}

	return perm;
}
cv::Mat Math3dGeom::estimate3dTransform(std::vector<cv::Point3f> srcArray, std::vector<cv::Point3f> dstArray, int nMaxPts)
{

	std::vector<int> idx = randperm(nMaxPts);
	std::vector<cv::Point3f> sampledSrcArray, sampledDstArray;

	

	if(srcArray.size() > nMaxPts)
	{
		for(int i =0 ; i < idx.size(); i++)
		{
			sampledSrcArray.push_back(srcArray[idx[i]]);
			sampledDstArray.push_back(dstArray[idx[i]]);

		}
	}
	else
	{
		sampledSrcArray = srcArray;
		sampledDstArray = dstArray;
	}
	return estimate3dTransform(sampledSrcArray, sampledDstArray);

}
cv::Mat Math3dGeom::estimate3dTransform(std::vector<cv::Point3f> srcArray, std::vector<cv::Point3f> dstArray)
{
	cv::Mat TR = cv::Mat::eye(4,4, CV_64F);
	cv::Mat srcMat = cv::Mat(srcArray).reshape(1);
	srcMat.convertTo(srcMat, CV_64F);


	cv::Mat dstMat = cv::Mat(dstArray).reshape(1);
	dstMat.convertTo(dstMat, CV_64F);


	/*http://nghiaho.com/?page_id=671, http://nghiaho.com/uploads/code/rigid_transform_3D.m*/
	
	//1. Find centroid

	cv::Mat centroidSrcMat(calcVecMean(srcArray));
	cv::Mat centroidDstMat(calcVecMean(dstArray));

	cv::Mat opSrc = srcMat - cv::repeat(centroidSrcMat.t(), srcMat.rows, 1);
	cv::Mat opDst = dstMat - cv::repeat(centroidDstMat.t(), dstMat.rows, 1);


	//2. Find Rotation..
	cv::SVD svd(opSrc.t()*opDst);
	cv::Mat Ut, V;
	Ut = svd.u.t();
	V = svd.vt.t();
	
	cv::Mat Rot = V*Ut;
	if(cv::determinant(Rot)<0)
	{
		fprintf(stderr, "Reflection is detected!");
		/*V.at<double>(0,2) = -V.at<double>(0,2);
		V.at<double>(1,2) = -V.at<double>(1,2);
		V.at<double>(2,2) = -V.at<double>(2,2);
		cv::Mat newRot = V*Ut;
		newRot.copyTo(Rot);*/
		Rot.at<double>(0,2) = -Rot.at<double>(0,2);
		Rot.at<double>(1,2) = -Rot.at<double>(1,2);
		Rot.at<double>(2,2) = -Rot.at<double>(2,2);


	}
	



	//3. Find Transform.
	cv::Mat t = (-Rot*centroidSrcMat) + centroidDstMat;

	//4. Construction homogeneous transform matrix.

	Rot.copyTo(TR(cv::Range(0,3), cv::Range(0,3)));

	t.copyTo(TR(cv::Range(0,3), cv::Range(3,4)));


	/*_PrintResult("Transform");
		for (int i=0; i < 4; i++)
	{
		_PrintResult("Row %d: %f, %f, %f, %f", i, TR.at<double>(i,0), TR.at<double>(i,1), TR.at<double>(i,2), TR.at<double>(i,3));
	}*/

	

	
	return TR;

}



std::vector<cv::Point3f> Math3dGeom::apply3dTransform(cv::Mat TR /*CV_64F*/, std::vector<cv::Point3f> srcArray)
{
	std::vector<cv::Point3d> dstArray;


	/*int time = timeGetTime();
	cv::Mat Rot = TR(cv::Range(0,3), cv::Range(0,3));
	cv::Mat t = TR(cv::Range(0,3), cv::Range(3,4));

	for (int i =0 ; i < srcArray.size(); i++)
	{
		cv::Mat pt(srcArray[i]); 
		pt.convertTo(pt, CV_64F);
		cv::Mat test = Rot*pt+t;
		std::vector<cv::Point3d> ptvec = cv::Mat_<cv::Point3d>(test);

	}
	_PrintResult("Indv. Transform: %d msec.", timeGetTime()-time);


	time = timeGetTime();*/
	cv::Mat srcMat = cv::Mat(srcArray).reshape(1);
	srcMat.convertTo(srcMat, CV_64F);
	cv::Mat opMat= cv::Mat::ones(srcMat.rows, 4, CV_64F);
	srcMat.copyTo(opMat.colRange(cv::Range(0,3)));

	cv::Mat resMat = TR*opMat.t();
	
	cv::Mat res = resMat.rowRange(cv::Range(0,3));
	dstArray = cv::Mat_<cv::Point3d>(res);
	//_PrintResult("All. Transform: %d msec.", timeGetTime()-time);
	//return dstArray;


	std::vector<cv::Point3f> dstArrayf;
	for(int i=0; i < dstArray.size(); i++)
		dstArrayf.push_back(dstArray[i]);

	return  dstArrayf;
}

std::vector<cv::Point2f> Math3dGeom::apply2dTransform(cv::Mat TR /*CV_32F*/, std::vector<cv::Point2f> srcArray)
{
	std::vector<cv::Point2f> dstArray;

	cv::Mat srcMat = cv::Mat(srcArray).reshape(1);
	//srcMat.convertTo(srcMat, CV_64F);
	cv::Mat opMat= cv::Mat::ones(srcMat.rows, 3, CV_32F);
	srcMat.copyTo(opMat.colRange(cv::Range(0,2)));

	cv::Mat resMat = TR*opMat.t();
	
	cv::Mat res = resMat.rowRange(cv::Range(0,2));
	dstArray = cv::Mat_<cv::Point2f>(res);
	//_PrintResult("All. Transform: %d msec.", timeGetTime()-time);
	//return dstArray;

	return  dstArray;
}
cv::Point2f Math3dGeom::apply2dTransform(cv::Mat TR, cv::Point2f srcPt)
{
	cv::Point2f dstPt;

	cv::Mat srcMat = cv::Mat(srcPt).reshape(1);
	srcMat.convertTo(srcMat, CV_64FC1);
	TR.convertTo(TR, CV_64FC1);

	cv::Mat dstMat = TR*srcMat;

	dstPt.x = (float)dstMat.at<double>(0,0);
	dstPt.y = (float)dstMat.at<double>(1,0);

	return  dstPt;
}


std::vector<cv::Point2f> Math3dGeom::randSampling(std::vector<cv::Point2f> &srcArray, int maxPts)
{
	//random sampling to reduce the number of points 
	std::vector<int> idx = randperm(maxPts);

	std::vector<cv::Point2f> sampledSrcArray;

	if(srcArray.size() > maxPts)
	{
		for(int i =0 ; i < idx.size(); i++)
		{
			sampledSrcArray.push_back(srcArray[idx[i]]);

		}
	}
	else
		sampledSrcArray = srcArray;

	return sampledSrcArray;
}


//template <typename T> std::vector<T> Math3dGeom::randSampling(std::vector<T> srcArray, int maxPts)
//{
//	//random sampling to reduce the number of points 
//	//int nMaxPts = 500;
//	std::vector<int> idx = randperm(maxPts);
//
//	std::vector<T> sampledSrcArray;
//
//	if(srcArray.size() > nMaxPts)
//	{
//		for(int i =0 ; i < idx.size(); i++)
//		{
//			sampledSrcArray.push_back(srcArray[idx[i]]);
//
//		}
//	}
//	else
//		sampledSrcArray = srcArray;
//
//	return srcArray;
//}

/**
 From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
 */
Mat_<double> Math3dGeom::LinearLSTriangulation(Point3d u,       //homogenous image point (u,v,1)
                   Matx34d P,       //camera 1 matrix
                   Point3d u1,      //homogenous image point in 2nd camera
                   Matx34d P1       //camera 2 matrix
                                   )
{
    //build matrix A for homogenous equation system Ax = 0
    //assume X = (x,y,z,1), for Linear-LS method
    //which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1
    cv::Matx43d A(u.x*P(2,0)-P(0,0),    u.x*P(2,1)-P(0,1),      u.x*P(2,2)-P(0,2),
          u.y*P(2,0)-P(1,0),    u.y*P(2,1)-P(1,1),      u.y*P(2,2)-P(1,2),
          u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),   u1.x*P1(2,2)-P1(0,2),
          u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),   u1.y*P1(2,2)-P1(1,2));

    Mat_<double> B/*cv::Mat B*/ = (
		Mat_<double>(4,1) <<    -(u.x*P(2,3)    -P(0,3)), -(u.y*P(2,3)  -P(1,3)), -(u1.x*P1(2,3)    -P1(0,3)), -(u1.y*P1(2,3)    -P1(1,3))
		);
 
    //cv::Mat X;
	Mat_<double> X;
    solve(A,B,X,DECOMP_SVD);
 
    return X;
	
}



/**
 From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
 http://www.morethantechnical.com/2012/01/04/simple-triangulation-with-opencv-from-harley-zisserman-w-code/#more-1023
 */

Mat_<double>  Math3dGeom::IterativeLinearLSTriangulation(Point3d u,    //homogenous image point (u,v,1)
                                            Matx34d P,          //camera 1 matrix
                                            Point3d u1,         //homogenous image point in 2nd camera
                                            Matx34d P1          //camera 2 matrix
                                            ) {
	double EPSILON = 0;

    double wi = 1, wi1 = 1;
    Mat_<double> X(4,1); 
    for (int i=0; i<10; i++) { //Hartley suggests 10 iterations at most
        Mat_<double> X_ = LinearLSTriangulation(u,P,u1,P1);

		
        X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); 
		X(3) = 1.0;


		//X.at<double>(0) = X_.at<double>(0);
		//X.at<double>(1) = X_.at<double>(1);
		//X.at<double>(2) = X_.at<double>(2);
		//X_.at<double>(3) = 1.0;

		        //recalculate weights
        double p2x = Mat_<double>(Mat_<double>(P).row(2)*X)(0);
        double p2x1 = Mat_<double>(Mat_<double>(P1).row(2)*X)(0);
         

         

        //breaking point
		if(fabsf(wi - p2x) <= EPSILON && fabsf(wi1 - p2x1) <= EPSILON)
		{
			fprintf(stderr, "Breaking Point: %d!!", i);
			break;
		}
         
        wi = p2x;
        wi1 = p2x1;
         
        //reweight equations and solve
        Matx43d A((u.x*P(2,0)-P(0,0))/wi,       (u.x*P(2,1)-P(0,1))/wi,         (u.x*P(2,2)-P(0,2))/wi,     
                  (u.y*P(2,0)-P(1,0))/wi,       (u.y*P(2,1)-P(1,1))/wi,         (u.y*P(2,2)-P(1,2))/wi,     
                  (u1.x*P1(2,0)-P1(0,0))/wi1,   (u1.x*P1(2,1)-P1(0,1))/wi1,     (u1.x*P1(2,2)-P1(0,2))/wi1, 
                  (u1.y*P1(2,0)-P1(1,0))/wi1,   (u1.y*P1(2,1)-P1(1,1))/wi1,     (u1.y*P1(2,2)-P1(1,2))/wi1
                  );
        Mat_<double> B = (Mat_<double>(4,1) <<    -(u.x*P(2,3)    -P(0,3))/wi,
                          -(u.y*P(2,3)  -P(1,3))/wi,
                          -(u1.x*P1(2,3)    -P1(0,3))/wi1,
                          -(u1.y*P1(2,3)    -P1(1,3))/wi1
                          );
         
        solve(A,B,X_,DECOMP_SVD);
        X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); //X_(3) = 1.0;
		X(3) = 1.0;
    }
    return X;
}


//cv::Point2f Math3dGeom::perspectiveTransform(cv::Point2f pt,  cv::Mat H)
 void Math3dGeom::perspectiveTransform(cv::Point2f &pt,  cv::Point2f &dst, cv::Mat H)
{
	cv::Mat src;
	cv::Point2f srcPt = pt;

	if(H.type() == CV_64F)
	{
		src = cv::Mat(3,1,CV_64F);
		src.at<double>(0,0) = (double) srcPt.x;
		src.at<double>(1,0) = (double) srcPt.y;
		src.at<double>(2,0) = 1.0f;
	}
	else
	{
		src = cv::Mat(3,1,CV_32F);
		src.at<float>(0,0) = (float) srcPt.x;
		src.at<float>(1,0) = (float) srcPt.y;
		src.at<float>(2,0) = 1.0f;
	}


	double data[2];
	cv::Mat tmpMat = H*src;
	if(H.type() == CV_64F)
	{
		double den2 = 1.0/(tmpMat.at<double>(2)+1e-10);
		tmpMat = den2*tmpMat;
		data[0] = tmpMat.at<double>(0);
		data[1] = tmpMat.at<double>(1);
	}
	else
	{
		double den2 = 1.0/(tmpMat.at<float>(2)+1e-10);
		tmpMat = den2*tmpMat;
		data[0] = tmpMat.at<float>(0);
		data[1] = tmpMat.at<float>(1);
		
	}
	dst = cv::Point2f((float)data[0], (float)data[1]);


};

//cv::Point3f Math3dGeom::perspectiveTransform(cv::Point3f pt,  cv::Mat H)
void Math3dGeom::perspectiveTransform(cv::Point3f &pt,  cv::Point3f &dst, cv::Mat H)
{
	cv::Mat src;
	cv::Point3f srcPt = pt;
	if(H.type() == CV_64F)
	{
		src = cv::Mat(4,1,CV_64F);
		src.at<double>(0,0) = (double) srcPt.x;
		src.at<double>(1,0) = (double) srcPt.y;
		src.at<double>(2,0) = (double) srcPt.z;
		src.at<double>(3,0) = 1.0f;
	}
	else
	{
		src = cv::Mat(4,1,CV_32F);
		src.at<float>(0,0) = (float) srcPt.x;
		src.at<float>(1,0) = (float) srcPt.y;
		src.at<float>(2,0) = (float) srcPt.z;
		src.at<float>(3,0) = 1.0f;
	}

	

	//cv::Mat x =(H.row(0)*src)/(H.row(3)*src); 

	//cv::Mat y =(H.row(1)*src)/(H.row(3)*src); 

	//cv::Mat z =(H.row(2)*src)/(H.row(3)*src); 

	//cv::Point3f res;
	//if(H.type() == CV_64F)
	//	res = cv::Point3f((float)x.at<double>(0,0), (float) y.at<double>(0,0), (float) z.at<double>(0,0));
	//else
	//	res = cv::Point3f(x.at<float>(0,0),  y.at<float>(0,0), z.at<float>(0,0));
		//return res;



	//cv::Mat den = H.row(3)*src;
	//double data[3];

	//for(int i = 0; i < 3; i++)
	//{
	//	cv::Mat num = H.row(i)*src;
	//	if(H.type() == CV_64F)
	//		data[i] = num.at<double>(0)/(den.at<double>(0)+1e-10);
	//	else
	//		data[i] = num.at<float>(0)/(den.at<float>(0)+1e-10);

	//}
	////cv::Point3f res = cv::Point3f(data[0], data[1], data[2]);
	//dst = cv::Point3f(data[0], data[1], data[2]);


	double data[3];
	cv::Mat tmpMat = H*src;
	if(H.type() == CV_64F)
	{
		double den2 = 1.0/(tmpMat.at<double>(3)+1e-10);
		tmpMat = den2*tmpMat;
		data[0] = tmpMat.at<double>(0);
		data[1] = tmpMat.at<double>(1);
		data[2] = tmpMat.at<double>(2);
	}
	else
	{
		double den2 = 1.0/(tmpMat.at<float>(3)+1e-10);
		tmpMat = den2*tmpMat;
		data[0] = tmpMat.at<float>(0);
		data[1] = tmpMat.at<float>(1);
		data[2] = tmpMat.at<float>(2);
		
	}
	dst = cv::Point3f((float)data[0], (float)data[1], (float)data[2]);
};

double Math3dGeom::calcDistVec(cv::Point2f v0, cv::Point2f v1)
{
	v0 = 1.0/(mag(v0)+1e-8)*v0;

	return fabs(mag(v1 - dot(v0,v1)*v0));
}

cv::Point3f Math3dGeom::projPtonPlane(cv::Point3f pt, cv::Point3f normal, cv::Point3f ptPlane)
{
	cv::Point3f v = pt - ptPlane;
	double mag = cv::norm(normal);
	if(mag > 0)
		normal = 1.0/mag*normal;
	cv::Point3f res = pt - dot(v, normal)*normal;
	return res;
}