#ifndef CVUTILS_H
#define CVUTILS_H

#include <cv.h>

int writeMat(const CvMat *mat, FILE *fp);
int writeMat(const CvMat *mat, const char *filename);
int readMat(CvMat *&mat, const char *filename);
int readMat(CvMat *&mat, FILE *fp);

int writeMat(const cv::Mat &mat, FILE *fp);
int writeMat(const cv::Mat &mat, const char *filename);
int readMat(cv::Mat &mat, const char *filename);
int readMat(cv::Mat &mat, FILE *fp);

int copyMat(CvMat *src, CvMat *&dst);

#endif