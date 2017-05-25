#include "cvutils.h"


int writeMat(const CvMat *mat, FILE *fp)
{
	if (!fp)
		return false;

	fprintf(fp, "%d %d\n", mat->rows, mat->cols);
	for (int i = 0; i < mat->rows; i++)
	{
		for (int j = 0; j < mat->cols; j++)
		{
			//fprintf(fp, "%lf ", (double)cvmGet(mat, i, j));
			fprintf(fp, "%.15e ", (double)cvmGet(mat, i, j));
		}
		fprintf(fp, "\n");
	}
	fprintf(fp, "\n");
	return true;
}

int writeMat(const CvMat *mat, const char *filename)
{
	int result = false;

	FILE *fp = fopen(filename, "wb");
	if (fp)
	{
		result = writeMat(mat, fp);
		fclose(fp);
	}

	return result;

}

int readMat(CvMat *&mat, const char *filename)
{
	int result = false;

	FILE *fp = fopen(filename, "rb");
	if (fp)
	{
		result = readMat(mat, fp);
		fclose(fp);
	}

	return result;
}

int readMat(CvMat *&mat, FILE *fp)
{
	int rows = 0, cols = 0;

	cvReleaseMat(&mat);

	if (!fp)
		return false;

	if (fscanf(fp, "%d %d\n", &rows, &cols) != 2)
		return false;

	mat = cvCreateMat(rows, cols, CV_64F);

	for (int i = 0; i < mat->rows; i++)
	{
		for (int j = 0; j < mat->cols; j++)
		{
			double result;
			fscanf(fp, "%lf ", &result);
			cvmSet(mat, i, j, (float)result);
		}
		fscanf(fp, "\n");
	}
	fscanf(fp, "\n");
	return true;
}

// cv::Mat versions

int writeMat(const cv::Mat &mat, FILE *fp)
{
	if (!fp)
		return false;

	fprintf(fp, "%d %d\n", mat.rows, mat.cols);
	for (int i = 0; i < mat.rows; i++)
	{
		for (int j = 0; j < mat.cols; j++)
		{
			//fprintf(fp, "%lf ", mat.at<double>(i,j));
			//fprintf(fp, "%.15f ", mat.at<double>(i,j));
			fprintf(fp, "%.15e ", mat.at<double>(i,j));
		}
		fprintf(fp, "\n");
	}
	fprintf(fp, "\n");
	return true;
}

int writeMat(const cv::Mat &mat, const char *filename)
{
	int result = false;

	FILE *fp = fopen(filename, "wb");
	if (fp)
	{
		result = writeMat(mat, fp);
		fclose(fp);
	}

	return result;

}

int readMat(cv::Mat &mat, const char *filename)
{
	int result = false;

	FILE *fp = fopen(filename, "rb");
	if (fp)
	{
		result = readMat(mat, fp);
		fclose(fp);
	}

	return result;
}

int readMat(cv::Mat &mat, FILE *fp)
{
	int rows = 0, cols = 0;

	mat = cv::Mat();

	if (!fp)
		return false;

	if (fscanf(fp, "%d %d\n", &rows, &cols) != 2)
		return false;

	//mat = cvCreateMat(rows, cols, CV_64F);
	mat.create(cv::Size(rows, cols), CV_64F);

	for (int i = 0; i < mat.rows; i++)
	{
		for (int j = 0; j < mat.cols; j++)
		{
			double result;
			fscanf(fp, "%lf ", &result);
			//cvmSet(mat, i, j, (float)result);
			mat.at<double>(i,j) = result;
		}
		fscanf(fp, "\n");
	}
	fscanf(fp, "\n");
	return true;
}

int copyMat(CvMat *src, CvMat *&dst)
{
	if (!src || !src->rows || !src->cols)
		return false;

	if (!dst || dst->rows != src->rows || dst->cols != src->cols || dst->type != src->type)
	{
		cvReleaseMat(&dst);
		dst = cvCreateMat(src->rows, src->cols, src->type);
	}

	cvCopy(src, dst);

	return true;
}