	/*
	Skeletonization of a binary image.
	Black values (0) mean object and white values (1 = 255) background.
	Source: Parker, J.R. Algorithms for image processing and computer vision.
					New York, NY: John Wiley & Sons, 1997. 417p. pp. 203-218.

	Program adapted to C/OpenCV by Jose Iguelmar Miranda.
	March, 2010.
	I have also a Java version of this program.
	jose.ig.miranda@gmail.com
*/

#define NORTH 1
#define SOUTH 3

#include <cv.h>

class Skeletonize
{
public:
 

// 1-neighbors of pixel.
int nays8(IplImage *im, int r, int c) {
	CvScalar pixel;
	int blue, k = 0, i, j;

  for (i = r-1; i <= r+1; i++) 
		for (j = c-1; j <= c+1; j++) 
			if (i != r || c != j) {
				pixel = cvGet2D(im, i, j);
				blue = pixel.val[0];
				if (blue >= 1)
					k++;
			}

	return k;
}

int connectivity(IplImage *im, int r, int c) {
	int N = 0, b1, b2;
	CvScalar pixel;

	pixel = cvGet2D(im, r, c+1);
	b1 = pixel.val[0];
	pixel = cvGet2D(im, r-1, c+1);
	b2 = pixel.val[0];
	if (b1 >= 1 && b2 == 0) 
		N++;

	pixel = cvGet2D(im, r-1, c+1);
	b1 = pixel.val[0];
	pixel = cvGet2D(im, r-1, c);
	b2 = pixel.val[0];
	if (b1 >= 1 && b2 == 0)
		N++;

	pixel = cvGet2D(im, r-1, c);
	b1 = pixel.val[0];
	pixel = cvGet2D(im, r-1, c-1);
	b2 = pixel.val[0];
	if (b1 >= 1 && b2 == 0)
		N++;

	pixel = cvGet2D(im, r-1, c-1);
	b1 = pixel.val[0];
	pixel = cvGet2D(im, r, c-1);
	b2 = pixel.val[0];
	if (b1 >= 1 && b2 == 0)
		N++;

	pixel = cvGet2D(im, r, c-1);
	b1 = pixel.val[0];
	pixel = cvGet2D(im, r+1, c-1);
	b2 = pixel.val[0];
	if (b1 >= 1 && b2 == 0)
		N++;

	pixel = cvGet2D(im, r+1, c-1);
	b1 = pixel.val[0];
	pixel = cvGet2D(im, r+1, c);
	b2 = pixel.val[0];
	if (b1 >= 1 && b2 == 0)
		N++;

	pixel = cvGet2D(im, r+1, c);
	b1 = pixel.val[0];
	pixel = cvGet2D(im, r+1, c+1);
	b2 = pixel.val[0];
	if (b1 >= 1 && b2 == 0)
		N++;

	pixel = cvGet2D(im, r+1, c+1);
	b1 = pixel.val[0];
	pixel = cvGet2D(im, r, c+1);
	b2 = pixel.val[0];
	if (b1 >= 1 && b2 == 0)
		N++;

	return N;
}

void deleteCB(IplImage *im, IplImage *tmp) {
	int w, h, blue, i, j;
	CvScalar pixel;

	w = im->width;
	h = im->height;

	for (i = 1; i < h-1; i++)
		for (int j = 1; j < w-1; j++) {
			pixel = cvGet2D(tmp, i, j);
			blue = pixel.val[0];
			if (blue == 1) {
				pixel.val[0] = 0;
				cvSet2D(im, i, j, pixel);
				cvSet2D(tmp, i, j, pixel);
			}
		}
}

void stair(IplImage *im, IplImage *tmp, int dir) {
	int i, j, b1, b2, b3, b4, b5, b6, b7, b8, b9, w, h;
	CvScalar pixel;
	int N, S, E, W, NE, NW, SE, SW, C;

	w = im->width;
	h = im->height;

	if (dir == NORTH)
		for (i = 1; i < h-1; i++)
			for (j = 1; j < w-1; j++) {
				pixel = cvGet2D(im, i-1, j-1);
				b1 = pixel.val[0];
				pixel = cvGet2D(im, i-1, j);
				b2 = pixel.val[0];
				pixel = cvGet2D(im, i-1, j+1);
				b3 = pixel.val[0];
				pixel = cvGet2D(im, i, j-1);
				b4 = pixel.val[0];
				pixel = cvGet2D(im, i, j);
				b5 = pixel.val[0];
				pixel = cvGet2D(im, i, j+1);
				b6 = pixel.val[0];
				pixel = cvGet2D(im, i+1, j-1);
				b7 = pixel.val[0];
				pixel = cvGet2D(im, i+1, j);
				b8 = pixel.val[0];
				pixel = cvGet2D(im, i+1, j+1);
				b9 = pixel.val[0];
				if (b1 == 1)
					NW = 1;
				else
					NW = 0;
				if (b2 == 1)
					N = 1;
				else
					N = 0;
				if (b3 == 1)
					NE = 1;
				else
					NE = 0;
				if (b4 == 1)
					W = 1;
				else
					W = 0;
				if (b5 == 1)
					C = 1;
				else
					C = 0;
				if (b6 == 1)
					E = 1;
				else
					E = 0;
				if (b7 == 1)
					SW = 1;
				else
					SW = 0;
				if (b8 == 1)
					S = 1;
				else
					S = 0;
				if (b9 == 1)
					SE = 1;
				else
					SE = 0;

				if (dir == NORTH) {
					if (C && !(N && ((E && !NE && !SW && (!W || !S)) || 
						 (W && !NW && !SE && (!E || !S))))) {
						pixel.val[0] = 0;
						cvSet2D(tmp, i, j, pixel);
					} else {
						pixel.val[0] = 1;
						cvSet2D(tmp, i, j, pixel);
					}
				} else if (dir == SOUTH) {
					if (C && !(S && ((E && !SE && !NW && (!W || !N)) || 
						 (W && !SW && !NE && (!E || !N))))) {
						pixel.val[0] = 0;
						cvSet2D(tmp, i, j, pixel);
					} else {
						pixel.val[0] = 1;
						cvSet2D(tmp, i, j, pixel);
					}
				}
			}
}

// Zhang-Suen algorithm.
void skeletonize(IplImage *im) {
	int janelaAH[][2] = {
		{1, 0}, {0, -1}, {-1, 0}, {0, 1}
	};
	int janelaH[][2] = {
		{0, -1}, {1, 0}, {0, 1}, {-1, 0}
	};
	int aBlue[6];
	int w, h, i, v, j, k, blue, lin, col, iJanela, again = 1;
	CvScalar pixel, pixOut;	
	IplImage *tmp = 0;
	
	w = im->width;
	h = im->height;
	tmp = cvCreateImage(cvGetSize(im), IPL_DEPTH_8U, 1);
	
  for (i = 0; i < h; i++) {
		for (j = 0; j < w; j++) { 
			pixel = cvGet2D(im, i, j);
			blue = pixel.val[0];
			if (blue > 0)
				pixel.val[0] = 0;
			else
				pixel.val[0] = 1;
			cvSet2D(im, i, j, pixel);
			pixOut.val[0] = 0;
			cvSet2D(tmp, i, j, pixOut);
		}
	}

	while (again) {
		again = 0;
  	for (i = 1; i < h-1; i++) 
			for (j = 1; j < w-1; j++) { 
				pixel = cvGet2D(im, i, j);
				blue = pixel.val[0];
				if (blue != 1)
					continue;
				k = nays8(im, i, j);
				iJanela = 0;
				if ((k >= 2 && k <= 6) && connectivity(im, i, j) == 1) {
					for (v = 0; v < 6; v++) {
						col = j + janelaAH[iJanela][0];
						lin = i + janelaAH[iJanela][1];
						pixel = cvGet2D(im, lin, col);
						aBlue[v] = pixel.val[0];
						iJanela++;
						if (v == 2) 
							iJanela = 1;
					}
					if (aBlue[0]*aBlue[1]*aBlue[2] == 0 &&
							aBlue[3]*aBlue[4]*aBlue[5] == 0) {
						pixOut.val[0] = 1;
						cvSet2D(tmp, i, j, pixOut);
						again = 1;
					}
				}		// if ((k >= 2...
			}		// for (j = 1;...

			deleteCB(im, tmp);
			if (!again)
				break;

  	for (i = 1; i < h-1; i++) 
			for (j = 1; j < w-1; j++) { 
				pixel = cvGet2D(im, i, j);
				blue = pixel.val[0];
				if (blue != 1)
					continue;
				k = nays8(im, i, j);
				iJanela = 0;
				if ((k >= 2 && k <= 6) && connectivity(im, i, j) == 1) {
					for (v = 0; v < 6; v++) {
						col = j + janelaH[iJanela][0];
						lin = i + janelaH[iJanela][1];
						pixel = cvGet2D(im, lin, col);
						aBlue[v] = pixel.val[0];
						iJanela++;
						if (v == 2) 
							iJanela = 1;
					}
					if (aBlue[0]*aBlue[1]*aBlue[2] == 0 &&
							aBlue[3]*aBlue[4]*aBlue[5] == 0) {
						pixOut.val[0] = 1;
						cvSet2D(tmp, i, j, pixOut);
						again = 1;
					}
				}		// if ((k >= 2...
			}		// for (j = 1;...

		deleteCB(im, tmp);
	}		// while

	stair(im, tmp, NORTH);
	deleteCB(im, tmp);
	stair(im, tmp, SOUTH);
	deleteCB(im, tmp);

  for (i = 1; i < h-1; i++) 
		for (j = 1; j < w-1; j++) { 
			pixel = cvGet2D(im, i, j);
			blue = pixel.val[0];
			if (blue > 0)
				pixel.val[0] = 0;
			else
				pixel.val[0] = 255;
			cvSet2D(im, i, j, pixel);
		}
}		// End skeletonize

private:
};