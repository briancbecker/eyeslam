/*==================================================================================

    Filename:  cvcolortraining.cpp

    Copyright 2006 Daniel Barber                 
                   Robotics Laboratory at UCF
                   http://robotics.ucf.edu

    Program Contents:
    -------------------------------------------------------------------------------
    Contains functions for training/learning to recognize a specific color in
    different color spaces with a lookup table.  (High memory, high speed).
    -------------------------------------------------------------------------------

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Library General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

==================================================================================*/
#include <stdio.h>
#include <vector>
#include <cv.h>
#include <highgui.h>
#include "image.h"
#include "cvcolorclassifier.h"
#include "string.h"
#include "fileio.h"
#include "array.h"

//using namespace std;

int invalidParametersError()
{
    printf("Invalid Number of Parameters\n");
    printf("Usage: cvcolortrainer <color space> <scale> <sig big> <sig small> <input dir> <output file name>\n");
    printf("\t<color space> : 0 - RGB, 1 - HSV, 2 - YCrCb\n");
    printf("\t<color space scale> : [1-100]\n");
    printf("\t<sig big>     :  Sigma value for positive gaussian spheres\n");
    printf("\t<sig small>   :  Sigma value for negative gaussian spheres\n");
    printf("\t<input dir>   : Directory containing input images and mask images (bmp, jpg, png).\n"); 
    printf("\t                Mask files have same file name as input images with \"mask.\" at the beginning.\n");
    printf("\t<output file name> : Output file name for training data.\n");
    return 0;
}

int main(int argc, char **argv)
{
    if(argc < 7 || atoi(argv[1]) < 0 || atoi(argv[1]) > 2)
        return invalidParametersError();

    Image input, mask, output, output2;
    CvColorClassifier colorClassifier;
    String dir = argv[5];  
    String outFile = argv[6];
    double sigBig, sigSmall;
    Array<String> files;
    Array<String> temp;
    int createdWindows = 0;
    int learnedColor = 0;

    //  Add to the directory path if needed
    if(dir.size() == 0 || dir[dir.size() - 1] != '/')
        dir += '/';

    String sigmas = argv[3];
    if(!sigmas.toDouble(&sigBig))
        return invalidParametersError();
    sigmas = argv[4];
    if(!sigmas.toDouble(&sigSmall))
        return invalidParametersError();

    //  Load in any image files to use for training
    FileIO::scanDirForFiles("*.png", temp, dir, 1);
    files = temp;
    FileIO::scanDirForFiles("*.jpg", temp, dir, 1);
    for(unsigned int i = 0; i < temp.size(); i++)
        files.push_back(temp[i]);
    FileIO::scanDirForFiles("*.bmp", temp, dir, 1);
    for(unsigned int i = 0; i < temp.size(); i++)
        files.push_back(temp[i]);
	FileIO::scanDirForFiles("*.ppm", temp, dir, 1);
	for(unsigned int i = 0; i < temp.size(); i++)
		files.push_back(temp[i]);

    colorClassifier.setColorSpaceScale(atoi(argv[2]));

    //  If files have been loaded, try train on them
    for(unsigned int i = 0; i < files.size(); i++)
    {
        String maskFile = files[i];
        //  There should be a mask file with "mask." at the beginning
        //  and everything else the same name as input
        maskFile.insert(0, "mask.");        
        if(input.load(dir + files[i]) && mask.load(dir  + maskFile, 0))
        {
            //  Make sure they are the same size
            if(input.h() != mask.h() || input.w() != mask.w())
                continue;

            if(!createdWindows)
            {
                cvNamedWindow("Input", 1);
                cvNamedWindow("Training Mask", 1);
                cvNamedWindow("Output", 1);
                createdWindows = 1;
            }

            cvShowImage("Input", input);
            cvShowImage("Training Mask", mask); 
            printf("Training on image %s...", files[i]);
            
            if(colorClassifier.train(input, mask, sigBig, sigSmall, atoi(argv[1])))
                learnedColor = true;

            output.create(mask.size(), mask.d(), mask.channels());
			int start = clock();
            colorClassifier.classify(input, output, .15);
			printf("(%d ms) ", clock() - start);
            cvShowImage("Output", output);
            printf("Done! Completed %.2lf percent\n", (i*200.0)/files.size());
            cvWaitKey(500);
        }
    }
 
    
    if(learnedColor)
    {
        printf("Saving training data...");
        if(colorClassifier.save(outFile))
            printf("Complete!\n");
        else
            printf("Failed to save training file!\n");

		int total=0;
		int correct=0;
        printf("Re-Loading training data...");
        if(colorClassifier.load(outFile))
        {
            //  If files have been loaded, try train on them
            for(unsigned int i = 0; i < files.size(); i++)
            {
                String maskFile = files[i];
                //  There should be a mask file with "mask." at the beginning
                //  and everything else the same name as input
                maskFile.insert(0, "mask.");        
                if(input.load(dir + files[i]) && mask.load(dir  + maskFile, 0))
                {
                    //  Make sure they are the same size
                    if(input.h() != mask.h() || input.w() != mask.w())
                        continue;

                    if(!createdWindows)
                    {
                        cvNamedWindow("Input", 1);
                        cvNamedWindow("Training Mask", 1);
                        cvNamedWindow("Output", 1);
                        createdWindows = 1;
                    }

                    cvShowImage("Input", input);
                    cvShowImage("Training Mask", mask); 
                    printf("Classifying on image %s...", files[i]);
                    output.create(input.size(), input.d(), 1);
					//output2.create(input.size(), input.d(), 1);
					int start = clock();
                    colorClassifier.classify(input, output, .001);
					printf("(%d ms)", clock() - start);
					//cvSmooth(output, output2, CV_MEDIAN, 3);
					//cvCopy(output2, output);
                    
					for(int j=0;j<mask.bytes();j++)
					{
						if(mask.ptr()[j]==255)
						{
							total++;
							if(output.ptr()[j]==255)
							{
								correct++;
							}
						}
					}
					
					cvShowImage("Output", output);
                    printf("Done! Completed %.2lf percent\n", (i*200.0)/files.size());
					
					printf("Correct: %lf\n",(100.0*correct)/total);
                    
					cvWaitKey(2000);
                }
            }
 
            printf("Complete!\n");
        }
        else
            printf("Failed to load training file!\n");
    }
    
    cvWaitKey(0);
    cvDestroyWindow("Input");
    cvDestroyWindow("Output");
    cvDestroyWindow("Training Mask");
}

/* End of file */
