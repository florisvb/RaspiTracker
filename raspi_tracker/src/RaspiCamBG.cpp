/*

 Copyright (c) by Floris van Breugel,
 All rights reserved.

 License: http://www.opensource.org/licenses/bsd-license.php

*/

#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>

#include "RaspiCamCV.h"

#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"

#include "read_settings.h"

#define SHOWFRAMES 1

cv::Mat imgmat, bkgrnd, absdiff;
cv::Mat imgmat_draw, absdiff_draw; 
std::string param_value;

int main(int argc, const char** argv){
	// Read camera settings
	ReadSettings();

        RaspiCamCvCapture * capture = raspiCamCvCreateCameraCapture(0); // Index doesn't really matter
	
	if (SHOWFRAMES==1) {
		cvNamedWindow("bkgrnd", 1);
		cvNamedWindow("imgmat", 1);
		cvNamedWindow("frame", 1);
	}

	FILE *datafile;
	datafile = fopen("datafile.txt", "w+");
	
	std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
	int nframes = 0;
	while (nframes < NUMBER_OF_FRAMES) {

		//ros::param::get("shutter_speed", param_value); 
		//raspiCamCvSetCaptureProperty(capture, MMAL_PARAMETER_ISO, 100);
		//raspiCamCvSetCaptureProperty(capture, MMAL_PARAMETER_SHUTTER_SPEED, std::stoi(param_value) );

		std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> frame_time = std::chrono::duration_cast<std::chrono::duration<double>>(t0-start_time);

		IplImage* bigImage = raspiCamCvQueryFrame(capture);
		IplImage* image = cvCreateImage( cvSize((int)(320), (int)(240)), bigImage->depth, bigImage->nChannels);
		cvResize(bigImage, image, CV_INTER_NN);
		imgmat = cv::cvarrToMat(image);

		// Set background to current image
		if (nframes < 30) {
			bkgrnd = imgmat.clone(); 
		}

		absdiff = abs(imgmat - bkgrnd);
		
		cv::threshold(absdiff, absdiff, THRESHOLD, 255, cv::THRESH_BINARY);
		if (SHOWFRAMES==1) {
			absdiff_draw = absdiff.clone();
			imgmat_draw = imgmat.clone();
		}

		cv::vector<cv::vector<cv::Point> > contours; 
		cv::vector<cv::Vec4i> hierarchy;
		cv::findContours( absdiff, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
		
		/// Get the moments
		cv::vector<cv::Moments> mu(contours.size() );
		for( int i = 0; i < contours.size(); i++ )
		{ mu[i] = moments( contours[i], true ); }

		// count how many properly sized objects we have
		int n = 0;
		for( int i = 0; i < contours.size(); i++ )
		{ 
			if (mu[i].m00 > MINSIZE) {
				n ++;
			}
		}

		fprintf(datafile, "%d, %d, %f, ", nframes, n, frame_time);

		///  Get the mass centers:
		if (n>0)
		{
			int ni = 0;
			cv::vector<cv::Point2f> mc( n );
			int sz[n];
			for( int i = 0; i < contours.size(); i++ )
			{ 
				if (mu[i].m00 > MINSIZE) {
					mc[ni] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); 
					sz[ni] = mu[i].m00;
					ni ++;
				}
			}
		
			/// Draw centers
			for( int i = 0; i< n; i++ )
			{
				// Save centers to file
				fprintf(datafile, "%f, %f, %d, ", mc[i].x, mc[i].y, sz[i]);

				// Draw the centers
				if (SHOWFRAMES==1) {
					circle( imgmat_draw, mc[i], 6, (255,255,255), 2, 8, 0 );
					circle( imgmat_draw, mc[i], 4, (0,0,0), 2, 8, 0 );
					circle( imgmat_draw, mc[i], 8, (0,0,0), 2, 8, 0 );
				}
			}
		}
		fprintf(datafile, "\n");		

		/*************************************************/

		if (SHOWFRAMES==1) {
			cv::imshow("bkgrnd", bkgrnd);
			cv::imshow("imgmat", imgmat_draw);
			cv::imshow("frame", absdiff);
			cv::imshow("frame_threshed", absdiff_draw);
			cvWaitKey(1);
		}
		

		bkgrnd = bkgrnd*0.992 + 0.008*imgmat;
		
		
		nframes ++;


	}

	if (SHOWFRAMES==1) {
		cvDestroyWindow("RaspiCamTest");
	}
	
	raspiCamCvReleaseCapture(&capture);
	fclose(datafile);
	return 0;
}
