#ifndef NAVIMAGE_H
#define NAVIMAGE_H
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/gpu/gpu.hpp>

#define DEBUG
static const int TILT_THRESH = 35;
using namespace std;
using namespace cv;
using namespace cv::gpu;

class NavImage
{
 protected:
	GpuMat gray;  //gray frame
	GpuMat gpuFrame;
 public:
	Mat original, frame;
	NavImage();
	Mat getBinary(int thresh = 1, int maxThresh = 255);
	void applyGaussian(int size = 5);
	void findChessboard(int row, int column);
	//Mat applyBoundingBox(Mat frame);
};
#endif
