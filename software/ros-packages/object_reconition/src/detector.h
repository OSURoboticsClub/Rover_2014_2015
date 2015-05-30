#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/gpu/gpu.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/ml/ml.hpp>
#include "includes/SampleDetector.h"
#include "includes/helper.h"
#include "includes/LinearSVM.h"
using namespace std;
using namespace cv;

#define MODEL "stopsign.yaml" 

#ifndef procimg
#define procimg
int processImage(Mat &img);
#endif
