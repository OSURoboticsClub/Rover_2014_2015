#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/gpu/gpu.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/ml/ml.hpp>
#include<math.h>
#include<pthread.h>
using namespace std;
using namespace cv;

#ifndef HELPERS
#define HELPERS 1
void calculate_features_from_input(char* imageFilename, vector<float>& featureVector,
        HOGDescriptor& hog); 

int calculate_features_from_input(Mat &image, vector<float>& featureVector,
        HOGDescriptor& hog); 

void flv2mat(vector<float> &values, Mat &cp);

int get_overlap(int x1, int y1, int x2, int y2, int width, int height, double scale);

#endif
