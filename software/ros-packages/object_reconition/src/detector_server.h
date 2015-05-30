#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stdexcept>

#include<sys/types.h>
#include<sys/socket.h>
#include<stdlib.h>
#include<unistd.h>
#include<netinet/in.h>

#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/gpu/gpu.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/ml/ml.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


#include "includes/LinearSVM.h"

using namespace std;
using namespace cv;

#define MODEL "stopsign.yaml" 

struct img_packet{
    int cols;
    int rows;
};

class DetectorServer{
    private:
    	/*
    	 * Variables for object recognition
    	 */
	    LinearSVM svm;
	    LinearSVM small_svm;
	    gpu::HOGDescriptor desc;
	    gpu::HOGDescriptor small_desc;
    	int img_count = 0;
	
    public:
	int processImage(Mat &img);
	DetectorServer();
};

