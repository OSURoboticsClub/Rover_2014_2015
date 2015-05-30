#include<stdexcept>

#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include<opencv2/ml/ml.hpp>

#include<sys/types.h>
#include<sys/socket.h>
#include<stdlib.h>
#include<unistd.h>
#include<netinet/in.h>
#include<netdb.h>
#include<arpa/inet.h>

struct img_packet{
    int cols;
    int rows;
};

#ifndef procimg
#define procimg
int processImage(cv::Mat &img);
#endif
