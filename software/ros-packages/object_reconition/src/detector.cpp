#include "detector.h"

int processImage(Mat &img){
    if(img.cols == 0) return 0;
    //the cv class used to extract the hog descriptors
    gpu::HOGDescriptor desc;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    //the svm we are going to use
    LinearSVM svm;
    //load the model used to classify
    svm.load("/home/sp/SeniorProject/models/stopsign.yaml"); 

    vector<float> support_vector;
    svm.getSupportVector(support_vector);
    desc.setSVMDetector(support_vector);
    
    //upload the image to the gpu for detection
    Mat gray_img;
    cvtColor(img, gray_img, CV_BGR2GRAY); 
    
    //imwrite("/var/www/training_tests/test_img.jpg", gray_img);
    //our large image holder
    gpu::GpuMat gimg;
    //create the vector to hold the results
    vector<Rect> matches;
    
    //upload our image to the gpu
    gimg.upload(gray_img);
    desc.detectMultiScale(gimg, matches, 0, Size(), Size(0, 0), 1.04, 5);
    return matches.size()>0;
}
