#include "detector_server.h"


/*
 * Sets up the detector server and initalizes the SVM
 * and Descriptor
 */
DetectorServer::DetectorServer(){
    //load the model used to classify
    svm.load("/home/sp/SeniorProject/models/stopsign_3.yaml"); 
    //convert the svm to a vector for floats
    vector<float> support_vector;
    svm.getSupportVector(support_vector);
    //load this into our detector
    desc.setSVMDetector(support_vector);
}

/*
 * Detects objects that match the model in the image
 * returns > 1 if there was a match found in the image
 */
int DetectorServer::processImage(const sensor_msgs::ImageConstPtr& msg){
    try{
        Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    }catch(cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    if(img.cols == 0) return 0;    
    Mat gray_img;
    img_count++;
    cvtColor(img, gray_img, CV_BGR2GRAY); 
    
    //our large image holder
    gpu::GpuMat gimg;
    
    //upload our image to the gpu
    gimg.upload(gray_img);
    
    /*
    //quick rejector
    vector<Rect> bad_matches;
    desc.detectMultiScale(gimg, bad_matches, 0, Size(), Size(0, 0), 2.0, 5);
    if(bad_matches.size() == 0){
	return 0;
    }
    */
    
    //create the vector to hold the results
    vector<Rect> matches;

    desc.detectMultiScale(gimg, matches, 0, Size(), Size(0, 0), 1.10, 5);
    for(int i=0; i < matches.size(); ++i){
	rectangle(gray_img, matches[i], Scalar(255, 0, 0));		
    }    
    imwrite("/var/www/training_tests/test"+to_string(img_count)+".jpg", gray_img);

    return matches.size()>0;
}

int main(){
    DetectorServer ds();
    ros::init(argc, argv, "object_recognition");
    ros::NodeHandle nh;
    cv::namedWindow("view");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("arm/image", 1, DetectorServer::processImage, ds);
    rospy.spin();
    cv::destroyWindow("view");
}
