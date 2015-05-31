#include "helper.h"

using namespace std;
using namespace cv;



int calculate_features_from_input(Mat &image, vector<float>& featureVector,
        HOGDescriptor& hog) 
{
        if(image.empty()){
            featureVector.clear();
            return 0;
        }
        if (image.cols != hog.winSize.width || image.rows != hog.winSize.height) {
            featureVector.clear();
            return 0;
        }
        hog.compute(image, featureVector); 
        image.release();
        return 1;
}


void calculate_features_from_input(char* imageFilename, vector<float>& featureVector,
        HOGDescriptor& hog) 
{
        Mat image;
        image = imread(imageFilename);
        if(image.empty()){
            featureVector.clear();
            cout << "Image " << imageFilename << " Empty, skipping." << endl;
            return;
        }
        if (image.cols != hog.winSize.width || image.rows != hog.winSize.height) {
            featureVector.clear();
            cout << imageFilename << " Couldn't use for training since the dimensions are wrong" << endl;
            return;
        }
        hog.compute(image, featureVector); 
        image.release();
}

void flv2mat(vector<float> &values, Mat &cp){
    int orient  = (cp.rows > cp.cols);
    for(int i=0; i < cp.rows; ++i){
        for(int j=0; j < cp.cols; ++j){
            if(orient){
                cp.at<float>(i, j) = values.at(i);
            }else{
                cp.at<float>(i, j) = values.at(j);
            }
        }
    }
}

/*
 * Calculates the overlap of two rectangles
 */
int get_overlap(int x1, int y1, int x2, int y2, int width, int height, double scale){
    int x22 = x2+width*scale;
    int y22 = y2+height*scale;
    int xover = max(0, min(x1+width, x22) - max(x1,x2));
    int yover = max(0, min(y1+height,y22) - max(y1, y2));
    return xover*yover;
}


