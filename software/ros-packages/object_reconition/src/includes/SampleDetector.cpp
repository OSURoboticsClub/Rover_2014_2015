#include "SampleDetector.h"

using namespace std;
using namespace cv;

SampleDetector::SampleDetector(CvSVM *psvm, int width, int height){
    svm = psvm;
    thread_run = 1;
    init_row = 0;
    threads_working = 0;
    running_threads = 0;
    pthread_mutex_init(&mutex_row, NULL);
    pthread_mutex_init(&write_match, NULL);
    pthread_mutex_init(&working_dec, NULL);
    pthread_mutex_lock(&mutex_row);
    tag_width = width;
    tag_height = height;
}
SampleDetector::~SampleDetector(){
    pthread_mutex_destroy(&mutex_row);
    pthread_mutex_destroy(&write_match);
}

void *SampleDetector::start_thread(void *context){ 
    ((SampleDetector *) context)->thread_loop();
     pthread_exit(NULL);
}

void *SampleDetector::thread_loop(){
    running_threads++;
    while(thread_run){
        pthread_mutex_lock(&mutex_row);
        //this is needed in case something has changes since we tried to get a key
        if(!thread_run){
            break;
        }
        struct SdRow data = row_data;
        init_row = 0;
        pthread_mutex_lock(&working_dec);
        threads_working++;
        pthread_mutex_unlock(&working_dec);
        scan_row(thread_img, *data.hog, data.row, data.step_size); 
        pthread_mutex_lock(&working_dec);
        threads_working--;
        pthread_mutex_unlock(&working_dec);
    }
    running_threads--;
    return NULL;
}

void SampleDetector::kill_thread(){
    thread_run = 0;
    pthread_mutex_unlock(&mutex_row);
}

/*
 * Adds the pos (x and y) of a matching vector
 */
void SampleDetector::add_pos(int row, int col, vector<float> &desc){
    //make sure that only one thread is adding a pos at a time
    pthread_mutex_lock(&write_match);
    struct SdMatch pos = {col, row, desc};
    found.push_back(pos);
    pthread_mutex_unlock(&write_match);
}

void SampleDetector::scan_row(Mat &img, HOGDescriptor& hog, int row, int step_size){
    vector<float> vfeatures;
    for(int c=0, max_col=img.cols-tag_width; c < max_col; c+=step_size){ 
        //extract the current block the size of out descriptor
        Mat block(img, Rect(c, row, tag_width, tag_height)); 
        vfeatures.clear(); 
        int s = calculate_features_from_input(block, vfeatures, hog); 
        if(s != 0){
            Mat features(1, vfeatures.size(), CV_32FC1); 
            flv2mat(vfeatures, features);
            float clss = svm->predict(features);
            if(clss == 1.0){
                add_pos(row, c, vfeatures);           
            }
        }
    }
}

void SampleDetector::scan_row_threaded(HOGDescriptor& hog, int row, int step_size){
   row_data = {&hog, row, step_size};
   init_row = 1;
   pthread_mutex_unlock(&mutex_row);
   while(init_row){ }
}

void SampleDetector::set_img(Mat &img){
    thread_img.release();
    thread_img = Mat(img);
}

void SampleDetector::clear(){
    found.clear();
}

vector<struct SdMatch> SampleDetector::get_matches(){
    return found;
}



