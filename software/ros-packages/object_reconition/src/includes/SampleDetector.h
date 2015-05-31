#include <string>
#include <vector>
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/gpu/gpu.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/ml/ml.hpp>
#include "helper.h"

using namespace std;
using namespace cv;

struct SdMatch{
    int x;
    int y;
    vector<float> desc;
};

struct SdPos{
    int x;
    int y;
};
// used to store all the info needed for a thread to run over a row
struct SdRow{
    HOGDescriptor *hog;
    int row; 
    int step_size;
};


class SampleDetector{
    private:
        CvSVM *svm;
        vector<struct SdMatch> found;
        int tag_width;
        int tag_height;
        int thread_run; //when set to zero will kill all threads
        int threads_working; //a count of the amount of threads working
        int running_threads;
        Mat thread_img; 
        struct SdRow row_data; 
        int init_row;
        pthread_mutex_t mutex_row;
        pthread_mutex_t write_match;
        pthread_mutex_t working_dec;
    public: 
        SampleDetector(CvSVM *psvm, int width, int height);
        static void *start_thread(void *arg);
        void *thread_loop();
        void add_pos(int row, int col, vector<float> &desc);
        //use for non-threaded mode
        void scan_row(Mat &img, HOGDescriptor& hog, int row, int step_size);
        //use for threaded must have called set_img
        void scan_row_threaded(HOGDescriptor& hog, int row, int step_size);
        void set_img(Mat &img);
        void clear();
        int cur_threads(){ return running_threads; }
        void kill_thread();
        int tworking(){ return threads_working; }
        ~SampleDetector();
        vector<struct SdMatch> get_matches();
};



