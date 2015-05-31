#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/gpu/gpu.hpp>
#include "navimage.h"

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

using namespace std;
using namespace cv;
using namespace cv::gpu;

string windowName;
string trackbarWindow;

//#define CALC_FPS
#define SHOW_VIDEO
#define RES_720

#ifdef RES_480
const int cam_height = 480;
const int cam_width = 640;
#endif

#ifdef RES_1080
const int cam_height = 1080;
const int cam_width = 1920;
#endif

#ifdef RES_720
const int cam_height = 720;
const int cam_width = 1280;
#endif

static const int CHESS_INNER_ROW = 5;
static const int CHESS_INNER_COLUMN = 4;

void cameraSetup(VideoCapture &capture)
{
	capture.set(CV_CAP_PROP_FRAME_WIDTH, cam_width);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, cam_height);
	cout << "Width: " << capture.get(CV_CAP_PROP_FRAME_WIDTH) << endl;
	cout << "Height: " << capture.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;
}

void applyAll(NavImage *image)
{
	//image.getBinary();
	//image->applyGaussian();
	image->findChessboard(CHESS_INNER_ROW, CHESS_INNER_COLUMN);
}

void showStats()
{
	cout << endl << "Stats:" << endl;
}

int main(int argc, char **argv)
{
	if (argc > 1) {
		cout << "reading file not implemented" << endl;
		exit(0);
	}
	
	VideoCapture cap(0);
	if (!cap.isOpened()) {
		cerr << "Error connecting to a camera device" << endl;
		exit(0);
	}

#ifdef SHOW_VIDEO
	windowName = "camera feed";
	namedWindow(windowName, WINDOW_NORMAL);
	resizeWindow(windowName, cam_width, cam_height);
	trackbarWindow = "trackbar";
	namedWindow(trackbarWindow, WINDOW_NORMAL);
#endif
	cameraSetup(cap);

	cout << "In capture ..." << endl;
#ifdef CALC_FPS
	string str = "Captures per second: ";
	float cap_per_sec = 0;
	time_t time1 = time(NULL);
	time_t time2 = time(NULL);
	cout << str << "00.00";
	cout.flush();
#endif
	NavImage image;

	int keyPress;
	while (true) {
		cap.read(image.frame);
		applyAll(&image);
#ifdef SHOW_VIDEO
		if(!image.frame.empty()){
			imshow(windowName, image.frame);
		}
#endif
#ifdef CALC_FPS
		cap_per_sec += 1;
		time2 = time(NULL);
		if (difftime(time2, time1) > 1) {
			time1 = time(NULL);
			cout << string(str.length() + 5, '\b');
			cout << str;
			cout.flush();
			fprintf(stderr, "%2.2f", cap_per_sec);
			cap_per_sec = 0;
		}
#endif
		keyPress = waitKey(1);
		if (keyPress != -1) {
			showStats();
			exit(EXIT_SUCCESS);
		}
	}
}
