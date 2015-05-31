/*
 * boardDetect.cpp
 *
 *  Created on: Apr 10, 2015
 *      Author: scott
 */
#include "HomeBase.hpp"

using namespace cv;
using namespace std;


float dist(Point2f p1, Point2f p2){
	float dist = sqrt((p2.x - p1.x)*(p2.x - p1.x) + (p2.y - p1.y)*(p2.y - p1.y));
	return(dist);
}
vector<Point2f> getCorners(vector<vector<Point2f> >imagePoints, Size boardSize){
	vector<Point2f> corners(4);
	corners[0] = imagePoints[0][0];
	corners[1] = imagePoints[0][boardSize.width-1];
	corners[2] = imagePoints[0][(boardSize.width * (boardSize.height-1))];
	corners[3] = imagePoints[0][(boardSize.width * (boardSize.height)) -1];
	return corners;
}
vector<float> getPixDist(vector<Point2f> corners){
	vector<float> dists(4);
	Point2f tl = corners[0];
	Point2f tr = corners[1];
	Point2f bl = corners[2];
	Point2f br = corners[3];

	float h1 = dist(tl, tr);
	float h2 = dist(bl, br);
	float v1 = dist(tl, bl);
	float v2 = dist(tr, br);

	dists[0] = h1;
	dists[1] = h2;
	dists[2] = v1;
	dists[3] = v2;

	return dists;
}
vector<float> computeFocal(int side, vector<float> pixDist, float D){
	//F = (P x  D) / W
	//float D = TRAIN_DIST; //train distance 22 inches or 558.8mm
	float Wh, Wv;
	if(side == FRONT){
		Wh = FRONT_SZIE * (FRONT_COLS-1); //width of 4 squares (50.8mm) each
		Wv = FRONT_SZIE * (FRONT_ROWS-1); //width of 3 squares (50.8mm) each
	} else {
		Wh = BACK_SZIE * (BACK_COLS-1); //width of 4 squares (50.8mm) each
		Wv = BACK_SZIE * (BACK_ROWS-1); //width of 3 squares (50.8mm) each
	}
	float Fh1 = (pixDist[0] * D) / Wh;
	float Fh2 = (pixDist[1] * D) / Wh;
	float Fv1 = (pixDist[2] * D) / Wv;
	float Fv2 = (pixDist[3] * D) / Wv;
	float Fh  = (Fh1 + Fh2) / 2;
	float Fv  = (Fv1 + Fv2) / 2;

	vector<float> focal(2);
	focal[0] = Fh;
	focal[1] = Fv;
	return focal;
}
float computeDistance(int side, vector<float> pixDist, vector<float> focal){
	//D = (W x F) / P

	float Wh, Wv;
	if(side == FRONT){
		Wh = FRONT_SZIE * (FRONT_COLS-1);
		Wv = FRONT_SZIE * (FRONT_ROWS-1);
	} else {
		Wh = BACK_SZIE * (BACK_COLS-1);
		Wv = BACK_SZIE * (BACK_ROWS-1);
	}
	float Dh1 = (Wh * focal[0]) / pixDist[0];
	float Dh2 = (Wh * focal[0]) / pixDist[1];
	float Dv1 = (Wv * focal[1]) / pixDist[2];
	float Dv2 = (Wv * focal[1]) / pixDist[3];
	float D = (Dh1 + Dh2 + Dv1 + Dv2) / 4;
	return D;
}
vector<float> trainDistance(int side, Size boardSize){
	Mat image5, image10;
	if(side == FRONT){
		image5 = imread(FRONT_TRAIN5); //read in trained image file
		image10 = imread(FRONT_TRAIN10); //read in trained image file
		if (image5.empty() || image10.empty()) {
			cerr << "could not find training image" << endl;
			exit(0);
		}
	} else {
		image5 = imread(BACK_TRAIN5); //read in trained image file
		image10 = imread(BACK_TRAIN10); //read in trained image file
		if (image5.empty() || image10.empty()) {
			cerr << "could not find training image" << endl;
			exit(0);
		}
	}

	vector<vector<Point2f> > imagePoints5(1), imagePoints10(1);
	bool found5 = findChessboardCorners(image5, boardSize, imagePoints5[0]);
	bool found10 = findChessboardCorners(image10, boardSize, imagePoints10[0]);

	if(!(found5 && found10))
	{
		cerr << "Could not find chess board in TRAINING!" << endl;
		return vector<float>();
	}
	vector<Point2f> corners5 = getCorners(imagePoints5, boardSize);
	vector<Point2f> corners10 = getCorners(imagePoints10, boardSize);
	vector<float> pix_dist5 = getPixDist(corners5);
	vector<float> pix_dist10 = getPixDist(corners10);
	vector<float> focal5 = computeFocal(side, pix_dist5, TRAIN_DIST5);
	vector<float> focal10 = computeFocal(side, pix_dist10, TRAIN_DIST10);
	vector<float> focal(2);
	focal[0] = (focal5[0] + focal10[0])/2;
	focal[1] = (focal5[1] + focal10[1])/2;
	return focal;
}
/*
 *  Given two vertices, finds the vertical angle of the
 *  top vertex with respect to the bottom (bottom = center/pivot).
 *  @return value of range [-90, 90],
 *          positive angle is clockwise tilt,
 *          negative angle is counter-clockwise tilt
 *      - * -
 *    *   |   *
 *    -   |   +
 *        |
 *        *
 */
float
findTilt(Point2f p1, Point2f p2)
{
	Point2f top, bot;
	if (p1.y > p2.y) {
		top = p2;
		bot = p1;
	} else if (p1.y < p2.y) {
		top = p1;
		bot = p2;
	} else {
		if (p1.x > p2.x) {
			return -90;
		} else if (p1.x < p2.x) {
			return 90;
		} else {
			return 0;
		}
	}

	int delta_x, delta_y;
	float theta;
	delta_x = top.x - bot.x;
	delta_y = bot.y - top.y; //y pixel increases downwards
	theta = atan(delta_x / delta_y);
	return theta;
}

/*
 *  This function calculate angle of orientation (yaw) using trigonometric methods
 *  Assumptions: the four corner vertices create a parallelogram, thus we are
 *               using only one of the sides for calculating tilt
 */
float
findOrientation(vector<Point2f> corners, int rows, int cols)
{
	bool ERROR = false;
	int tilt;
//	if (corners.size() != (rows*cols)) {
//		cerr << "Incompatible rows and cols for passed in corners" << endl;
//	}
	Point2f top_left = corners[0];
	Point2f top_right = corners[1];
    Point2f bot_left = corners[2];
    Point2f bot_right = corners[3];

	float theta = findTilt(bot_right, top_right);
	if (theta > TILT_THRESH) {
		ERROR = true;
		tilt = theta; //set global
		return 0;
	} else {
		ERROR = false;
	}

	//fix needed: rearrange if board is upside down

	float delta_y;
	delta_y = bot_right.y - top_right.y;
	float r_len = delta_y / cos(theta);
	delta_y = bot_left.y - top_left.y;
	float l_len = delta_y / cos(theta);

	float ratio = l_len / r_len;  //ratio (left : right)
	float orientation = (1 - ratio) * 264; //264 based on my testing
	//cout << "left: " << l_len << ", right: " << r_len << ", ratio: " << ratio << ", angle: " << orientation << endl;
	return orientation;
}
vector<float> detectBoard(Mat &image, int side, Size boardSize, vector<float> focal){
	vector<float> board(2);

	if( image.empty() )
		break;
	// Find the chessboard corners
	vector<vector<Point2f> > imagePoints(1);
	bool found = findChessboardCorners(image, boardSize, imagePoints[0]);
	if(!found)
	{
		cerr << "Could not find chess board!" << endl;
		board[0] = CANT_FIND;
		return board;
	}else {

		vector<Point2f> corners = getCorners(imagePoints, boardSize);
		vector<float> pix_dist = getPixDist(corners);
		float D = computeDistance(side, pix_dist, focal);
		float angle;
		if(side = FRONT){
			angle = findOrientation(corners, FRONT_ROWS, FRONT_COLS);
		} else {
			angle = findOrientation(corners, BACK_ROWS, BACK_COLS);
		}
		board[0] = D;
		board[1] = angle; //this should be the angle of the board
		drawChessboardCorners(image, boardSize, cv::Mat(imagePoints[0]), found );

		ostringstream msg;
		msg << "Distance "<< D << "mm" << " Angle " << angle;
		int baseLine = 0;
		Size textSize = getTextSize(msg.str(), 1, 1, 1, &baseLine);
		Point textOrigin(image.cols - 2*textSize.width - 10, image.rows - 2*baseLine - 10);
		putText( image, msg.str(), textOrigin, 1, 1, Scalar(0,0,255));
	}

	return board;
}

int main( int argc, const char** argv )
{
	Size frontSize(FRONT_COLS,FRONT_ROWS); //(cols-1, rows-1) This way it finds points going across the rows
	Size backSize(BACK_COLS,BACK_ROWS); //(cols-1, rows-1) This way it finds points going across the rows
	vector<float> focalf = trainDistance(FRONT, frontSize);
	vector<float> focalb = trainDistance(BACK, backSize);


	VideoCapture cap; //capturing video off of camera
	cap.open(0); //open camera to default camera

	if( !cap.isOpened() ) //make sure camera is open
	{
		cout << "***Could not initialize capturing...***\n";
		cout << "Current parameter's value: \n";
		return -1;
	}

	//check frame resolution and try and set it (currently doesn't work on Jetson)
	cout << ": width=" << cap.get(CV_CAP_PROP_FRAME_WIDTH) << ", height=" << cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;
	Mat image;
	float distance, angle;
	int side = 0;
	while(true){
		cap >> image;
		vector<float> board = detectBoard(image, FRONT, frontSize, focalf);
		if(board[0] == CANT_FIND){
			board = detectBoard(image, BACK, backSize, focalb);
			side = BACK;
		} else {
			side = FRONT;
		}

		distance = board[0];
		angle = board[1];
		if (! image.empty()) {
			imshow("Image View", image);
		}
		char c = (char)waitKey(10);
		if( c == 27 )
			break;
		switch(c)
		{
		case 'n':
			break;
		default:
			;
		}
	}
}
