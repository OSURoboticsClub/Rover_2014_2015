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
vector<float> computeFocal(vector<float> pixDist){
	//F = (P x  D) / W
	float D = TRAIN_DIST; //train distance 22 inches or 558.8mm
	float Wh = SQUARE_SZIE * (GRID_COLS-1); //width of 4 squares (50.8mm) each
	float Wv = SQUARE_SZIE * (GRID_ROWS-1); //width of 3 squares (50.8mm) each
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
float computeDistance(vector<float> pixDist, vector<float> focal){
	//D = (W x F) / P
	float Wh = SQUARE_SZIE * (GRID_COLS-1); //width of 4 squares (50.8mm) each
	float Wv = SQUARE_SZIE * (GRID_ROWS-1); //width of 3 squares (50.8mm) each
	float Dh1 = (Wh * focal[0]) / pixDist[0];
	float Dh2 = (Wh * focal[0]) / pixDist[1];
	float Dv1 = (Wv * focal[1]) / pixDist[2];
	float Dv2 = (Wv * focal[1]) / pixDist[3];
	float D = (Dh1 + Dh2 + Dv1 + Dv2) / 4;
	return D;
}
vector<float> trainDistance(Size boardSize){
	Mat image;

	//image = imread("/home/scott/Rover/Rover2015/software/HomeBase/ChessboardDetection/bw_cal.jpg"); //read in trained image file
	image = imread("/home/ubuntu/Rover/ChessboardDetection/bw_cal.jpg"); //read in trained image file
	vector<vector<Point2f> > imagePoints(1);
	bool found = findChessboardCorners(image, boardSize, imagePoints[0]);

	if(!found)
	{
		cerr << "Could not find chess board!" << endl;
		return vector<float>();
	}
	vector<Point2f> corners = getCorners(imagePoints, boardSize);
	vector<float> pix_dist = getPixDist(corners);
	vector<float> focal = computeFocal(pix_dist);
	return focal;
}
vector<float> detectBoard(VideoCapture &cap, Mat &image, Size boardSize, vector<float> focal){
	vector<float> board(2);
	while(true){
		cap >> image; //pull frame off of the camera
		if( image.empty() )
			break;
		// Find the chessboard corners
		vector<vector<Point2f> > imagePoints(1);
		bool found = findChessboardCorners(image, boardSize, imagePoints[0]);
		if(!found)
		{
			cerr << "Could not find chess board!" << endl;
		}else {

			vector<Point2f> corners = getCorners(imagePoints, boardSize);
			vector<float> pix_dist = getPixDist(corners);
			float D = computeDistance(pix_dist, focal);
			board[0] = D;
			board[1] = 0; //this should be the angle of the board
			drawChessboardCorners(image, boardSize, cv::Mat(imagePoints[0]), found );

			ostringstream msg;
			msg << D << "mm";
			int baseLine = 0;
			Size textSize = getTextSize(msg.str(), 1, 1, 1, &baseLine);
			Point textOrigin(image.cols - 2*textSize.width - 10, image.rows - 2*baseLine - 10);
			putText( image, msg.str(), textOrigin, 1, 1, Scalar(0,0,255));
		}

		waitKey(10);
		if(found)
			break;
	}
	return board;
}
int main( int argc, const char** argv )
{
	Size boardSize(GRID_COLS,GRID_ROWS); //(cols-1, rows-1) This way it finds points going across the rows

	vector<float> focal = trainDistance(boardSize);

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
	while(true){
		vector<float> board = detectBoard(cap, image, boardSize, focal);
		distance = board[0];
		angle = board[1];
		imshow("Image View", image);
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
