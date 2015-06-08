#include "navimage.h"

bool ERROR = false;
int tilt;

NavImage::NavImage()
{
	// Mat frame;
	
	// frame.copyTo(original);
	// gpuFrame.upload(frame);
	// gpuFrame.copyTo(gray);
}

/*
 * Binary function
 * Returns: binary Mat image
 */
Mat NavImage::getBinary(int thresh, int maxThresh)
{
	this->gpuFrame.upload(this->frame);
	GpuMat binary;

	gpu::cvtColor(this->gpuFrame, this->gray, CV_BGR2GRAY);
	gpu::threshold(this->gray, binary, thresh, maxThresh, THRESH_BINARY);

	Mat tmp;
	binary.download(tmp);
	return tmp;
}

void
NavImage::applyGaussian(int size)
{
	this->gpuFrame.upload(this->frame);
	if (this->gpuFrame.empty()) {
		cout << "empty gpuFrame" << endl;
	}
	gpu::GaussianBlur(this->gpuFrame, this->gpuFrame, Size(size, size), 2);
	gpuFrame.download(this->frame);
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
	if (corners.size() != (rows*cols)) {
		cerr << "Incompatible rows and cols for passed in corners" << endl;
	}
	Point2f top_left = corners.at(0);
	Point2f top_right = corners.at(rows - 1);
    Point2f bot_left = corners.at(rows * (cols - 1));
    Point2f bot_right = corners.at((rows * cols) - 1);

	float theta = findTilt(bot_right, top_right);
	if (theta > abs(TILT_THRESH)) {
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
	cout << "left: " << l_len << ", right: " << r_len << ", ratio: " << ratio << ", angle: " << orientation << endl;
	return orientation;
}


/*
 *  Note: pixels x+ to the right and y+ downwards
 */
void
NavImage::findChessboard(int rows, int cols)
{
	vector<Point2f> corners;
	bool found;
	int flags = CV_CALIB_CB_ADAPTIVE_THRESH;
	flags += CV_CALIB_CB_NORMALIZE_IMAGE;
	//flags += CALIB_CB_FAST_CHECK;
	found = findChessboardCorners(this->frame, Size(rows, cols), corners, flags);
	float orientation = 0;
	if (found) {
		drawChessboardCorners(this->frame, Size(rows, cols), corners, found);

		orientation = findOrientation(corners, rows, cols);
		if (ERROR) {
			cerr << "Error: " << tilt << " degree tilt needs to be < " << TILT_THRESH << endl;
			return;
		}
	}
}
