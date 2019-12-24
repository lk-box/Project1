#pragma once
#include <iostream>
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/videoio.hpp"
#include <cmath>
#include <omp.h>
#include "Result.h"
using namespace cv;
using namespace std;


class ZJB
{
public:
	ZJB(const char *_Filename);
	ZJB(int i);
	ZJB();
	void brightAdjust(Mat src, Mat &dst, double dContrast, double dBright);
	void getDiffImage(Mat src1, Mat src2, Mat dst, int nThre);
	vector<RotatedRect> armorDetect(vector<RotatedRect> vEllipse, Mat frame);
	void drawBox(RotatedRect box, Mat img);
	void drawBoxd(RotatedRect box, Mat img);

	Mat get_image();
//	void track_armor();
	void b_track_armor(Mat srcimage, Result &r);

private:
	VideoCapture cap;
	Mat dstimg;
	int tsize1;
	int tangle, bright;
	int k, ch;
	int thresh, thresh1;
};

