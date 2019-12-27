#include <opencv2/opencv.hpp>
#include <opencv2/ml/ml.hpp> 
#include <pthread.h>
#include <assert.h>
#include <chrono>
#include <stdio.h>
#include <atomic>
#include "ZJB.h"
#include "PNPSolver.h"
#include "Result.h"
#include "Drawline.h"
#include <iostream>
using namespace std;
using namespace cv;
using namespace cv::ml;
#pragma comment(lib,"pthreadVC2.lib")
//#define Setfbl

String videoPath0 = "avi\\1_1.avi";
String videoPath1 = "avi\\5.avi";

FileStorage fsRead;
Ptr<SVM> SvmLoad;
VideoCapture cap(videoPath0);

static pthread_mutex_t counter_mutex = PTHREAD_MUTEX_INITIALIZER;
//mutex  m, n;
Result r;


void* run1(void* arg)
{
	Mat frame;
	ZJB x;
	Mat src;
	src = Mat::zeros(Size(450, 1), CV_8UC1);
	fsRead["gray_th"] >> x.gray_th;
	fsRead["color_th"] >> x.color_th;
	fsRead["ch"] >> x.ch;
	fsRead["diff_angle"] >> x.diff_angle;
	fsRead["min_area"] >> x.min_area;
	fsRead["max_area"] >> x.max_area;


	namedWindow("data");
	
#ifdef Setfbl
	cap.set(CAP_PROP_FRAME_WIDTH, 1024);//宽度 1024
	cap.set(CAP_PROP_FRAME_HEIGHT, 768);//高度768
#endif	
	while (1)
	{
		pthread_mutex_lock(&counter_mutex);
		cap >> frame;
		pthread_mutex_unlock(&counter_mutex);

		if (frame.data)
		{
			clock_t start, end;
			start = clock();
			x.b_track_armor(frame, r, fsRead, SvmLoad);
			end = clock();
			double fps = 1 / ((double)(end - start) / CLOCKS_PER_SEC);
			putText(frame, "fps: " + to_string(fps), Point(10, 20), 5, 1.2, Scalar(0, 0, 255));
			cout << fps << endl;
		}
		else
		{
			break;
		}
		pthread_mutex_lock(&counter_mutex);
		createTrackbar("gray_th", "data", &x.gray_th, 255, 0);
		createTrackbar("color_th", "data", &x.color_th, 255, 0);
		createTrackbar("ch", "data", &x.ch, 1, 0);
		createTrackbar("diff_angle", "data", &x.diff_angle, 30, 0);
		
		imshow("data", src);
		waitKey(1);
		pthread_mutex_unlock(&counter_mutex);
	}
	return NULL;
}


int main()
{ 
	cout << "加载中..." << endl;
	fsRead.open("xml\\armor.xml", FileStorage::READ, "utf-8");
	SvmLoad = StatModel::load<SVM>("./xml/svm.xml");
	cout << "加载xml完毕。" << endl;
	//long totalFrameNumber = cap.get(CAP_PROP_FRAME_COUNT);//获取视频的总帧数
	//cout << "整个视频共" << totalFrameNumber << "帧" << endl;

	//Mat f;
	//for (int i = 0; i < 1700; i++)
	//{
	//	cap >> f;
	//}
	//
	pthread_t t1, t2;
	pthread_create(&t1, NULL, run1, NULL);
	//pthread_create(&t2, NULL, run1, NULL);
	pthread_join(t1, NULL);
	//pthread_join(t2, NULL);
	//thread t2(run1, cap);
	//t2.join();

	waitKey(0);
	return 0;
}






