#include <iostream>
#include <opencv2/opencv.hpp>
#include <windows.h> 
#include <opencv2/ml/ml.hpp> 
#include <thread>  
#include <chrono>
#include <atomic>
#include "ZJB.h"
#include "DFC.h"
#include "PNPSolver.h"
#include "Result.h"
#include "Drawline.h"
using namespace std;
using namespace cv;
using namespace cv::ml;

#define Z  //装甲板
//#define D  //大风车
//#define PT //普通摄像头
//#define CJ //测距
//#define XC  //多线程
//#define XC1  //多线程
//#define BLUR
//#define DXC
//#define HUATU

#ifdef Z

//#define cs
mutex  m, n;

vector<Mat> Frame;

bool flag = 0;
Result r;
//int k = 0;
int thresh;//100
int thresh1;//10
int ch;
vector<int> a, b;
int count1 = 0;
int count2 = 0;
double foreangle = 0;
void run1()
{
	VideoCapture cap("avi\\9.avi");//"avi\\3.avi" 1063
	Mat frame;
	long totalFrameNumber = cap.get(CAP_PROP_FRAME_COUNT);//获取视频的总帧数
	cout << "整个视频共" << totalFrameNumber << "帧" << endl;   
	cap.set(CAP_PROP_FRAME_WIDTH, 1024);//宽度 1024
	cap.set(CAP_PROP_FRAME_HEIGHT, 768);//高度768
	
	//for (int i = 0; i < 1700; i++)//1700
	//{
	//	cap.read(frame);
	//}

	while (1)
	{
		if (Frame.empty())
		{
			//if (count1 > 500)
			//{
			//	Frame.clear();
			//	flag = 1;
			//	break;
			//}
			if (cap.read(frame))
			{
				m.lock();
				Frame.push_back(frame);
				m.unlock();
			}
			else
			{
				Frame.clear();
				flag = 1;
				break;
			}
		}
	}
}
void run2()
{
	while (1)
	{
		if (flag == 1)
		{
			break;
		}
		if (!Frame.empty())
		{
			ZJB x;
			Mat tmp;
			m.lock();
			if (!Frame.empty())
			{
				tmp = Frame.back();
				Frame.pop_back();
			}
			m.unlock();

			clock_t t1 = clock();
			x.b_track_armor(tmp, r);
			clock_t t2 = clock();
			//cout << t2 - t1 << endl;


			m.lock();
			//if (r.d_pitch == 1000.0 && foreangle == 1000.0)
			//{
			//	//cout << "停止";
			//	cout << endl;
			//	a.clear();
			//}

				/*cout << "pnp: " << r.d_distance << endl;
				cout << "相似:" << r.r_x << endl;
				cout << "pitch: " << r.d_pitch << endl;
				cout << "yaw: " << r.d_yaw << endl;
				cout << endl;*/
			if (a.size() >= 3)
			{
				a.erase(a.begin() , a.begin() + 1);

				if (abs(r.d_pitch * 10 - a.back()) < 100)
				{
					a.push_back(r.d_pitch * 10 * 0.5 + a[1] * 0.3 + a[0] * 0.2);
					//cout << r.d_pitch * 10 << " ";
				}
			}
			else
			{
				if (a.size() == 0)
				{
					//cout << "发送：" << endl;
					a.push_back((int)(r.d_pitch * 10));
				}
				else
				{
					a.push_back((int)(r.d_pitch * 10));
					//cout << r.d_pitch * 10 << " ";
				}
			}
			foreangle = r.d_pitch;
			m.unlock();

			if (flag == 1)
			{
				break;
			}
		}
	}
}


int main()
{ 
	thread t1(run1);
	thread t2(run2);
	//thread t3(run2);

	t1.join();
	t2.join();
	//t3.join();
	//cout << 1;
	//for (int i = 0; i < a.size(); i++)
	//{
	//	cout << a[i] << endl;
	//}
	//cout << endl;
	//cout << 2;
	//vector<int> c;
	//c.push_back(a[0]);
	//c.push_back(a[1]);

	//for (int i = 2; i < a.size(); i++)
	//{
	//	c.push_back (a[i] * 0.5 + a[i - 1] * 0.3 + a[i - 2] * 0.2);
	//}
	//cout << 3;
	//for (int i = 0; i < c.size(); i++)
	//{
	//	cout << c[i] << endl;
	//}



	//Drawline e(c);
	//e.draw(c.size(), 500);

	waitKey(0);


	return 0;
}
#endif

#ifdef D
int main()
{
	DFC d("avi\\red0.avi");
	//DFC d("blue0.avi");
	//DFC d("taili.mp4");

	d.track_pinwheel_1();
	//d.track_pinwheel();

	
	return 0;
}
#endif

#ifdef PT

int main()
{
	VideoCapture capture(0);
	Mat frame;
	capture.set(CAP_PROP_FRAME_WIDTH, 1024);//宽度 
	capture.set(CAP_PROP_FRAME_HEIGHT,768);//高度
	//capture.set(CAP_PROP_AUTO_EXPOSURE, 0.25); 
	capture.set(CAP_PROP_EXPOSURE, -7);
	int k = 50;
	while (1)
	{
		//cout << capture.get(CAP_PROP_FRAME_WIDTH)<< endl;//640
		//cout << capture.get(CAP_PROP_FRAME_HEIGHT) << endl;//480
		capture.read(frame);
		imshow("frame", frame);
		if (waitKey(1) == 32)
		{
			k++;
			string Img_Name = "D:\\RM\\matlab\\image\\im" + to_string(k) + ".jpg";
			imwrite(Img_Name, frame);
		}
		//当等待时间内无任何操作时等待结束后返回-1。 
			 //当等待时间内有输入字符时，则返回输入字符的ASCII码对应的十进制值。
	}
	return 0;
}


#endif

#ifdef CJ
int main()
{
	
	//
	//PNPSolver p;
	//p.read_xml("rm.xml");
	//vector <Point2f> points;
	////
	////points.push_back(cv::Point2f(970, 414));   //P5
	////points.push_back(cv::Point2f(1306, 402));  //P1
	////points.push_back(cv::Point2f(1320, 612));  //P2
	////points.push_back(cv::Point2f(980, 625));  //P3

	//points.push_back(cv::Point2f(950, 288));   //P5
	//points.push_back(cv::Point2f(1328, 288));  //P1
	//points.push_back(cv::Point2f(1341, 441));  //P2
	//points.push_back(cv::Point2f(942, 437));  //P3
	//p.get_distance(points);
	//system("pause");
	Mat im = imread("im52.jpg");// 290 362    771  362           481 F = (P*D)/W
	imshow("image", im);                                         //W =  210    D = 240    P = 481            

	return 0;
}

#endif

#ifdef XC

int index = 0;
int tickets = 1000;
HANDLE hMutex = NULL;
HANDLE hEvent = NULL;
CRITICAL_SECTION g_cs;
clock_t t1;
Mat src;
Mat rimg, bimg, gimg;
int b = 0, g = 0, r = 0;
//DWORD WINAPI Fun(LPVOID lpParamter)
//{
//	for (int i = 0; i < 10; i++)
//	{
//		WaitForSingleObject(hMutex, INFINITE);
//
//		cout << "This is a Thread!" << endl;
//		cout << "A Thread Fun Display!\n";
//		Sleep(1000);
//
//		ReleaseMutex(hMutex);
//	}
//	return 0L;
//}

DWORD WINAPI Func1(LPVOID pPaream)
{
		//WaitForSingleObject(hMutex, INFINITE);
		//WaitForSingleObject(hEvent, INFINITE);
		EnterCriticalSection(&g_cs);
		double time0 = static_cast<double>(getTickCount());

		for (int y = 0; y < bimg.rows; y++)
		{
			uchar *data = bimg.ptr<uchar>(y);
			uchar *data1 = src.ptr<uchar>(y);
			for (int x = 0; x < bimg.cols; x++)
			{
				//WaitForSingleObject(hMutex, INFINITE);
				data[x] = data1[x * 3];
				//ReleaseMutex(hMutex);
			}
		}

		time0 = ((double)getTickCount() - time0) / getTickFrequency();
		cout << "bimg：" << time0 << "秒" << endl;

	

		//waitKey(10);
		//ReleaseMutex(hMutex);
		//SetEvent(hEvent);
		LeaveCriticalSection(&g_cs);
	
	return 0;
}

DWORD WINAPI Func2(LPVOID pPaream)
{
	
		//WaitForSingleObject(hMutex, INFINITE);
		//WaitForSingleObject(hEvent, INFINITE);
		EnterCriticalSection(&g_cs);
		double time0 = static_cast<double>(getTickCount());

		for (int y = 0; y < bimg.rows; y++)
		{
			uchar *data = gimg.ptr<uchar>(y);
			uchar *data1 = src.ptr<uchar>(y);
			for (int x = 0; x < bimg.cols; x++)
			{
				//WaitForSingleObject(hMutex, INFINITE);
				data[x] = data1[x * 3 + 1];
				//ReleaseMutex(hMutex);
			}
		}
		time0 = ((double)getTickCount() - time0) / getTickFrequency();
		cout << "gimg：" << time0 << "秒" << endl;


		//ReleaseMutex(hMutex);
		//SetEvent(hEvent);
		LeaveCriticalSection(&g_cs);
	
	return 0;
}
DWORD WINAPI Func3(LPVOID pPaream)
{

		//WaitForSingleObject(hMutex, INFINITE);
		//WaitForSingleObject(hEvent, INFINITE);
		EnterCriticalSection(&g_cs);
		double time0 = static_cast<double>(getTickCount());

		for (int y = 0; y < bimg.rows; y++)
		{
			uchar *data = rimg.ptr<uchar>(y);
			uchar *data1 = src.ptr<uchar>(y);
			for (int x = 0; x < bimg.cols; x++)
			{
				//WaitForSingleObject(hMutex, INFINITE);
				data[x] = data1[x * 3 + 2];
				//ReleaseMutex(hMutex);
			}
		}
		time0 = ((double)getTickCount() - time0) / getTickFrequency();
		cout << "rimg：" << time0 << "秒" << endl;

		//cout << "rimg ok!";

		//ReleaseMutex(hMutex);
		//SetEvent(hEvent);
		LeaveCriticalSection(&g_cs);
	
	return 0;
}

int main()
{
	src = imread("cj0.jpg");
	Size imgSize;
	imgSize = src.size();
	rimg = Mat(imgSize, CV_8UC1);
	gimg = Mat(imgSize, CV_8UC1);
	bimg = Mat(imgSize, CV_8UC1);

	double time0 = static_cast<double>(getTickCount());

	for (int y = 0; y < bimg.rows; y++)
	{
		uchar *datar = rimg.ptr<uchar>(y);
	/*	uchar *datag = gimg.ptr<uchar>(y);
		uchar *datab = bimg.ptr<uchar>(y);*/
		uchar *data1 = src.ptr<uchar>(y);
		for (int x = 0; x < bimg.cols; x++)
		{
			//datab[x] = data1[x * 3];
			//datag[x] = data1[x * 3 + 1];
			datar[x] = data1[x * 3 + 2];
		}
	}
	time0 = ((double)getTickCount() - time0) / getTickFrequency();
	cout << "此方法运行时间为：" << time0 << "秒" << endl;


	//HANDLE hThread1 = CreateThread(NULL, 0, Func1, NULL, 0, NULL);
	//CloseHandle(hThread1);
	//HANDLE hThread2 = CreateThread(NULL, 0, Func2, NULL, 0, NULL);
	//CloseHandle(hThread2);
	//HANDLE hThread3 = CreateThread(NULL, 0, Func3, NULL, 0, NULL);
	//CloseHandle(hThread3);

	//hMutex = CreateMutex(NULL, FALSE, NULL);

	//while (1)
	//{
	//	if (r == 1 && b == 1 && g == 1)
	//	{
	//		imshow("rimg", rimg);
	//		imshow("gimg", gimg);
	//		imshow("bimg", bimg);
	//	}
	//}
	//imshow("rimg", rimg);
	//imshow("gimg", rimg);
	//HANDLE hThread1 = CreateThread(NULL, 0, Func1, NULL, 0, NULL);
	//CloseHandle(hThread1);
	


	//if (hMutex)
	//{
	//	if (ERROR_ALREADY_EXISTS == GetLastError())
	//	{
	//		cout << "Only one instance can run" << endl;
	//		return 0;
	//	}
	//}

	//hEvent = CreateEvent(NULL, FALSE, TRUE, NULL);
	
	//InitializeCriticalSection(&g_cs);
	//Sleep(1000);
	Sleep(100000);
	//DeleteCriticalSection(&g_cs);
	return 0;
}


#endif

#ifdef XC1
int cnt = 20;
mutex m;

void t1()
{
	while (cnt > 0)
	{

		//m.lock();
		lock_guard<mutex> lockGuard(m);
		if (cnt > 0)
		{
			--cnt;
			cout << cnt << endl;
		}

		//m.unlock();
	}
}
void t2()
{
	while (cnt > 0)
	{
		//m.lock();
		lock_guard<mutex> lockGuard(m);
		if (cnt > 0)
		{
			--cnt;
			cout << cnt << endl;
		}

		//m.unlock();
	}
}


const int N = 10000000;
int num1 = 0;
int num2 = 0;
//atomic_int num{ 0 };
void run1()
{
	for (int i = 0; i < N; i++)
	{
		//m.lock();
		num1++;
		//m.unlock();
	}
}
void run2()
{
	for (int i = 0; i < N; i++)
	{
		//m.lock();
		num1--;
		//m.unlock();
	}
}


int main()
{
 //   thread th1(t1);  //实例化一个线程对象th1，使用函数t1构造，然后该线程就开始执行了（t1()）
 //   thread th2(t2);
	//th1.join(); //等待th1执行完
	//th2.join(); //等待th2执行完

 //   cout << "here is main\n\n";

	//auto n = thread::hardware_concurrency();//获取cpu核心个数  
	//cout << n << endl;

	clock_t start = clock();
	thread t1(run1);
	t1.join();
	thread t2(run2);
	t2.join();
	clock_t end = clock();
	cout << "num1=" << num1 << " num2=" << num2 << " " << ",用时 " << end - start << " ms" << endl;

	system("pause");
    return 0; 
}
	
#endif

#ifdef BLUR
#include <stdio.h>
#include <assert.h>
#include <string>
#include <opencv2/opencv.hpp>
using namespace std;
#define THREAD_NUMS 3

/*paramThread用于传递线程需要的参数值*/
struct paramThread
{
	int w;
	int h;
	uchar * data;
};

/********************************************************
*	@brief       : 多线程处理函数
*	@param  args : 多线程传入的参数
*	@return      : void
********************************************************/
void threadProcess(void* args) 
{
	paramThread *para = (paramThread *)args;
	int w = para->w;
	int h = para->h;
	Mat image(h, w, CV_8UC3, (uchar *)para->data);
	blur(image, image, Size(7, 7), Point(-1, -1), BORDER_REPLICATE);
	//GaussianBlur(image, image, Size(9, 9), 0, 0);
	//dilate(image, image, Mat(), Point(-1, -1), 5);
}

/********************************************************
*	@brief       : 实现图像分割，
*	@param  num  :  分割个数
*	@param  type : 0：垂直分割(推荐)，1：水平分割（不推荐）
*	@return      : vector<cv::Mat>
*   PS：使用水平分割时（type=1），处理完后必须调用catImage进行拼接，
*   使用垂直分割时（type=0），可以不进行catImage，因为是对原图进行操作的
********************************************************/
vector<cv::Mat> splitImage(cv::Mat image, int num, int type) 
{
	int rows = image.rows;
	int cols = image.cols;
	vector<cv::Mat> v;
	if (type == 0) {//垂直分割
		for (size_t i = 0; i < num; i++) 
		{
			int star = rows / num * i;
			int end = rows / num * (i + 1);
			if (i == num - 1) 
			{
				end = rows;
			}
			//cv::Mat b = image.rowRange(star, end);
			v.push_back(image.rowRange(star, end));
		}
	}
	else if (type == 1) //水平分割
	{
		for (size_t i = 0; i < num; i++) 
		{
			int star = cols / num * i;
			int end = cols / num * (i + 1);
			if (i == num - 1) 
			{
				end = cols;
			}
			//cv::Mat b = image.colRange(star, end);
			/*解决水平分割的Bug:必须clone()*/
			v.push_back(image.colRange(star, end).clone());
		}
	}
	return  v;
}

/********************************************************
*	@brief       : 实现图像拼接，
*	@param  v    :
*	@param  type : 0：垂直拼接，1：水平拼接
*	@return      : Mat
********************************************************/
Mat catImage(vector<Mat> v, int type) 
{
	Mat dest = v.at(0);
	for (size_t i = 1; i < v.size(); i++)
	{
		if (type == 0)//垂直拼接
		{
			vconcat(dest, v.at(i), dest);
		}
		else if (type == 1)//水平拼接
		{
			hconcat(dest, v.at(i), dest);
		}
	}
	return dest;
}

int main() 
{
	string path = "png\\1.png";
	Mat src = imread(path);
	printf("image size =  w=%d, h=%d\n", src.cols, src.rows);

	Mat image1 = src.clone();
	Mat image2 = src.clone();
	imshow("src", src);

	double T0 = static_cast<double>(getTickCount());
	/*不使用多线程图像处理*/

	blur(image1, image1, Size(7, 7));
	//GaussianBlur(image1, image1, Size(9, 9), 0, 0);
	//dilate(image1, image1, Mat(), Point(-1, -1), 5);
	double T1 = static_cast<double>(getTickCount());

	/*使用多线程图像处理*/
	int type = 0;
	vector<Mat> v = splitImage(image2, THREAD_NUMS, type);
	paramThread args[THREAD_NUMS];

	for (size_t i = 0; i < THREAD_NUMS; i++)
	{
		args[i].h = v.at(i).rows;
		args[i].w = v.at(i).cols;
		args[i].data = v.at(i).data;
	}

	thread pt1(threadProcess, (void *)(&args[0]));
	thread pt2(threadProcess, (void *)(&args[1]));
	thread pt3(threadProcess, (void *)(&args[2]));
	//thread pt4(threadProcess, (void *)(&args[3]));
	//thread pt5(threadProcess, (void *)(&args[4]));
	//thread pt6(threadProcess, (void *)(&args[5]));
	//thread pt7(threadProcess, (void *)(&args[6]));
	//thread pt8(threadProcess, (void *)(&args[7]));

	pt1.join();
	pt2.join(); 
	pt3.join();
	//pt4.join();
	//pt5.join();
	//pt6.join();
	//pt7.join();
	//pt8.join();

	Mat dest = catImage(v, type);
	double T2 = static_cast<double>(cv::getTickCount());
	printf("       run times = %3.3fms\n", (T1 - T0) * 1000 / getTickFrequency());
	printf("Thread run times = %3.3fms\n,", (T2 - T1) * 1000 / getTickFrequency());

	imshow("dest", dest); 
	//waitKey(30);
	imshow("image2", image2); 
	//waitKey(30);

	waitKey(0);
	return 0;
}
#endif

#ifdef DXC

vector<Mat> Frame;
vector<Mat> Dst;
Mat gray, diff;
mutex m;
void run1()
{
	VideoCapture cap;
	Mat frame;
	cap.open("avi\\3.avi");
	while (1)
	{
		cap.read(frame);
		if (!frame.data)
		{
			break;
		}
		//m.lock();
		Frame.push_back(frame);
		//m.unlock();
		//Sleep(10);
	}
	cout << "size: " << Frame.size();
}
void run2()
{
	while (1)
	{
		
	}
}
void run3()
{
	while (1)
	{
		
	}
}
void run4()
{
	while (1)
	{
		for (int i = 0; i < Dst.size(); i++)
		{
			Mat dst;
			dst = Dst[i].clone();
			//m.lock();
			imshow("dst", dst);
			waitKey(10);
			//m.unlock();
			if (i == Dst.size() - 1)
			{
				break;
			}
		}
	}
}



int main()
{
	thread t1(run1);
	thread t2(run2);
	thread t3(run3);
	thread t4(run4);

	t1.join();
	t2.join();
	t3.join();
	t4.join();

	cout << "over!" << endl;
	system("pause");
	return 0;
}

#endif

#ifdef HUATU
//int main()
//{
//	Mat data;
//	data = Mat::zeros(400, 700, CV_8UC3);
//	namedWindow("data");
//	vector<Point> a;
//	Point p1;
//	Point p2;
//	
//	for (int i = 0; i < 700; i++)
//	{
//		Point p;
//		p.x = i;
//		p.y = abs(400 - i);
//		cout << p.x << " " << p.y << endl;
//		a.push_back(p);
//	}
//	for (int i = 1; i < 700 - 1; i++)
//	{
//		if (i == 1)
//		{
//			Point p1;
//			Point p2;
//			p1 = a[0];
//			p2 = a[1];
//			cout << p1.x << " " << p1.y << " " << p2.x << " " << p2.y << endl;
//			line(data, p1, p2, Scalar(255, 100, 0), 5, LINE_8, 0);
//			imshow("data", data);
//			waitKey(50);
//		}
//		else
//		{
//			Point p1;
//			Point p2;
//			p1 = a[i - 1];
//			p2 = a[i];
//			cout << p1.x<< " " << p1.y << " "<< p2.x << " " << p2.y << endl;
//			line(data, p1, p2, Scalar(255, 100, 0), 5, LINE_8, 0);
//			imshow("data", data);
//			waitKey(50);
//		}
//		/*line(data, p1, p2, Scalar(255, 100, 0), 5, LINE_8, 0);
//		imshow("data", data);
//		waitKey(50);*/
//	}
//	waitKey(0);
//	return 0;
//}
int main()
{
	vector<int> a;
	for (int i = 0; i < 500; i++)
	{
		a.push_back(i);
	}
	Drawline d(a);
	d.draw(500, 500);
	waitKey(0);
	return 0;
}
#endif




