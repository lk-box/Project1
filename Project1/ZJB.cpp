#include "ZJB.h"
#include "PNPSolver.h"
#include "ArrorAttitudeAlgorithm.h"
#include "Result.h"

//#define RESIZE
#define LZH
//#define BLZ


static pthread_mutex_t a = PTHREAD_MUTEX_INITIALIZER;
enum
{
	WIDTH_GREATER_THAN_HEIGHT,
	ANGLE_TO_UP
};

typedef struct lnode
{
	float narea;
	int node;
}sor;


/******************************************************************************
功能： 初始化一些参数
参数：
	*_Filename 要打开的Video文件名
*/

ZJB::ZJB()
{
	k = 0;
}


cv::RotatedRect& adjustRec(cv::RotatedRect& rec, const int mode)
{
	using std::swap;

	float& width = rec.size.width;
	float& height = rec.size.height;
	float& angle = rec.angle;

	if (mode == WIDTH_GREATER_THAN_HEIGHT)
	{
		if (width < height)
		{
			swap(width, height);
			angle += 90.0;
		}
	}

	while (angle >= 90.0) angle -= 180.0;
	while (angle < -90.0) angle += 180.0;

	if (mode == ANGLE_TO_UP)
	{
		if (angle >= 45.0)
		{
			swap(width, height);
			angle -= 90.0;
		}
		else if (angle < -45.0)
		{
			swap(width, height);
			angle += 90.0;
		}
	}
	return rec;
}

bool ZJB::suitble(RotatedRect &r1, RotatedRect &r2)
{
	double dAngle = abs(r1.angle - r2.angle);
	while (dAngle > 180)
	{
		dAngle -= 180;
	}
	if (dAngle < diff_angle || 180 - dAngle < diff_angle)
	{
		if ((r1.size.height*0.7f < r2.size.height
			&& r1.size.height*1.3f > r2.size.height)
			|| (r2.size.height*0.7f < r1.size.height
				&& r2.size.height*1.3f > r1.size.height)) // 0.7   1.3
		{
			float armor_width = fabs(r1.center.x - r2.center.x);
			if (armor_width > (r1.size.width + r2.size.width) )
			{
				float h_max = (r1.size.height + r2.size.height) / 2.0f;
				// 两个灯条高度差不大
				if (fabs(r1.center.y - r2.center.y) < 0.7f * h_max)
				{
					return true;
				}
			}
		}
	}
	return false;
}


float saturate_x(float a)
{
	if (a >= 640)
	{
		a = 640;
	}
	else if (a <= 0)
	{
		a = 0;
	}
	return a;
}
float saturate_y(float a)
{
	if (a >= 480)
	{
		a = 480;
	}
	else if (a <= 0)
	{
		a = 0;
	}
	return a;
}
int svmPredict(const char *_Filename, const Mat & image, Ptr<SVM> svm)
{

	//Ptr<ml::SVM> svm = Algorithm::load<ml::SVM>(_Filename);
	//检测样本    
	Mat dstImage = image.clone();

	Mat src = image.clone();
	if (!src.data)
	{
		cout << "empty!" << endl;
	}
	Mat trainTempImg = Mat::zeros(Size(48, 48), CV_8UC3);

	resize(src, trainTempImg, trainTempImg.size(), 0, 0);

	HOGDescriptor *hog = new HOGDescriptor(Size(48, 48), Size(8, 8), Size(4, 4), Size(4, 4), 9);
	vector<float>descriptors;//存放结果       
	hog->compute(trainTempImg, descriptors); //Hog特征计算  , Size(1, 1), Size(0, 0)    
	//cout << "HOG dims: " << descriptors.size() << endl;  //打印Hog特征维数  ，这里是4356
	Mat SVMtrainMat;
	SVMtrainMat.create(1, descriptors.size(), CV_32FC1);

	int n = 0;
	for (vector<float>::iterator iter = descriptors.begin(); iter != descriptors.end(); iter++)
	{
		SVMtrainMat.at<float>(0, n) = *iter;
		n++;
	}

	int ret = svm->predict(SVMtrainMat);//检测结果

	return ret;
}
vector<RotatedRect> ZJB::armorDetect(vector<RotatedRect> vEllipse, Mat frame)
{
	vector<RotatedRect> vRlt;
	RotatedRect armor;
	int nL, nW;
	double dAngle;
	vRlt.clear();
	if (vEllipse.size() < 2)
	{
		return vRlt;
	}
	for (unsigned int i = 0; i < vEllipse.size() - 1; i++)
	{
		for (unsigned int j = i + 1; j < vEllipse.size(); j++)
		{
			if (suitble(vEllipse[i], vEllipse[j]))
			{
				armor.center.x = (vEllipse[i].center.x + vEllipse[j].center.x) / 2;
				armor.center.y = (vEllipse[i].center.y + vEllipse[j].center.y) / 2;
				armor.angle = (vEllipse[i].angle + vEllipse[j].angle) / 2;
				//if (180 - dAngle < 20)
				//{
				//	armor.angle += 90;
				//}
				nL = (vEllipse[i].size.height + vEllipse[j].size.height) / 2;
				nW = sqrt((vEllipse[i].center.x - vEllipse[j].center.x) * (vEllipse[i].center.x - vEllipse[j].center.x) + (vEllipse[i].center.y - vEllipse[j].center.y) * (vEllipse[i].center.y - vEllipse[j].center.y)); //装甲的宽度等于两侧LED所在旋转矩形中心坐标的距离

				if (nL < nW)
				{
					armor.size.height = nL;
					armor.size.width = nW;
				}
				else
				{
					int tmp;
					tmp = nW;
					nW = nL;
					nL = tmp;
					armor.size.height = nL;
					armor.size.width = nW;
				}
				

				//vRlt.push_back(armor);armor.size.height * 3 > armor.size.width &&
				//vRlt.push_back(armor);
				//adjustRec(armor, WIDTH_GREATER_THAN_HEIGHT);

				//Mat roi = frame(Range(saturate_y(armor.center.y - (armor.size.height / 2)), saturate_y(armor.center.y + (armor.size.height / 2))), Range(saturate_x(armor.center.x - (armor.size.width / 2)), saturate_x( armor.center.x + (armor.size.width / 2))));
				//cvtColor(roi, roi, COLOR_BGR2GRAY);
				//gammaTransform(roi, 3.33);
				//imshow("roi", roi);
				//int average_intensity = static_cast<int>(mean(roi).val[0]);//&& average_intensity < 60&& average_intensity > 50 && average_intensity < 70
				//cout << average_intensity << endl;

				if (armor.size.height * 6 > armor.size.width && armor.size.height < armor.size.width )//&& suitble(vEllipse[i], vEllipse[j])
				{
					vRlt.push_back(armor);
				}
			}
		}
	}
	return vRlt;
}





int daJinThreshold(Mat grayImg)
{
	if (grayImg.channels() != 1)
		cvtColor(grayImg, grayImg, COLOR_BGR2GRAY);

	int width = grayImg.cols;
	int height = grayImg.rows;
	int x = 0, y = 0;
	int pixelCount[256];
	float pixelPro[256];
	int pixelSum = width * height, threshold = 0;


	//初始化  
	for (uint nI = 0; nI < 256; nI++)
	{
		pixelCount[nI] = 0;
		pixelPro[nI] = 0;
	}

	//统计灰度级中每个像素在整幅图像中的个数  
	for (uint nI = y; nI < height; nI++)
	{
		uchar* pixData = grayImg.ptr<uchar>(nI);
		for (uint nJ = x; nJ < width; nJ++)
		{
			pixelCount[pixData[nJ]] = pixData[nJ];
		}
	}


	//计算每个像素在整幅图像中的比例  
	for (uint nI = 0; nI < 256; nI++)
	{
		pixelPro[nI] = (float)(pixelCount[nI]) / (float)(pixelSum);
	}

	//经典ostu算法,得到前景和背景的分割  
	//遍历灰度级[0,255],计算出方差最大的灰度值,为最佳阈值  
	float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
	for (uint nI = 0; nI < 256; nI++)//i为阈值
	{
		w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;

		for (uint nJ = 0; nJ < 256; nJ++)//遍历阈值数组当i=0，1，2，3...255
		{
			if (nJ <= nI) //背景部分  
			{
				//以i为阈值分类，第一类总的概率  
				w0 += pixelPro[nJ];
				u0tmp += nJ * pixelPro[nJ];
			}
			else       //前景部分  
			{
				//以i为阈值分类，第二类总的概率  
				w1 += pixelPro[nJ];
				u1tmp += nJ * pixelPro[nJ];
			}
		}

		u0 = u0tmp / w0;        //第一类的平均灰度  
		u1 = u1tmp / w1;        //第二类的平均灰度  
		u = u0tmp + u1tmp;      //整幅图像的平均灰度  
		//计算类间方差  
		deltaTmp = w0 * (u0 - u)*(u0 - u) + w1 * (u1 - u)*(u1 - u);
		//找出最大类间方差以及对应的阈值  
		if (deltaTmp > deltaMax)
		{
			deltaMax = deltaTmp;
			threshold = nI;
		}
	}
	return threshold;
}
void getArrorNumBinary(Mat &src)
{
	//@@@直方图均衡化
	equalizeHist(src, src);
	int thre = daJinThreshold(src);
#ifdef MLStep20
	printf("thre=%d\n", thre);
	imshow("equalizeHist", src);
#endif // MLStep20
	//@@@大津二值化
	medianBlur(src, src, 5);
	threshold(src, src, thre, 255, THRESH_BINARY);
	//@@@去除椒盐噪点
	
}
int  ZJB::drawBox(RotatedRect box, Mat img, Ptr<SVM> svm)
{
	Point2f pt[4];
	int i;
	for (i = 0; i < 4; i++)
	{
		pt[i].x = 0;
		pt[i].y = 0;
	}
	box.points(pt);
	
	Mat imageROI;
	int x = pt[3].x + box.size.width * ((1 - 2 / 3.0) / 3.0);
	int y = pt[3].y - box.size.height / 3;
	int wh = box.size.width * 4 / 5;
	int ht = box.size.height * 2;


	pt[3].x = x;
	pt[3].y = y;
	pt[0].x = x + wh;
	pt[0].y = y;
	pt[1].x = x + wh;
	pt[1].y = y + ht;
	pt[2].x = x;
	pt[2].y = y + ht;

	line(img, pt[0], pt[1], CV_RGB(255, 0, 255), 1, 8, 0);
	line(img, pt[1], pt[2], CV_RGB(255, 0, 255), 1, 8, 0);
	line(img, pt[2], pt[3], CV_RGB(255, 0, 255), 1, 8, 0);
	line(img, pt[3], pt[0], CV_RGB(255, 0, 255), 1, 8, 0);
	//int x;
	//int y; 
	//int wh;
	//int ht;
	//if (pt[3].x < pt[2].x)
	//{
	//	x = pt[3].x;
	//}
	//else
	//{
	//	x = pt[2].x;
	//}
	//if (pt[3].y < pt[0].y)
	//{
	//	y = pt[3].y;
	//}
	//else
	//{
	//	y = pt[0].y;
	//}
	//if (pt[1].x > pt[0].x)
	//{
	//	wh = pt[1].x - x;
	//}
	//else
	//{
	//	wh = pt[0].x - x;
	//}
	//if (pt[1].y > pt[2].y)
	//{
	//	ht = pt[1].y - y;
	//}
	//else
	//{
	//	ht = pt[2].y - y;
	//}
	//line(img, pt[0], pt[1], CV_RGB(255, 0, 255), 1, 8, 0);
	//line(img, pt[1], pt[2], CV_RGB(255, 0, 255), 1, 8, 0);
	//line(img, pt[2], pt[3], CV_RGB(255, 0, 255), 1, 8, 0);
	//line(img, pt[3], pt[0], CV_RGB(255, 0, 255), 1, 8, 0);

	//if (x <= 0)
	//{
	//	x = 0;
	//}
	//if (y <= 0)
	//{
	//	y = 0;
	//}
	//if (x + wh >= 640)
	//{
	//	wh = 640 - x;
	//}
	//if (y + ht >= 480)
	//{
	//	ht = 480 - y;
	//}

	//if (x <= 0 || y <= 0 || x >= 640 || y >= 480 || x + wh >= 640 || y + ht >= 480)
	//{
	//	return 0;
	//}

	//imageROI = img(Rect(x, y, wh , ht));

	//k++;

	//cvtColor(imageROI, imageROI, COLOR_BGR2GRAY);
	//resize(imageROI, imageROI, cv::Size(48, 48), (0, 0), (0, 0), cv::INTER_LINEAR);  //将图片调整为相同的大小
	//getArrorNumBinary(imageROI);

	////string Img_Name = "D:\\RM\\Opencv\\lk\\data\\t4\\car" + to_string(k) + ".png";

	//int a = 0;
	//a = svmPredict("xml\\svm.xml", imageROI, svm);
	//
	//if (a != 0)
	//{
	//	//line(img, pt[0], pt[1], CV_RGB(255, 0, 255), 1, 8, 0);
	//	//line(img, pt[1], pt[2], CV_RGB(255, 0, 255), 1, 8, 0);
	//	//line(img, pt[2], pt[3], CV_RGB(255, 0, 255), 1, 8, 0);
	//	//line(img, pt[3], pt[0], CV_RGB(255, 0, 255), 1, 8, 0);
	//	cout << "这是" << a << "号装甲板！" << endl;
	//}
	//else
	//{
	//	cout << "这不是装甲板！" << endl;
	//	return 0;
	//}
	//
	//imshow("roi", imageROI);
	//waitKey(1);
	
	//imwrite(Img_Name, imageROI);
	return 1;

}
void ZJB::draw_light(RotatedRect box, Mat img)
{
	Point2f pt[4];
	int i;
	for (i = 0; i < 4; i++)
	{
		pt[i].x = 0;
		pt[i].y = 0;
	}
	box.points(pt);

	/*line(img, pt[0], pt[1], CV_RGB(0, 255, 255), 1, 8, 0);
	line(img, pt[1], pt[2], CV_RGB(0, 255, 255), 1, 8, 0);
	line(img, pt[2], pt[3], CV_RGB(0, 255, 255), 1, 8, 0);
	line(img, pt[3], pt[0], CV_RGB(0, 255, 255), 1, 8, 0);*/
}


void ZJB::b_track_armor(Mat srcimage, Result &r, FileStorage fs, Ptr<SVM> svm)
{
	if (srcimage.data)
	{
		Mat frame;
		Mat mask;
		Size imgSize;
		imgSize = frame.size();
		//frame = srcimage.clone();
		
		if (r.r_x != 0)
		{
			mask = Mat::zeros(srcimage.size(), CV_8U);
			rectangle(mask, Point(r.r_center_x - r.r_width, r.r_center_y - r.r_height), Point(r.r_center_x + r.r_width , r.r_center_y + r.r_height), Scalar::all(255), -1);
			srcimage.copyTo(frame, mask);
			//pthread_mutex_lock(&a);
			//resize(mask, mask, Size(r.r_width + 20, r.r_height + 20), (0, 0), (0, 0), 3);
			//imshow("mask", frame);
			//pthread_mutex_unlock(&a);
			//imgSize = mask.size();
		}
		else
		{
			frame = srcimage.clone();
		}
		
		Mat rimg = Mat(imgSize, CV_8UC1);
		Mat gimg = Mat(imgSize, CV_8UC1);
		Mat bimg = Mat(imgSize, CV_8UC1);

		Mat diff = Mat(imgSize, CV_8UC1);
		Mat th = Mat(imgSize, CV_8UC1);
		Mat th1 = Mat(imgSize, CV_8UC1);
		Mat rlt = Mat(imgSize, CV_8UC1);

		vector<vector<Point>> contour;
		RotatedRect s;
		vector<RotatedRect> vArmor;
		vector<RotatedRect> vRlt;
		vector<RotatedRect> vEllipse;

		Mat gray = Mat(imgSize, CV_8UC3);
		cvtColor(frame, gray, COLOR_BGR2GRAY);

		Mat channels[3];
		split(frame, channels);

		bimg = channels[0];
		gimg = channels[1];
		rimg = channels[2];
		
		if (ch == 0)
		{
			diff = bimg - rimg;
		}
		else
		{
			diff = rimg - bimg;
		}
		dilate(diff, diff, Mat(), Point(-1, -1), 3);
		threshold(diff, th1, color_th, 255, THRESH_BINARY);	
		threshold(gray, th, gray_th, 255, THRESH_BINARY);

#ifdef LZH
		
		bitwise_and(th, th1, rlt);

		pthread_mutex_lock(&a);
		imshow("rlt", rlt);
		pthread_mutex_unlock(&a);

		findContours(rlt, contour, RETR_EXTERNAL, CHAIN_APPROX_NONE);
		
		for (int k = 0; k < contour.size(); k++)
		{
			double area = contourArea(contour[k]);
			if (area < min_area || area > max_area)//
			{
				continue;
			}
			double length = arcLength(contour[k], true); // 灯条周长
			if (length > 3 && length < 2000)
			{
				if (contour[k].size() > 6)
				{
					bool Flag = true;
					s = fitEllipse(contour[k]);
					adjustRec(s, ANGLE_TO_UP);
					if (s.size.height > s.size.width)//
					{
						draw_light(s, frame);
						vEllipse.push_back(s);
					}
				}
			}
		}
#endif 

#ifdef BLZ
		vector<vector<Point>> contours_diff;
		vector<vector<Point>> contours_gray;
		findContours(th1, contours_diff, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		findContours(th, contours_gray, RETR_EXTERNAL, CHAIN_APPROX_NONE);


		for (size_t i = 0; i < contours_gray.size(); i++)
		{
			double area = contourArea(contours_gray[i]);
			if (area < 7.0 || 5000 < area)//
			{
				continue;
			}
			for (size_t ii = 0; ii < contours_diff.size(); ii++)
			{
				if (pointPolygonTest(contours_diff[ii], contours_gray[i][0], false) >= 0.0)
				{
					double length = arcLength(contours_gray[i], true); // 灯条周长
					if (length > 13 && length < 4000)
					{
						if (contours_gray[i].size() > 10)
						{
							bool Flag = true;
							s = fitEllipse(contours_gray[i]);

							adjustRec(s, ANGLE_TO_UP);
							if ( s.size.height > s.size.width)//
							{
								draw_light(s, frame);
								vEllipse.push_back(s);
							}
						}
					}
				}
			}
		}
#endif
		vRlt = armorDetect(vEllipse, frame);

		vector<float> Dis;
		unsigned int const counts = vRlt.size();
		sor S[100];

		r.d_distance = 0.0;
		r.d_pitch = 0.0;
		r.d_yaw = 0.0;
		r.r_x = 0;
		r.r_y = 0;
		r.r_width = 680;
		r.r_height = 480;

		for (unsigned int i = 0; i < vRlt.size(); i++)
		{
			if (vRlt[i].angle < 90)
			{
				vRlt[i].angle += 180;
			}
			//drawBox(vRlt[i], frame, svm);

			int armor_size;
			if (vRlt[i].size.height * 7 / 2 > vRlt[i].size.width)
			{
				armor_size = 0;
			}
			else 
			{
				armor_size = 1;
			}


			ArrorAttitudeAlgorithm angle;
			double yaw = 0 , pitch = 0;
			angle.angleSover(frame, vRlt[i], yaw, pitch);
			r.d_pitch = pitch;
			r.d_yaw = yaw;
			r.d_distance = angle.angleSover(frame, vRlt[i], yaw, pitch);


			S[i].node = i;
			S[i].narea = vRlt[i].size.height * vRlt[i].size.width;
			Dis.push_back(S[i].narea);
		}
		sort(Dis.begin(), Dis.end());
		

		for (size_t i = 0; i < vRlt.size(); i++)
		{
			if (S[i].narea == Dis.back())
			{
				drawBox(vRlt[i], frame, svm);
				ArrorAttitudeAlgorithm angle;
				double yaw, pitch;
				angle.angleSover(frame, vRlt[i], yaw, pitch);
				
				r.d_distance = Dis[0];
				
				r.d_pitch = pitch;
				r.d_yaw = yaw;

				r.r_center_x = vRlt[i].center.x;
				r.r_center_y = vRlt[i].center.y;

				r.r_x = vRlt[i].center.x - vRlt[i].size.width;
				r.r_y = vRlt[i].center.y - vRlt[i].size.height;
				r.r_width = vRlt[i].size.width;
				r.r_height = vRlt[i].size.height;

				break;
			}
			
		}
		
		

#ifdef RESIZE
		resize(th, th, Size(th.cols / 2, th.rows / 2), (0, 0), (0, 0), 3);
		resize(th1, th1, Size(th1.cols / 2, th1.rows / 2), (0, 0), (0, 0), 3);
		resize(frame, frame, Size(frame.cols / 2, frame.rows / 2), (0, 0), (0, 0), 3);
#endif
		pthread_mutex_lock(&a);
		//imshow("gray", th);
		//imshow("diff", th1);
		imshow("frame", frame);
		pthread_mutex_unlock(&a);
		
		if (waitKey(1) == 32)
		{
			waitKey(0);
		}
		Dis.clear();
		vEllipse.clear();
		vRlt.clear();
		vArmor.clear();
	}
	
}



