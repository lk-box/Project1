#include "ZJB.h"
#include "PNPSolver.h"
#include "ArrorAttitudeAlgorithm.h"
#include "Result.h"


#define tsize 0.3
#define tangle 5//8
//#define RESIZE
#define LZH
//#define BLZ


mutex a;
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
ZJB::ZJB(const char *_Filename)
{
	//FileStorage fs("test.xml", FileStorage::APPEND);
	//fs.release();
	FileStorage fs("armor.xml", FileStorage::READ);
	thresh = (int)fs["thresh"];
	thresh1 = (int)fs["thresh1"];
	ch = (int)fs["ch"];
	fs.release();

	cap.open(_Filename);
	if (!cap.isOpened())
	{
		cout << "不能初始化摄像头！";
	}
}
ZJB::ZJB(int i)
{
	//FileStorage fs("test.xml", FileStorage::APPEND);
	//fs.release();
	k = 0;
	cap.open(i);
	if (!cap.isOpened())
	{
		cout << "不能初始化摄像头！";
	}
	cap.set(CAP_PROP_FRAME_WIDTH, 1024);//宽度 
	cap.set(CAP_PROP_FRAME_HEIGHT, 768);//高度
}
ZJB::ZJB()
{
	
}


Mat gammaTransform(Mat &srcImage, float kFactor)//伽马变换
{
	unsigned char LUT[256];
	for (int i = 0; i < 256; i++)
	{
		float f = (i + 0.5f) / 255;//归一化
		f = (float)(pow(f, kFactor));//指数预补偿补偿
		LUT[i] = saturate_cast<uchar>(f*255.0f - 0.5f);//一张图片中也就256中值，开始存储256种查找表
	}
	Mat resultImage = srcImage.clone();

	for (int r = 0; r < resultImage.rows; r++)
	{
		uchar* imgData = resultImage.ptr<uchar>(r);
		for (int c = 0; c < resultImage.cols*resultImage.channels(); c++)
		{
			imgData[c] = LUT[imgData[c]];
		}
	}
	return resultImage;
}

double calc_distance(Point2f p1, Point2f p2)
{
	return pow(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2), 0.5);
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


bool suitble(RotatedRect &r1, RotatedRect &r2)
{
	double dAngle = abs(r1.angle - r2.angle);
	while (dAngle > 180)
	{
		dAngle -= 180;
	}
	if (dAngle < tangle || 180 - dAngle < tangle)
	{
		if ((r1.size.height*0.8f < r2.size.height
			&& r1.size.height*1.2f > r2.size.height)
			|| (r2.size.height*0.8f < r1.size.height
				&& r2.size.height*1.2f > r1.size.height)) // 0.7   1.3
		{
			float armor_width = fabs(r1.center.x - r2.center.x);
			if (armor_width > (r1.size.width + r2.size.width) )
			{
				float h_max = (r1.size.height + r2.size.height) / 2.0f;
				// 两个灯条高度差不大
				if (fabs(r1.center.y - r2.center.y) < 0.5f * h_max)
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
				if (armor.size.height * 6 > armor.size.width && armor.size.height * 3/ 2 < armor.size.width )//&& suitble(vEllipse[i], vEllipse[j])
				{
					vRlt.push_back(armor);
				}

			}
		}
	}
	return vRlt;
}


void ZJB::drawBox(RotatedRect box, Mat img)
{
	Point2f pt[4];
	int i;
	for (i = 0; i < 4; i++)
	{
		pt[i].x = 0;
		pt[i].y = 0;
	}
	box.points(pt);
	line(img, pt[0], pt[1], CV_RGB(255, 0, 255), 2, 8, 0);
	line(img, pt[1], pt[2], CV_RGB(255, 0, 255), 2, 8, 0);
	line(img, pt[2], pt[3], CV_RGB(255, 0, 255), 2, 8, 0);
	line(img, pt[3], pt[0], CV_RGB(255, 0, 255), 2, 8, 0);
}
void ZJB::drawBoxd(RotatedRect box, Mat img)
{
	Point2f pt[4];
	int i;
	for (i = 0; i < 4; i++)
	{
		pt[i].x = 0;
		pt[i].y = 0;
	}
	box.points(pt);

	line(img, pt[0], pt[1], CV_RGB(0, 255, 0), 1, 8, 0);
	line(img, pt[1], pt[2], CV_RGB(0, 255, 0), 1, 8, 0);
	line(img, pt[2], pt[3], CV_RGB(0, 255, 0), 1, 8, 0);
	line(img, pt[3], pt[0], CV_RGB(0, 255, 0), 1, 8, 0);
}


//void ZJB::track_armor()
//{
//	cap >> frame;
//	Size imgSize;
//	imgSize = frame.size();
//	Mat rimg = Mat(imgSize, CV_8UC1);
//	Mat gimg = Mat(imgSize, CV_8UC1);
//	Mat bimg = Mat(imgSize, CV_8UC1);
//
//	Mat binary = Mat(imgSize, CV_8UC1);
//	Mat diff = Mat(imgSize, CV_8UC1);
//	Mat rlt = Mat(imgSize, CV_8UC1);
//	Mat th = Mat(imgSize, CV_8UC1);
//	Mat th1 = Mat(imgSize, CV_8UC1);
//
//	dstimg.create(frame.size(), frame.type());
//
//	vector<vector<Point>> contour;
//	RotatedRect s;
//
//	vector<RotatedRect> vEllipse;
//	vector<RotatedRect> vRlt;
//	vector<RotatedRect> vArmor;
//
//	namedWindow("frame", 2);
//
//	FileStorage fs("test.xml", FileStorage::READ);
//
//	int thresh = (int)fs["thresh"];
//	int thresh1 = (int)fs["thresh1"];
//	ch = (int)fs["ch"];
//
//	fs.release();
//	createTrackbar("thresh", "frame", &thresh, 255, 0);
//	createTrackbar("thresh1", "frame", &thresh1, 255, 0);
//	createTrackbar("ch", "frame", &ch, 1, 0);
//
//	Mat dst1 = Mat(imgSize, CV_8UC3);
//	Mat gray = Mat(imgSize, CV_8UC3);
//	int q = 0;
//	while (1)
//	{
//		if (cap.read(frame))
//		{
//			cvtColor(frame, gray, COLOR_BGR2GRAY);
//			Mat channels[3];
//			split(frame, channels);
//
//			bimg = channels[0];
//			rimg = channels[2];
//
//			if (ch == 1)         
//			{
//				diff = rimg - bimg;
//				dilate(diff, diff, Mat(), Point(-1, -1), 5);
//
//				threshold(diff, th1, thresh1, 255, 0);
//				imshow("th1", th1);
//				threshold(gray, th, thresh, 255, 0);  //th 71
//				imshow("th", th);
//				bitwise_and(th, th1, rlt);
//
//				imshow("rlt", rlt);
//
//			}
//			else if (ch == 0)
//			{
//				diff = bimg - rimg;
//				dilate(diff, diff, Mat(), Point(-1, -1), 5);
//
//				threshold(diff, th1, thresh1, 255, 0);
//				imshow("th1", th1);
//				threshold(gray, th, thresh, 255, 0);  
//				imshow("th", th);
//				bitwise_and(th, th1, rlt);
//
//				imshow("rlt", rlt);
//
//				//brightAdjust(bimg, binary, 1, bright);
//				//blur(bimg, binary, Size(3, 3));
//				//getDiffImage(bimg, rimg, binary, thresh);
//			}
//
//			findContours(rlt, contour, RETR_EXTERNAL, CHAIN_APPROX_NONE);
//			//drawContours(frame, contour, -1, Scalar(0, 255, 0), 2);
//
//			for (int k = 0; k < contour.size(); k++)
//			{
//				double length = arcLength(contour[k], true); // 灯条周长
//				if (length > 15 && length < 2000)
//				{
//					if (contour[k].size() > 6)
//					{
//						Flag = true;
//						s = fitEllipse(contour[k]);
//
//						for (int i = 0; i < Size2; i++)
//						{
//							for (int j = 0; j < Size2; j++)
//							{
//								if (s.center.y - Size1 + j > 0
//									&& s.center.y - Size1 + j < 480
//									&& s.center.x - Size1 + i > 0
//									&& s.center.x - Size1 + i < 640)
//								{
//									Vec3b v3b = frame.at<Vec3b>((int)(s.center.y - Size1 + j), (int)s.center.x - Size1 + i);
//									if (v3b[2] < 80 || v3b[1] < 80 || v3b[0] < 80)
//									{
//										Flag = false;
//									}
//								}
//							}
//						}
//						if (Flag && s.size.width * 3 < s.size.height && (s.angle < 50 || s.angle > 130))//
//						{
//							drawBoxd(s, frame);
//							cout << endl;
//							vEllipse.push_back(s);
//						}
//					}//end of if (contour[i].size() > 10)
//				}
//			}//end of for (int i = 0; i < contour.size(); i++)
//
//			vRlt = armorDetect(vEllipse);
//			//cout << vRlt.size();
//			for (unsigned int i = 0; i < vRlt.size(); i++)
//			{
//				if (vRlt[i].angle < 90)
//				{
//					vRlt[i].angle += 180;
//				}
//				//cout << vRlt[i].angle;
//				drawBox(vRlt[i], frame);
//
//				PNPSolver p;
//				p.read_xml("rm.xml");
//				p.get_distance(vRlt[i]);
//			}
//			imshow("frame", frame);
//			
//			if (waitKey(1) == 32)
//			{
//				waitKey(0);
//			}
//			if (waitKey(1) == 27)
//			{
//				break;
//			}
//			vEllipse.clear();
//			vRlt.clear();
//			vArmor.clear();
//		}
//		else
//		{
//			break;
//		}
//	}
//	cap.release();
//}



Mat ZJB::get_image()
{
	Mat frame;
	cap >> frame;
	return frame;
}

void ZJB::b_track_armor(Mat srcimage, Result &r)
{
	if (srcimage.data)
	{
		Mat frame;
		Mat mask;

		frame = srcimage.clone();
		
		//if (r.r_x != 0)
		//{
		//	mask = Mat::zeros(srcimage.size(), CV_8U);
		//	rectangle(mask, Point(r.r_center_x - r.r_width, r.r_center_y - r.r_height), Point(r.r_center_x + r.r_width , r.r_center_y + r.r_height), Scalar::all(255), -1);
		//	srcimage.copyTo(frame, mask);

		//	//a.lock();
		//	//resize(mask, mask, Size(mask.cols / 2, mask.rows / 2), (0, 0), (0, 0), 3);
		//	//imshow("mask", frame);
		//	//a.unlock();
		//}
		//else
		//{
		//	frame = srcimage.clone();
		//}
		
		Size imgSize;
		imgSize = frame.size();
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

		a.lock();
		FileStorage fs("armor.xml", FileStorage::READ);
		int thresh = (int)fs["thresh"];
		int thresh1 = (int)fs["thresh1"];
		int ch = (int)fs["ch"];
		fs.release();
		a.unlock();
		
		Mat gray = Mat(imgSize, CV_8UC3);

		cvtColor(frame, gray, COLOR_BGR2GRAY);
		Mat channels[3];
		split(frame, channels);

		bimg = channels[0];
		gimg = channels[1];
		rimg = channels[2];

		if (ch == 1)
		{
			subtract(rimg, gimg, diff);
		}
		else if (ch == 0)
		{
			subtract(bimg, rimg, diff);
		}
		//blur(gray, gray, Size(3, 3));
		//dilate(gray, gray, Mat(), Point(-1, -1), 1);
		//erode(gray, gray, Mat(), Point(-1, -1), 1);
		threshold(gray, th, thresh, 255, CV_THRESH_BINARY);

		//imshow("gray", th);
		//dilate(diff, diff, Mat(), Point(-1, -1), 1);
		GaussianBlur(diff, diff, Size(3, 3), 0, 0);
		
		//erode(diff, diff, Mat(), Point(-1, -1), 1);

		/*Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
		morphologyEx(diff, diff, MORPH_CLOSE, element);*/
#ifdef LZH
		dilate(diff, diff, Mat(), Point(-1, -1), 2);
#endif
		threshold(diff, th1, thresh1, 255, CV_THRESH_BINARY);
	/*	imshow("diff", th1);*/
		

#ifdef LZH
		bitwise_and(th, th1, rlt);
		a.lock();
		imshow("rlt", rlt);
		a.unlock();
		findContours(rlt, contour, RETR_EXTERNAL, CHAIN_APPROX_NONE);
		
		for (int k = 0; k < contour.size(); k++)
		{
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
						drawBoxd(s, frame);
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
								drawBoxd(s, frame);
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
		r.d_pitch = 1000.0;
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
			drawBox(vRlt[i], frame);
			PNPSolver p;
			Point3f point;
			p.read_xml("rm.xml");
			int armor_size;
			if (vRlt[i].size.height * 7 / 2 > vRlt[i].size.width)
			{
				armor_size = 0;
			}
			else 
			{
				armor_size = 1;
			}
			point = p.get_distance(vRlt[i], armor_size);

			/*ArrorAttitudeAlgorithm angle;
			double yaw = 0 , pitch = 0;
			angle.angleSover(frame, vRlt[i], yaw, pitch);
			r.d_pitch = pitch;
			r.d_yaw = yaw;
			r.d_distance = angle.angleSover(frame, vRlt[i], yaw, pitch);*/

			//cout << r.d_distance <<endl;
			//r.d_distance = angle.angleSover(frame, vRlt[i], yaw, pitch);
			S[i].node = i;
			//S[i].nheight = vRlt[i].size.height;
			//Dis.push_back(vRlt[i].size.height);

			S[i].narea = vRlt[i].size.height * vRlt[i].size.width;
			Dis.push_back(S[i].narea);
		}
		sort(Dis.begin(), Dis.end());
		

		for (size_t i = 0; i < vRlt.size(); i++)
		{
			if (S[i].narea == Dis.back())
			{
				drawBox(vRlt[i], frame);
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
		a.lock();
		imshow("gray", th);
		imshow("diff", th1);
		imshow("frame", frame);
		a.unlock();
		
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



