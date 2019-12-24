#include "Drawline.h"



Drawline::Drawline(vector<int> v0p)
{
	ip = v0p;
}

Drawline::Drawline(vector<int> v1p, vector<int> v2p)
{
	ip1 = v1p;
	ip2 = v2p;
}

void Drawline::draw(int x, int y)
{
	Mat data;
	data = Mat::zeros( y + 300, x+100, CV_8UC3);
	//namedWindow("data");
	vector<Point> a;
	Point p1;
	Point p2;

	for (int i = 0; i < x; i++)
	{
		Point p;
		p.x = i;
		p.y = y - ip[i];
		vp.push_back(p);
	}
	for (int i = 1; i < x; i++)
	{
		if (i == 1)
		{
			Point p1;
			Point p2;
			p1 = vp[0];
			p2 = vp[1];
			//cout << p1.x << " " << p1.y << " " << p2.x << " " << p2.y << endl;
			line(data, p1, p2, Scalar(255, 100, 0), 1, LINE_8, 0);
			imshow("data", data);
			waitKey(1);
		}
		else
		{
			Point p1;
			Point p2;
			p1 = vp[i - 1];
			p2 = vp[i];
			//cout << p1.x << " " << p1.y << " " << p2.x << " " << p2.y << endl;
			line(data, p1, p2, Scalar(255, 100, 0), 1, LINE_8, 0);
			imshow("data", data);
			waitKey(1);
		}

	}
	
}
void Drawline::draw_two(int x, int y)
{
	Mat data;
	data = Mat::zeros(2 * y, x + 100, CV_8UC3);
	//namedWindow("data");
	vector<Point> a;


	for (int i = 0; i < x; i++)
	{
		Point p1;
		Point p2;
		p1.x = i;
		p1.y = y - ip1[i];
		vp1.push_back(p1);
		p2.x = i;
		p2.y = y - ip2[i];
		vp2.push_back(p2);
	}
	for (int i = 1; i < x; i++)
	{
		if (i == 1)
		{
			Point p1;
			Point p2;
			Point p3;
			Point p4;
			p1 = vp1[0];
			p2 = vp1[1];
			p3 = vp2[0];
			p4 = vp2[1];

			//cout << p1.x << " " << p1.y << " " << p2.x << " " << p2.y << endl;
			line(data, p1, p2, Scalar(255, 0, 0), 1, LINE_8, 0);
			line(data, p3, p4, Scalar(0, 0, 255), 1, LINE_8, 0);
			imshow("data", data);
			waitKey(1);
		}
		else
		{
			Point p1;
			Point p2;
			Point p3;
			Point p4;
			p1 = vp1[i - 1];
			p2 = vp1[i];
			p3 = vp2[i - 1];
			p4 = vp2[i];

			//cout << p1.x << " " << p1.y << " " << p2.x << " " << p2.y << endl;
			line(data, p3, p4, Scalar(255, 0, 0), 1, LINE_8, 0);
			line(data, p1, p2, Scalar(0, 0, 255), 1, LINE_8, 0);
			imshow("data", data);
			waitKey(1);
		}

	}

}

Drawline::~Drawline()
{
}
