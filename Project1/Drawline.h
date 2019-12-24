#pragma once
#include <vector>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;
class Drawline
{
public:
	Drawline(vector<int> v1p, vector<int> v2p);
	Drawline(vector<int> v0p);
	void draw_two(int x, int y);
	void draw(int x, int y);
	~Drawline();
private:
	vector<Point> vp1;
	vector<Point> vp2;
	vector<Point> vp;
	vector<int> ip1;
	vector<int> ip2;
	vector<int> ip;
};

