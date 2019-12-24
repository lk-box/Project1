#pragma once
#include <opencv2/opencv.hpp>
#include <math.h>
#include <iostream>
#include <fstream>
using namespace std;
using namespace cv;

// �������ڿ��ٽ��PNP���⣬˳������ռ�������ת�Լ�ͼ��ϵ�����ϵ������ϵ��ϵ����ͶӰ����
// Ĭ��ʹ��Gao��P3P+��ͶӰ����Ҫ������4��������
// ����˳��
// 1.��ʼ������
// 2.����SetCameraMatrix(),SetDistortionCoefficients()���ú�����ڲ����뾵ͷ�������
// 3.��Points3D��Points2D�����һһ��Ӧ���������
// 4.����Solve()�������м���
// 5.��RoteM, TransM, W2CTheta��������������
//
// ԭ��μ���http://www.cnblogs.com/singlex/category/911880.html
// Author��VShawn
// Ver:2016.11.25.0
class PNPSolver
{
public:
	PNPSolver();
	//��������ʼ��
	PNPSolver(Mat, Mat);
	~PNPSolver();

	/***********************λ�˹�������������**************************/
	vector<Point3f> Points3D;//�洢�ĸ������������
	vector<Point2f> Points2D;//�洢�ĸ����ͼ������

	/***********************λ�˹��ƽ��**************************/
	//����������ת������ƽ�ƾ���
	Mat RoteM, TransM;
	//����ϵ�����ϵ��������תŷ���ǣ�����ϵ�մ���ת��������������ϵ��ȫƽ�С�
	//��ת˳��Ϊx��y��z
	Point3f Theta_W2C;
	//���ϵ������ϵ��������תŷ���ǣ��������ϵ�մ���ת���������������ϵ��ȫƽ�С�
	//��ת˳��Ϊz��y��x
	Point3f Theta_C2W;

	//�������ϵ�У���������ϵԭ��Ow������
	Point3f Position_OwInC;
	//��������ϵ�У��������ϵԭ��Oc������
	Point3f Position_OcInW;


	/*********************���з���*****************************/

	//���ݼ�����Ľ��������������ͶӰ��ͼ�񣬷�����������㼯
	//ʹ��ǰ��Ҫ����Solve()������λ��
	//����Ϊ��������ϵ�ĵ����꼯��
	//���Ϊ��ͶӰ��ͼ���ϵ�ͼ�����꼯��
	vector<Point2f> WordFrame2ImageFrame(vector<Point3f> WorldPoints);



	//��������Ĳ�����ͼ������ת�������������
	//ʹ��ǰ��Ҫ����Solve()������λ��
	//����Ϊͼ���ϵĵ�����
	//double FΪ��ͷ����
	//���Ϊ���ڽ���=Fʱ���������ϵ����
	Point3f ImageFrame2CameraFrame(Point2f p, double F);


	/********************���о�̬����*********************/
	//��������������ת������ϵ
	static Point3f RotateByVector(double old_x, double old_y, double old_z, double vx, double vy, double vz, double theta)
	{
		double r = theta * CV_PI / 180;
		double c = cos(r);
		double s = sin(r);
		double new_x = (vx*vx*(1 - c) + c) * old_x + (vx*vy*(1 - c) - vz*s) * old_y + (vx*vz*(1 - c) + vy*s) * old_z;
		double new_y = (vy*vx*(1 - c) + vz*s) * old_x + (vy*vy*(1 - c) + c) * old_y + (vy*vz*(1 - c) - vx*s) * old_z;
		double new_z = (vx*vz*(1 - c) - vy*s) * old_x + (vy*vz*(1 - c) + vx*s) * old_y + (vz*vz*(1 - c) + c) * old_z;
		return Point3f(new_x, new_y, new_z);
	}

	//���ռ����Z����ת
	//������� x yΪ�ռ��ԭʼx y����
	//thetazΪ�ռ����Z����ת���ٶȣ��Ƕ��Ʒ�Χ��-180��180
	//outx outyΪ��ת��Ľ������
	static void CodeRotateByZ(double x, double y, double thetaz, double& outx, double& outy)
	{
		double x1 = x;//����������һ�Σ���֤&x == &outx���������Ҳ�ܼ�����ȷ
		double y1 = y;
		double rz = thetaz * CV_PI / 180;
		outx = cos(rz) * x1 - sin(rz) * y1;
		outy = sin(rz) * x1 + cos(rz) * y1;
	}

	//���ռ����Y����ת
	//������� x zΪ�ռ��ԭʼx z����
	//thetayΪ�ռ����Y����ת���ٶȣ��Ƕ��Ʒ�Χ��-180��180
	//outx outzΪ��ת��Ľ������
	static void CodeRotateByY(double x, double z, double thetay, double& outx, double& outz)
	{
		double x1 = x;
		double z1 = z;
		double ry = thetay * CV_PI / 180;
		outx = cos(ry) * x1 + sin(ry) * z1;
		outz = cos(ry) * z1 - sin(ry) * x1;
	}

	//���ռ����X����ת
	//������� y zΪ�ռ��ԭʼy z����
	//thetaxΪ�ռ����X����ת���ٶȣ��Ƕ��ƣ���Χ��-180��180
	//outy outzΪ��ת��Ľ������
	static void CodeRotateByX(double y, double z, double thetax, double& outy, double& outz)
	{
		double y1 = y;//����������һ�Σ���֤&y == &y���������Ҳ�ܼ�����ȷ
		double z1 = z;
		double rx = thetax * CV_PI / 180;
		outy = cos(rx) * y1 - sin(rx) * z1;
		outz = cos(rx) * z1 + sin(rx) * y1;
	}

	void read_xml(string);
	void read_xml(string, bool);
	Point3f get_distance(RotatedRect r, int armor_size);
private:

	Mat camera_matrix;//�ڲ�������
	Mat distortion_coefficients;//����ϵ��

	Mat rvec;//���������ת����
	Mat tvec;//�������ƽ������

	//��PNP���⣬���λ����Ϣ
	//���ú���RoteM, TransM, W2CTheta����������ȡ������������˵���μ�ע��
	//���������CV_ITERATIVE��CV_P3P��Ĭ�ϣ���CV_EPNP������μ�Opencv documentation.
	//ʵ��
	//CV_ITERATIVE�������ƺ�ֻ����4��������������⣬5�����ǹ���4��ⲻ����ȷ�Ľ�
	//CV_P3P��Gao�ķ�������ʹ�������ĸ������㣬������������������4Ҳ���ܶ���4
	//CV_EPNP��������ʵ����������>=4��������⣬����Ҫ4�㹲��
	//����ֵ��
	//0��ȷ
	//-1����ڲ�����������δ����
	//-2δ�ṩ�㹻�������㣬����������Ŀ��ƥ��
	//-3����ĵ������������printf��Ϣ
	int Solve(int method = SOLVEPNP_P3P);//    CV_ITERATIVE  CV_P3P   CV_EPNP
};

