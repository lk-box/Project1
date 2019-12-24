#include "PNPSolver.h"
#include "Drawline.h"

// �������ڿ��ٽ��PNP���⣬˳������ռ�������ת�Լ�ͼ��ϵ�����ϵ������ϵ��ϵ����ͶӰ����
// ����˳��
// 1.��ʼ������
// 2.����SetCameraMatrix(),SetDistortionCoefficients()���ú�����ڲ����뾵ͷ�������
// 3.��Points3D��Points2D�����һһ��Ӧ���������
// 4.����Solve()�������м���
// 5.��RoteM, TransM, W2CTheta��������������
//
// ԭ��μ���http://www.cnblogs.com/singlex/category/911880.html
// Author��VShawn
// Ver:2016.11.26.0
void convert1(vector <Point> point2i, vector <Point2f>& point2f);
void convert2(vector <Point2f> point2i, vector <Point>& point2f);
PNPSolver::PNPSolver()
{
	//��ʼ���������
	vector<double> rv(3), tv(3);
	Mat rvec(rv), tvec(tv);
}
PNPSolver::PNPSolver(Mat CameraMatrix, Mat DistortionCoefficients)
{
	//��ʼ���������
	vector<double> rv(3), tv(3);
	Mat rvec(rv), tvec(tv);
	camera_matrix = CameraMatrix;
	distortion_coefficients = DistortionCoefficients;
}

PNPSolver::~PNPSolver()
{
}

int PNPSolver::Solve(int method)
{
	//����У��
	if (camera_matrix.cols == 0 || distortion_coefficients.cols == 0)
	{
		printf("ErrCode:-1,����ڲ�����������δ���ã�\r\n");
		return -1;
	}

	if (Points3D.size() != Points2D.size())
	{
		cout << Points3D.size() << "," << Points2D.size() << endl;
		printf("ErrCode:-2,1111�\r\n");
		return -2;
	}
	if (method == SOLVEPNP_P3P || method == SOLVEPNP_ITERATIVE)
	{
		if (Points3D.size() != 4)
		{
			printf("ErrCode:-2,2222�\r\n");
			return -2;
		}
	}
	else
	{
		if (Points3D.size() < 4)
		{
			printf("ErrCode:-2,3333�\r\n");
			return -2;
		}
	}

	////TODO::�����Ƿ��ǹ�����ĵ�
	//if ((method == METHOD::CV_ITERATIVE || method == METHOD::CV_EPNP) && Points2D.size() == 4)
	//{
	//	//ͨ������������˻�÷����������������Ƿ�ƽ��
	//}






	/*******************���PNP����*********************/
	//�����ַ������
	solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, true, method);
	//ʵ��������ƺ�ֻ���ù�����������λ��
	//solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, CV_ITERATIVE);	//ʵ��������ƺ�ֻ���ù�����������λ��
	//solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, CV_P3P);		//Gao�ķ�������ʹ�������ĸ�������
	//solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, CV_EPNP);


	/*******************��ȡ��ת����*********************/
	Rodrigues(rvec, RoteM);  //�޼�,����ת���������ת����
	double r11 = RoteM.ptr<double>(0)[0];
	double r12 = RoteM.ptr<double>(0)[1];
	double r13 = RoteM.ptr<double>(0)[2];
	double r21 = RoteM.ptr<double>(1)[0];
	double r22 = RoteM.ptr<double>(1)[1];
	double r23 = RoteM.ptr<double>(1)[2];
	double r31 = RoteM.ptr<double>(2)[0];
	double r32 = RoteM.ptr<double>(2)[1];
	double r33 = RoteM.ptr<double>(2)[2];
	TransM = tvec;

	//������������ϵ��������תŷ���ǣ���ת�����ת����������ϵ��
	//��ת˳��Ϊz��y��x
	double thetaz = atan2(r21, r11) / CV_PI * 180;
	double thetay = atan2(-1 * r31, sqrt(r32*r32 + r33*r33)) / CV_PI * 180;
	double thetax = atan2(r32, r33) / CV_PI * 180;

	//cout << "x (yaw):" <<  thetax << endl;
	//cout << "y (pitch):" << thetay << endl;
	//cout << "z :" << thetaz << endl;
	//cout << endl;

	//���ϵ������ϵ��������תŷ���ǣ��������ϵ�մ���ת���������������ϵ��ȫƽ�С�
	//��ת˳��Ϊz��y��x
	Theta_C2W.z = thetaz;
	Theta_C2W.y = thetay;
	Theta_C2W.x = thetax;

	//���������ϵ�����ϵ��������תŷ���ǣ�����ϵ�մ���ת�����ת���������ϵ��
	//��ת˳��Ϊx��y��z
	Theta_W2C.x = -1 * thetax;
	Theta_W2C.y = -1 * thetay;
	Theta_W2C.z = -1 * thetaz;

	/*************************************�˴�������������ϵԭ��Oc����������ϵ�е�λ��**********************************************/

	/***********************************************************************************/
	/* ��ԭʼ����ϵ������תz��y��x������ת������������ϵƽ�У�����OcOw�������ת */
	/* ��������֪��������������ϵ��ȫƽ��ʱ��OcOw��ֵ */
	/* ��ˣ�ԭʼ����ϵÿ����ת��ɺ󣬶�����OcOw����һ�η�����ת�����տ��Եõ���������ϵ��ȫƽ��ʱ��OcOw */
	/* ����������-1������������ϵ����������� */
	/***********************************************************************************/

	//���ƽ�ƾ��󣬱�ʾ���������ϵԭ�㣬��������(x,y,z)�ߣ��͵�����������ϵԭ��
	double tx = tvec.ptr<double>(0)[0];
	double ty = tvec.ptr<double>(0)[1];
	double tz = tvec.ptr<double>(0)[2];

	//x y z ΪΨһ���������ԭʼ����ϵ�µ�����ֵ
	//Ҳ��������OcOw���������ϵ�µ�ֵ
	double x = tx, y = ty, z = tz;
	Position_OwInC.x = x;
	Position_OwInC.y = y;
	Position_OwInC.z = z;
	//�������η�����ת
	CodeRotateByZ(x, y, -1 * thetaz, x, y);
	CodeRotateByY(x, z, -1 * thetay, x, z);
	CodeRotateByX(y, z, -1 * thetax, y, z);


	//����������������ϵ�µ�λ������
	//������OcOw����������ϵ�µ�ֵ
	Position_OcInW.x = x*-1;
	Position_OcInW.y = y*-1;
	Position_OcInW.z = z*-1;

	return 0;
}


//���ݼ�����Ľ��������������ͶӰ��ͼ�񣬷�����������㼯
//����Ϊ��������ϵ�ĵ����꼯��
//���Ϊ��ͶӰ��ͼ���ϵ�ͼ�����꼯��
vector<Point2f> PNPSolver::WordFrame2ImageFrame(vector<Point3f> WorldPoints)
{
	vector<Point2f> projectedPoints;
	projectPoints(WorldPoints, rvec, tvec, camera_matrix, distortion_coefficients, projectedPoints);
	return projectedPoints;
}

//��������Ĳ�����ͼ������ת�������������
//ʹ��ǰ��Ҫ����Solve()������λ��
//����Ϊͼ���ϵĵ�����
//double FΪ��ͷ����
//���Ϊ���ڽ���=Fʱ���������ϵ����
Point3f PNPSolver::ImageFrame2CameraFrame(Point2f p, double F)
{
	double fx;
	double fy;
	double u0;
	double v0;

	fx = camera_matrix.ptr<double>(0)[0];
	u0 = camera_matrix.ptr<double>(0)[2];
	fy = camera_matrix.ptr<double>(1)[1];
	v0 = camera_matrix.ptr<double>(1)[2];
	double zc = F;
	double xc = (p.x - u0)*F / fx;
	double yc = (p.y - v0)*F / fy;
	return Point3f(xc, yc, zc);
}

void PNPSolver::read_xml(string xml, bool left_eye)
{
	const char *IntrinsicsPath = xml.c_str();
	FileStorage readfs;

	if (!readfs.open(IntrinsicsPath, FileStorage::READ))
	{
		cout << "Cannot open the file" << endl;
		exit(0);
	}
	if (left_eye)
	{
		readfs["cameraMatrixL"] >> camera_matrix;
		readfs["distCoeffL"] >> distortion_coefficients;
	}
	else
	{
		readfs["cameraMatrixR"] >> camera_matrix;
		readfs["distCoeffR"] >> distortion_coefficients;
	}

	//  distortion_coefficients
	//cout << camera_matrix << endl << distortion_coefficients << endl;
	readfs.release();
}

void PNPSolver::read_xml(string xml)
{
	const char *IntrinsicsPath = xml.c_str();
	FileStorage readfs;

	if (!readfs.open(IntrinsicsPath, FileStorage::READ))
	{
		cout << "Cannot open the file" << endl;
		exit(0);
	}
	readfs["cameraMatrix"] >> camera_matrix;
	readfs["distCoeff"] >> distortion_coefficients;
	//  distortion_coefficients
	//cout << camera_matrix << endl << distortion_coefficients << endl;
	readfs.release();
}


Point3f PNPSolver::get_distance(RotatedRect r, int armor_size)//vector <Point2f> points   RotatedRect r
{
	Point3f point;
	vector<Point> point2i;
	Point2f pt[4];
	for (int i = 0; i < 4; i++)
	{
		pt[i].x = 0;
		pt[i].y = 0;
	}
	r.points(pt);

	vector <Point2f> points;

	points.push_back(Point2f(pt[3]));   //1 ����  20+    ����  150+
	points.push_back(Point2f(pt[0]));  //2 ����
	points.push_back(Point2f(pt[1]));  //3 ����
	points.push_back(Point2f(pt[2]));  //0 ����
	//points.push_back(Point2f(r.center));  //0 ����

	//points.push_back(Point2f(r.center.y - r.size.height / 2, r.center.x - r.size.width / 2));   //1 ����  20+    ����  150+
	//points.push_back(Point2f(r.center.y + r.size.height / 2, r.center.x - r.size.width / 2));  //2 ����
	//points.push_back(Point2f(r.center.y + r.size.height / 2, r.center.x + r.size.width / 2));  //3 ����
	//points.push_back(Point2f(r.center.y - r.size.height / 2, r.center.x + r.size.width / 2));  //0 ����


	Points2D = points;
	convert2(points, point2i);
	convert1(point2i, Points2D);

	if (armor_size == 0)
	{
		Points3D.push_back(Point3f(-27.5, -65, 0));//27.5   
		Points3D.push_back(Point3f(-27.5, 65, 0));//65   
		Points3D.push_back(Point3f(27.5, 65, 0));//55
		Points3D.push_back(Point3f(27.5, -65, 0));//130
	}
	else
	{
		Points3D.push_back(Point3f(-27.5, -115, 0));//27.5   
		Points3D.push_back(Point3f(-27.5, 115, 0));//65   
		Points3D.push_back(Point3f(27.5, 115, 0));//55      55
		Points3D.push_back(Point3f(27.5, -115, 0));//130   230
	}
	
	Solve(SOLVEPNP_EPNP);//CV_P3P  CV_EPNP  CV_ITERATIVE  DLS  UPNP
	Points3D.clear();

	float Py = fabs(pt[0].y - pt[1].y);
	float Wy = 55.0;
	float Fy = 567.3833;
	float Dy = (Wy * Fy) / Py;

	//cout << "y = " << Dy << endl;
	//cout << "pnp:" << Position_OcInW.z << endl;
	Position_OcInW.x = Dy;
	//cout << "pnp:" << Position_OcInW.z << endl;
//	Position_OcInW.z = (Position_OcInW.z - (((int)Position_OcInW.z / 100)* 30));
	Position_OcInW.z /= 10;

	return Position_OcInW;
}

void convert1(vector <Point> point2i, vector <Point2f>& point2f)
{
	Mat src = Mat(point2i);
	Mat dst = Mat(point2f);
	Mat tmp;
	src.convertTo(tmp, dst.type());
	point2f = (vector<Point2f>)tmp;
}

void convert2(vector <Point2f> point2i, vector <Point>& point2f)
{
	Mat src = Mat(point2i);
	Mat dst = Mat(point2f);
	Mat tmp;
	src.convertTo(tmp, dst.type());
	point2f = (vector<Point>)tmp;
}
