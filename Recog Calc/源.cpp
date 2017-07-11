#include <cv.hpp>
#include <iostream>
#include<Windows.h>
#include<thread>
#include<vector>
//#include <opencv2/tracking/tracker.hpp>  
#include <opencv2/core/utility.hpp>  
//#include <opencv2/tracking.hpp>  
#include <opencv2/videoio.hpp>  
#include	"SerialCOM.h"
#include "omp.h"


#pragma pack(1)



using namespace std;
using namespace cv;

//void thread_task()
//{
//
//}
//Global params

Mat frame,image,HSVimage,Temp1,Temp2;
Mat mask,ROI;

extern Data_t uart_data_struct;

Point beacon_L;
Point car;
Point car_last=0;

bool Tracker_NeedtoInit = 1;


SimpleBlobDetector::Params params;


void onChangeTrakBar(int H1, void *data)
{
	params.minArea = H1;


}
void onChangeTrakBar1(int S1, void *data)
{
	params.maxArea = S1;


}
void onChangeTrakBar2(int V1, void *data)
{
	params.minCircularity = V1;
}
void onChangeTrakBar3(int V1, void *data)
{
	params.maxCircularity = V1;
}
void onChangeTrakBar4(int V1, void *data)
{
	params.blobColor = V1;
}

void MouseEvent(int event, int x, int y, int flags, void* data)
{
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		unsigned char* p = ROI.ptr<uchar>(y) + x * 3;
		cout << "H:" << (int)p[0] << endl;
		cout << "S:" << (int)p[1] << endl;
		cout << "V:" << (int)p[2] << endl;
		cout << "mouse X:" << x;
		cout << "mouse Y" << y;
	}
}


//tracker��ʼ��
void mouseClickCallback(int event, int x, int y, int flags, void* userdata)
{
	// �������ݷ���  
	cv::Rect2d * pRect =
		reinterpret_cast<cv::Rect2d*>(userdata);
	// ��갴�²���  
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		std::cout << "LBUTTONDOWN ("
			<< x << ", " << y << ")" << std::endl;
		// ��ȡx��y����  
		pRect->x = x;
		pRect->y = y;
	}
	// ���̧�����  
	else if (event == cv::EVENT_LBUTTONUP)
	{
		std::cout << "LBUTTONUP ("
			<< x << ", " << y << ")" << std::endl;
		// ��ȡ���ο��  
		pRect->width = std::abs(x - pRect->x);
		pRect->height = std::abs(y - pRect->y);
	}
}

double D_point_to_point(Point a, Point b)
{
	double dx = a.x - b.x;
	double dy = a.y - b.y;

	return sqrt(dx*dx + dy*dy);
}

//����㵽�߶εľ��룬ע�����߶Σ�����ֱ��
double D_point_to_line_by_points(Point p, Point line_start, Point line_stop)
{
	double k, b, A, B, C;
	double D_point_min;
	double D_line;

	D_point_min = D_point_to_point(p, line_start);
	k = D_point_to_point(p, line_stop);
	if (D_point_min > k) D_point_min = k;//���߶ζ˵����̾���

	k = (double)(line_stop.y - line_start.y) / (double)(line_stop.x - line_start.x);
	b = line_start.y - k*line_start.x;

	//-kx+y-b=0
	A = -k;
	B = 1;
	C = -b;

	D_line = abs(A*p.x + B*p.y + C) / sqrt(A*A + B*B);

	return D_line < D_point_min ? D_line : D_point_min;
}

Point dst_calculate(Point car_xy, Point beacon_xy)//���㲻��ײ���ű��Ŀ�������
{
	Point point_xy;
	double D_points;//����Բ������ľ���
	double R = 10;//�뾶10����
	double D_length;//���߳�
	double angle;//ֱ�����

	D_points = D_point_to_point(car_xy, beacon_xy);
	D_length = sqrt(D_points*D_points - R*R);

	angle = atan2(beacon_xy.y - car_xy.y, beacon_xy.x - car_xy.x) + asin(R / D_points);//�Ӻſ��Ըĳɼ��������������һ����

	point_xy.x = car_xy.x + D_length*cos(angle);
	point_xy.y = car_xy.y + D_length*sin(angle);

	return point_xy;
}



int main()
{
	VideoCapture cam;
	cam.open(0, 0);
	cam >> frame;
	vector<KeyPoint> keypoints;
	for (size_t i = 0; i < 10; i++)
	{
		cam >> frame;
	}

	//ʱ������ñ���
	DWORD TIME_start, TIME_end, TIME_perioud;

	uart_data_struct.angle = 0;
	uart_data_struct.distance = 0;


	////Tracker��ʼ��
	//cv::Rect2d *rect(new cv::Rect2d);
	//cv::Ptr<cv::TrackerKCF> tracker = cv::TrackerKCF::createTracker();
	//imshow("image", frame);
	//namedWindow("blob", WINDOW_NORMAL);
	//createTrackbar("minArea", "blob", 0, 1000, onChangeTrakBar, 0);
	//createTrackbar("maxArea", "blob", 0, 1000, onChangeTrakBar1, 0);
	//createTrackbar("minCircularity", "blob", 0, 1000, onChangeTrakBar2, 0);
	//createTrackbar("maxCircularity", "blob", 0, 1000, onChangeTrakBar3, 0);
	//createTrackbar("blobColor", "blob", 0, 255, onChangeTrakBar4, 0);
	setMouseCallback("image", mouseClickCallback, 0);

	Serial_COM_init();




	//��������ʼ��
		params.filterByArea = true;
		params.minArea = 31;
		params.maxArea = 150;
		params.filterByColor = true;
		params.blobColor = 0;
		params.filterByCircularity = true;
		params.maxCircularity = 50;
		params.minCircularity = 0;
		setMouseCallback("ROI", mouseClickCallback, 0);
		//cv::waitKey(0);
		//tracker->init(frame, *rect);
		TIME_start = GetTickCount();
		cv::waitKey(20);
		TIME_end = GetTickCount();


	while (true)
	{
		
		TIME_end = GetTickCount();
		TIME_perioud = TIME_end - TIME_start;
		TIME_start = GetTickCount();

		//ocl::oclMat ocl_img(img), ocl_gray;


		mask.create(480, 640, CV_8UC1);
		mask = 0;
		ROI = 0;
		cam >> frame;
		frame.copyTo(image);
		cvtColor(image, HSVimage, COLOR_BGR2HSV);
		//inRange(Temp1, Scalar(101, 193, 139), Scalar(187, 255, 203), Temp1);//������ɫ������Χ��
		Mat Temp2;
		inRange(HSVimage, Scalar(98, 109, 126), Scalar(180, 255, 213), Temp1);//������ɫ������Χ��
		inRange(HSVimage, Scalar(1, 0, 0), Scalar(53, 255, 255), Temp2);//��ɫ��������ŷ�Χ����
		addWeighted(Temp1, 1, Temp2, 1, 0, Temp1);


		//imshow("��һ��HSV����", Temp1);

		//imshow("image", image);

		//��̬ѧ�����kernel��Ϊ�����ٶȣ�kernel������Խ�С
		//Mat kern = getStructuringElement(MORPH_ELLIPSE, Size(1, 1));
		//erode(Temp1, Temp1, kern, Point(-1, -1), 1, 0);
		//��̬ѧ���㣬��ʴ�����ӻ����ߴ֣��Լ������ԣ��Ӷ��ܺõ��жϻ���
		//morphologyEx(Temp1, Temp1, MORPH_ERODE, kern,Point(-1,-1),1,0);
		int contours_num;
		vector<vector<Point>> contours;
		vector<Vec4i>hierarchy;



		Mat temp;
		int i = 0;
		int max_contour = 0;
		int num;
		double maxarea;

		findContours(Temp1, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
		num = contours.size();
		if (num == 0)
		{
			continue;
		}
		else
		{
			maxarea = contourArea(contours[0]);
			for (i = 0; i < num; i++)
			{
				if (maxarea < contourArea(contours[i]))
				{
					max_contour = i;
					maxarea = contourArea(contours[i]);

				}
			}
		}


			Scalar color(255, 255, 255);
			drawContours(mask, contours, max_contour, color, CV_FILLED, 8, hierarchy);
			//RemoveSmallRegion(mask, mask, 80, 0, 1);
			imshow("mask", mask);
			HSVimage.copyTo(ROI, mask);
			Mat Beacon_L;
			Mat Car;

#pragma omp parallel sections
		{
#pragma omp  section
			{
			inRange(ROI, Scalar(0, 0, 200), Scalar(0, 0, 255), Beacon_L);	//���ŵ��ű�
			//��ʼ��Track
			//tracker->init(frame, *rect);
			//if (tracker->update(image, *rect))
			//	// ����׷�ٽ��  
			//	cv::rectangle(image, *rect, cv::Scalar(0, 255, 0), 2, 1);


			Moments mo;
			mo = moments(Beacon_L, true);
			if (mo.m00 != 0.0000)
			{
				//Point beacon_L;
				beacon_L.x = mo.m10 / mo.m00;
				beacon_L.y = mo.m01 / mo.m00;

				cout << "Beacon_L X:" << beacon_L.x << endl;
				cout << "Beacon_L Y:" << beacon_L.y << endl;
			}
			//contours.clear();
			
			}
#pragma omp  section
			{

				vector<vector<Point>> contours_T;
				vector<Vec4i>hierarchy_T;
			inRange(ROI, Scalar(95, 0, 0), Scalar(180, 255, 71), Car);
			findContours(Car, contours_T, hierarchy_T, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));//�ҵ������ڵ���������

			maxarea = 0;
			//Point contour_center_xy[50];
			size_t contour_counter=0;
			size_t car_contour_num=0;

			contour_counter = 0;
			if (contours_T.size())
			{
				for (i = 0; i < contours_T.size() && i < 50; i++)//����������������
				{
					//contour_center_xy[contour_counter++] = center_cal(contours, i);

					if (contourArea(contours_T[i]) > maxarea)//��������ǳ�
					{
						maxarea = contourArea(contours_T[i]);
						//car_xy = contour_center_xy[contour_counter - 1];
						car_contour_num = i;

					}
				}
				imshow("car", Car);
				Moments mu;
				mu = moments(contours_T[car_contour_num]);
				//��������������     
				if (mu.m00 != 0.0000)
				{
					car = Point(mu.m10 / mu.m00, mu.m01 / mu.m00);
					cout << "car_X" << car.x << endl;
					cout << "car_Y" << car.y << endl;
					circle(image, car, 10, Scalar(255, 128, 128));
				}
				waitKey(1);
			
			}
			
			}
			
			//imshow("result", mask);
			//imshow("ROI", ROI);
			//imshow("Beacon",Beacon_L);
//#pragma omp  section
//			{
//		//Ptr<SimpleBlobDetector> d = SimpleBlobDetector::create(params);
//
//		//d->detect(ROI, keypoints);
//
//		//drawKeypoints(image, keypoints, frame,Scalar(0,0,255));
//
//		//imshow("blob", frame);
//			}
		}
//#pragma omp barrier
		Point2f car_speed_p;
		double car_angle;

		car_speed_p.x = (car.x - car_last.x) / (double)TIME_perioud;
		car_speed_p.y = (car.y - car_last.y) / (double)TIME_perioud;
		car_angle = atan2(car_speed_p.y, car_speed_p.x);
		car_last = car;

		uart_data_struct.dest_x = beacon_L.x;
		uart_data_struct.dest_y = beacon_L.y;
		uart_data_struct.car_x = car.x;
		uart_data_struct.car_y = car.y;

		Point dst_current = dst_calculate(car, beacon_L);
		uart_data_struct.distance = D_point_to_point(car, dst_current);
		uart_data_struct.angle=  (atan2(dst_current.y - car.y, dst_current.x - car.x) - atan2(car_speed_p.y, car_speed_p.x)) / 3.1415926 * 180;

		if (isinf(uart_data_struct.angle))
			uart_data_struct.angle = 0;


		if (uart_data_struct.angle > 180)
		{
			uart_data_struct.angle = 360 - uart_data_struct.angle;
		}
		if (uart_data_struct.angle < -180)
		{
			uart_data_struct.angle = 360 + uart_data_struct.angle;
		}
		
		line(image, car, dst_current, Scalar(0, 0, 255));
		circle(image, beacon_L, 10, Scalar(0, 255, 255));
		wait_for_send_end();
		send_once();
		cout << uart_data_struct.angle << endl;
		cout << TIME_perioud << endl;
		imshow("image", image);

		waitKey(10);
	}

	waitKey(0);
}