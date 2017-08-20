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

Mat frame,image,HSVimage,Temp1,Temp2,gray,gray_pre;
Mat hann, prev64f,curr64f;
Mat mask,ROI;
Mat Beacon_NL;



extern Data_t uart_data_struct;

Point beacon_L,beacon_pre=0;
Point car;
Point car_last=0;

Point beacon_NL[10];

bool Tracker_NeedtoInit = 1;


SimpleBlobDetector::Params params;


void fast_dilate(Mat &a)
{
	Mat b;
	const int half_size=7;

	b.create(480,640, CV_8UC1);
	int width_counter, height_counter;
	int width, height;
	b = 0;
	for (width_counter = 1;width_counter < 639;width_counter++)
		for (height_counter = 1;height_counter < 479;height_counter++)
		{
			if(a.at<uchar>(height_counter, width_counter))
			{
				if (0==a.at<uchar>(height_counter, width_counter+1))//右边是黑色
				{
					for (width = width_counter;width <= width_counter+ half_size;width++)
						for (height = height_counter - half_size;height <= height_counter+half_size;height++)
						{
							if (width >= 0 && width < 640 && height >= 0 && height < 480)
							b.at<uchar>(height, width) = 255;
						}
				}
				//else if (0 == a.at<uchar>(height_counter, width_counter - 1))//左边是黑色
				//{
				//	for (width = width_counter-half_size;width <= width_counter;width++)
				//		for (height = height_counter-half_size;height <= height_counter + half_size;height++)
				//		{
				//			if (width >= 0 && width < 640 && height >= 0 && height < 480)
				//				b.at<uchar>(height, width) = 255;
				//		}
				//}
				//else if (0 == a.at<uchar>(height_counter - 1, width_counter))//上面是黑色
				//{
				//	for (width = width_counter - half_size;width <= width_counter + half_size;width++)
				//		for (height = height_counter - half_size;height <= height_counter;height++)
				//		{
				//			if (width >= 0 && width < 640 && height >= 0 && height < 480)
				//				b.at<uchar>(height, width) = 255;
				//		}
				//}
				else if (0 == a.at<uchar>(height_counter + 1, width_counter))//下面是黑色
				{
					for (width = width_counter - half_size;width <= width_counter + half_size;width++)
						for (height = height_counter;height <= height_counter+half_size;height++)
						{
							if (width >= 0 && width < 640 && height >= 0 && height < 480)
								b.at<uchar>(height, width) = 255;
						}
				}
			}
		}
	addWeighted(a, 1, b, 1, 0, a);

}

void FastDilateByContours(Mat &a, vector<vector<Point>> contours, int contour_num)
{
	int size = contours[contour_num].size();
	int half_size = 1;//dilate核半边长
	int i;
	int width, height;
	for (i = 0;i < size;i++)
	{
		for (width = contours[contour_num][i].x - half_size;width <= contours[contour_num][i].x + half_size;width++)
		for (height = contours[contour_num][i].y - half_size;height <= contours[contour_num][i].y + half_size;height++)
		{
			if (width >= 0 && width < 640 && height >= 0 && height < 480)//防止越界
				a.at<uchar>(height, width) = 255;
		}
	
	}
				
}


//TODO:移除小面积轮廓滤波
void RemoveSmallRegion(Mat& Src, Mat& Dst, int AreaLimit, int CheckMode, int NeihborMode)
{
	int RemoveCount = 0;       //记录除去的个数  
							   //记录每个像素点检验状态的标签，0代表未检查，1代表正在检查,2代表检查不合格（需要反转颜色），3代表检查合格或不需检查  
	Mat Pointlabel = Mat::zeros(Src.size(), CV_8UC1);

	if (CheckMode == 1)
	{
		//cout << "Mode: 去除小区域. ";
		for (int i = 0; i < Src.rows; ++i)
		{
			uchar* iData = Src.ptr<uchar>(i);
			uchar* iLabel = Pointlabel.ptr<uchar>(i);
			for (int j = 0; j < Src.cols; ++j)
			{
				if (iData[j] < 10)
				{
					iLabel[j] = 3;
				}
			}
		}
	}
	else
	{
		//cout << "Mode: 去除孔洞. ";
		for (int i = 0; i < Src.rows; ++i)
		{
			uchar* iData = Src.ptr<uchar>(i);
			uchar* iLabel = Pointlabel.ptr<uchar>(i);
			for (int j = 0; j < Src.cols; ++j)
			{
				if (iData[j] > 10)
				{
					iLabel[j] = 3;
				}
			}
		}
	}

	vector<Point2i> NeihborPos;  //记录邻域点位置  
	NeihborPos.push_back(Point2i(-1, 0));
	NeihborPos.push_back(Point2i(1, 0));
	NeihborPos.push_back(Point2i(0, -1));
	NeihborPos.push_back(Point2i(0, 1));
	if (NeihborMode == 1)
	{
		//cout << "Neighbor mode: 8邻域." << endl;
		NeihborPos.push_back(Point2i(-1, -1));
		NeihborPos.push_back(Point2i(-1, 1));
		NeihborPos.push_back(Point2i(1, -1));
		NeihborPos.push_back(Point2i(1, 1));
	}
	//else cout << "Neighbor mode: 4邻域." << endl;
	int NeihborCount = 4 + 4 * NeihborMode;
	int CurrX = 0, CurrY = 0;
	//开始检测  
	for (int i = 0; i < Src.rows; ++i)
	{
		uchar* iLabel = Pointlabel.ptr<uchar>(i);
		for (int j = 0; j < Src.cols; ++j)
		{
			if (iLabel[j] == 0)
			{
				//********开始该点处的检查**********  
				vector<Point2i> GrowBuffer;                                      //堆栈，用于存储生长点  
				GrowBuffer.push_back(Point2i(j, i));
				Pointlabel.at

					<uchar>(i, j) = 1;
				int CheckResult = 0;                                               //用于判断结果（是否超出大小），0为未超出，1为超出  

				for (int z = 0; z<GrowBuffer.size(); z++)
				{

					for (int q = 0; q<NeihborCount; q++)                                      //检查四个邻域点  
					{
						CurrX = GrowBuffer.at

						(z).x + NeihborPos.at

						(q).x;
						CurrY = GrowBuffer.at

						(z).y + NeihborPos.at

						(q).y;
						if (CurrX >= 0 && CurrX<Src.cols&&CurrY >= 0 && CurrY<Src.rows)  //防止越界  
						{
							if (Pointlabel.at

								<uchar>(CurrY, CurrX) == 0)
							{
								GrowBuffer.push_back(Point2i(CurrX, CurrY));  //邻域点加入buffer  
								Pointlabel.at

									<uchar>(CurrY, CurrX) = 1;           //更新邻域点的检查标签，避免重复检查  
							}
						}
					}

				}
				if (GrowBuffer.size()>AreaLimit) CheckResult = 2;                 //判断结果（是否超出限定的大小），1为未超出，2为超出  
				else { CheckResult = 1;   RemoveCount++; }
				for (int z = 0; z<GrowBuffer.size(); z++)                         //更新Label记录  
				{
					CurrX = GrowBuffer.at

					(z).x;
					CurrY = GrowBuffer.at

					(z).y;
					Pointlabel.at

						<uchar>(CurrY, CurrX) += CheckResult;
				}
				//********结束该点处的检查**********  


			}
		}
	}

	CheckMode = 255 * (1 - CheckMode);
	//开始反转面积过小的区域  
	for (int i = 0; i < Src.rows; ++i)
	{
		uchar* iData = Src.ptr<uchar>(i);
		uchar* iDstData = Dst.ptr<uchar>(i);
		uchar* iLabel = Pointlabel.ptr<uchar>(i);
		for (int j = 0; j < Src.cols; ++j)
		{
			if (iLabel[j] == 2)
			{
				iDstData[j] = CheckMode;
			}
			else if (iLabel[j] == 3)
			{
				iDstData[j] = iData[j];
			}
		}
	}

	//cout << RemoveCount << " objects removed." << endl;
}




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


//tracker初始化
void mouseClickCallback(int event, int x, int y, int flags, void* userdata)
{
	// 矩形数据返回  
	cv::Rect2d * pRect =
		reinterpret_cast<cv::Rect2d*>(userdata);
	// 鼠标按下操作  
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		std::cout << "LBUTTONDOWN ("
			<< x << ", " << y << ")" << std::endl;
		// 获取x，y坐标  
		pRect->x = x;
		pRect->y = y;
	}
	// 鼠标抬起操作  
	else if (event == cv::EVENT_LBUTTONUP)
	{
		std::cout << "LBUTTONUP ("
			<< x << ", " << y << ")" << std::endl;
		// 获取矩形宽高  
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

//计算点到线段的距离，注意是线段，不是直线
double D_point_to_line_by_points(Point p, Point line_start, Point line_stop)
{
	double k, b, A, B, C;
	double D_point_min;
	double D_line;

	D_point_min = D_point_to_point(p, line_start);
	k = D_point_to_point(p, line_stop);
	if (D_point_min > k) D_point_min = k;//到线段端点的最短距离

	k = (double)(line_stop.y - line_start.y) / (double)(line_stop.x - line_start.x);
	b = line_start.y - k*line_start.x;

	//-kx+y-b=0
	A = -k;
	B = 1;
	C = -b;

	D_line = abs(A*p.x + B*p.y + C) / sqrt(A*A + B*B);

	return D_line < D_point_min ? D_line : D_point_min;
}

Point dst_calculate(Point car_xy, Point beacon_xy)//计算不碰撞到信标的目标点坐标
{
	Point point_xy;
	double D_points;//车到圆心坐标的距离
	double R = 10;//半径10像素
	double D_length;//切线长
	double angle;//直线倾角

	D_points = D_point_to_point(car_xy, beacon_xy);
	D_length = sqrt(D_points*D_points - R*R);

	angle = atan2(beacon_xy.y - car_xy.y, beacon_xy.x - car_xy.x) + asin(R / D_points);//加号可以改成减，这样车会从另一边走

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

	//时间计算用变量
	DWORD TIME_start, TIME_end, TIME_period;
	DWORD TIME_plane_speed_period=0;
	uart_data_struct.angle = 0;
	uart_data_struct.distance = 0;


	////Tracker初始化
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

	/*相机参数矩阵，用于矫正畸变*/
	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
	cameraMatrix.at<double>(0, 0) = 363.690170215082;
	cameraMatrix.at<double>(0, 1) = 0.00534271281921327;
	cameraMatrix.at<double>(0, 2) = 332.718695030701;
	cameraMatrix.at<double>(1, 1) = 363.212502587468;
	cameraMatrix.at<double>(1, 2) = 224.407999768117;

	Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
	distCoeffs.at<double>(0, 0) = -0.324102094042723;
	distCoeffs.at<double>(1, 0) = 0.103889196470942;
	distCoeffs.at<double>(2, 0) = -1.36313326740246e-06;
	distCoeffs.at<double>(3, 0) = 0.000155397405846074;
	distCoeffs.at<double>(4, 0) = 0;

	Mat view, rview, map1, map2;
	Size imageSize;
	imageSize = frame.size();
	initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
		getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
		imageSize, CV_16SC2, map1, map2);


	
	////奇异点检测初始化
	//	params.filterByArea = true;
	//	params.minArea = 31;
	//	params.maxArea = 150;
	//	params.filterByColor = true;
	//	params.blobColor = 0;
	//	params.filterByCircularity = true;
	//	params.maxCircularity = 50;
	//	params.minCircularity = 0;
		setMouseCallback("ROI", mouseClickCallback, 0);
		//cv::waitKey(0);
		//tracker->init(frame, *rect);
		TIME_start = GetTickCount();
		cv::waitKey(20);
		TIME_end = GetTickCount();

	
	while (true)
	{
		

		
		mask.create(480, 640, CV_8UC1);
		mask = 0;
		ROI = 0;
		cam >> frame;
		
		TIME_end = GetTickCount();
		TIME_period = TIME_end - TIME_start;
		TIME_start = TIME_end;
		
		
		frame.copyTo(image);

		//相机矫正重映射
		remap(image, image, map1, map2, INTER_LINEAR);



		cvtColor(image, HSVimage, COLOR_BGR2HSV);
		cvtColor(image, gray, COLOR_BGR2GRAY);
		//inRange(Temp1, Scalar(101, 193, 139), Scalar(187, 255, 203), Temp1);//赛道蓝色背景范围；
		Mat Temp2;
		inRange(HSVimage, Scalar(98, 109, 126), Scalar(180, 255, 213), Temp1);//赛道蓝色背景范围；
		inRange(HSVimage, Scalar(1, 0, 0), Scalar(53, 255, 255), Temp2);//黄色胶带（大概范围）；
		addWeighted(Temp1, 1, Temp2, 1, 0, Temp1);


		//imshow("第一次HSV运算", Temp1);

		//imshow("image", image);

		//形态学运算的kernel，为运算速度，kernel必须相对较小
		//Mat kern = getStructuringElement(MORPH_ELLIPSE, Size(1, 1));
		//erode(Temp1, Temp1, kern, Point(-1, -1), 1, 0);
		//形态学运算，腐蚀，增加黄线线粗，以及连续性，从而很好地判断黄线
		//morphologyEx(Temp1, Temp1, MORPH_DILATE, kern,Point(-1,-1),1,0);
		//int contours_num;
		vector<vector<Point>> contours;
		vector<Vec4i>hierarchy;



		int big_contours[2] = { 0,0 };//[0]存放最大轮廓的轮廓号 [1]用来存放第二大轮廓的轮廓号
		int big_contours_counter = 0;//用于存放大轮廓的个数，值只能是1或者2
		int i = 0;
		int num;
		double maxarea;
		//double area_temp;

		findContours(Temp1, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
		num = contours.size();
		if (num == 0)
		{//如果没找到轮廓直接下一帧
			continue;
		}

			Scalar color(255, 255, 255);
			for (i = 0;i < num;i++)
			{
				if (contourArea(contours[i]) > 700)
				{
					drawContours(mask, contours, i, color, CV_FILLED, 8, hierarchy);
					FastDilateByContours(mask, contours, i);
				}
			}
			//RemoveSmallRegion(mask, mask, 80, 0, 1);
			imshow("mask", mask);
			HSVimage.copyTo(ROI, mask);
			Mat Beacon_L;
			Mat Car;

#pragma omp parallel sections
		{
#pragma omp  section
			{
			inRange(ROI, Scalar(0, 0, 200), Scalar(0, 0, 255), Beacon_L);	//亮着的信标
			//初始化Track
			//tracker->init(frame, *rect);
			//if (tracker->update(image, *rect))
			//	// 绘制追踪结果  
			//	cv::rectangle(image, *rect, cv::Scalar(0, 255, 0), 2, 1);

			double distance_calced = sqrt(pow((beacon_L.x - beacon_pre.x), 2)+pow((beacon_L.y - beacon_pre.y), 2));
			Moments mo;
			mo = moments(Beacon_L, true);
			
			
			TIME_plane_speed_period+=TIME_period;//加上一个周期的时间
			if (mo.m00 != 0.0000)//如果找到亮的信标
			{
				//Point beacon_L;
				beacon_L.x = mo.m10 / mo.m00;
				beacon_L.y = mo.m01 / mo.m00;

				//cout << distance_calced << endl;
				if (distance_calced<150)
				{
					uart_data_struct.x_speed = (float)(beacon_L.x-beacon_pre.x)/(float)TIME_plane_speed_period*1000.0f;//单位是像素每秒
					uart_data_struct.y_speed = (float)(beacon_L.y - beacon_pre.y)/(float)TIME_plane_speed_period*1000.0f;
					//cout << "y" << endl;
				}
				else
				{
					uart_data_struct.x_speed = 0;
					uart_data_struct.y_speed = 0;
				}
				TIME_plane_speed_period=0;//清除信标灯计时

				beacon_pre = beacon_L;

				//cout << "Beacon_L X:" << beacon_L.x << endl;
				//cout << "Beacon_L Y:" << beacon_L.y << endl;
			}
			//contours.clear();
			
			}
#pragma omp  section
			{

				vector<vector<Point>> contours_T;
				vector<Vec4i>hierarchy_T;
				Mat car_t;
				inRange(ROI, Scalar(90, 1, 0), Scalar(138, 255, 85), Car);
				////形态学运算的kernel，为运算速度，kernel必须相对较小
				Mat kern = getStructuringElement(MORPH_RECT, Size(2, 1));
				////erode(Temp1, Temp1, kern, Point(-1, -1), 1, 0);
				////形态学运算，腐蚀


				//形态学开运算，去除细小物体干扰
				morphologyEx(Car, Car, MORPH_OPEN, kern,Point(-1,-1),1,0);
				
				findContours(Car, contours_T, hierarchy_T, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));//找到赛道内的所有物体

			maxarea = 0;

			size_t contour_counter=0;
			size_t car_contour_num=0;

			contour_counter = 0;
			if (contours_T.size())
			{
				for (i = 0; i < contours_T.size() && i < 50; i++)//提出所有物体的坐标
				{
					//contour_center_xy[contour_counter++] = center_cal(contours, i);

					if (contourArea(contours_T[i]) > maxarea)//面积最大的是车
					{
						maxarea = contourArea(contours_T[i]);
						//car_xy = contour_center_xy[contour_counter - 1];
						car_contour_num = i;

					}
				}
				RotatedRect car_RECT=minAreaRect(contours_T[car_contour_num]);
				car_RECT.angle;
				imshow("car", Car);
				Moments mu;
				mu = moments(contours_T[car_contour_num]);
				//计算轮廓的质心     
				if (mu.m00 != 0.0000)
				{
					car = Point(mu.m10 / mu.m00, mu.m01 / mu.m00);
					//cout << "car_X" << car.x << endl;
					//cout << "car_Y" << car.y << endl;
					circle(image, car, 10, Scalar(0, 0, 255));
				}
				//waitKey(10);
				waitKey(1);
			}
			
			}
		
			//imshow("result", mask);
			//imshow("ROI", ROI);
			//imshow("Beacon",Beacon_L);




			
#pragma omp  section	//此线程用于计算不亮的信标及其位置
		{
				inRange(ROI, Scalar(66, 79, 178), Scalar(95, 105, 255), Beacon_NL);

				////形态学运算的kernel，为运算速度，kernel必须相对较小
				Mat kern = getStructuringElement(MORPH_RECT, Size(2, 2));
				morphologyEx(Beacon_NL, Beacon_NL, MORPH_OPEN, kern, Point(-1, -1), 1, 0);
				Moments mu;
				vector<vector<Point>> contours_T;
				vector<Vec4i>hierarchy_T;

				Point b_T[10];
				double  absab = 0,sqrta=0,sqrtb=0, angle = 0,distance_t=0,distance_min=640,distance_To_BL=0;
				int j = 0;

				imshow("beaconNL", Beacon_NL);
				findContours(Beacon_NL, contours_T, hierarchy_T, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));//找到赛道内的所有物体
				if (contours_T.size())
				{
					for (i = 0; i < contours_T.size()&&i<10; i++)//提出所有信标的坐标
					{
						mu = moments(contours_T[i]);
						b_T[i] = Point(mu.m10 / mu.m00, mu.m01 / mu.m00);
						circle(image, b_T[i], 8, Scalar(128, 255, 64));


						//计算不亮的信标是否在前进方向前方
						absab = (b_T[i].x - car.x)*(car.x - car_last.x) + (b_T[i].y - car.y)*(car.y - car_last.y);
						sqrta = sqrt((b_T[i].x - car.x)*(b_T[i].x - car.x)+ (b_T[i].y - car.y)*(b_T[i].y - car.y));
						sqrtb = sqrt((car.x - car_last.x)*(car.x - car_last.x) + (car.y - car_last.y)*(car.y - car_last.y));
						angle = acos(absab / sqrta / sqrtb) * 180 / 3.1415926;



						//计算车子离不亮的信标的距离
						distance_t= sqrt((b_T[i].x - car.x)*(b_T[i].x - car.x) + (b_T[i].y - car.y)*(b_T[i].y - car.y));
						//计算该信标离亮信标的距离，防止不亮的信标与亮的信标重合
						distance_To_BL=sqrt((b_T[i].x - beacon_L.x)*(b_T[i].x - beacon_L.x) + (b_T[i].y - beacon_L.y)*(b_T[i].y - beacon_L.y));
						//if (angle<60&& distance_t>30&& distance_t<100)
						if (distance_To_BL<10)
						{
							continue;
						}

						if (distance_min>distance_t)
						{
							distance_min = distance_t;
							j = i;
							
						}
							

					}
					beacon_NL[0] = b_T[j]; 
					circle(image, beacon_NL[0], 8, Scalar(128, 255, 64),-1);
					uart_data_struct.beacon_num = 1;
					//for (size_t i = 0; i < j&&i<2; i++)
					{
						uart_data_struct.beacon[0].x = beacon_NL[0].x;
						uart_data_struct.beacon[0].y = beacon_NL[0].y;
					}
					
				}
				waitKey(1);
		}




		}
//#pragma omp barrier
		Point2f car_speed_p;
		double car_angle;

		car_speed_p.x = (car.x - car_last.x) / (double)TIME_period;
		car_speed_p.y = (car.y - car_last.y) / (double)TIME_period;
		car_angle = atan2(car_speed_p.y, car_speed_p.x);
		car_last = car;



		uart_data_struct.dest_x = beacon_L.x;
		uart_data_struct.dest_y = beacon_L.y;
		uart_data_struct.car_x = car.x;
		uart_data_struct.car_y = car.y;
		//cout <<"XS   "<< uart_data_struct.x_speed << endl;
		//cout <<"YS   "<< uart_data_struct.y_speed << endl;
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
	/*	cout << uart_data_struct.angle << endl;*/
		//cout << TIME_period << endl;

		imshow("image", image);

		waitKey(10);
	}

	waitKey(0);
}