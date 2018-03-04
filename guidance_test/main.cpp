#include <iostream>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <fstream>
#include <string>
#include <sys/timeb.h>  
#include <time.h>  
#include <stdlib.h>  
#include <iostream>  
#include <Windows.h> 
#include "DJI_guidance.h"
#include "DJI_utility.h"    


using namespace cv;
using namespace std;

Mat image;
bool selectObject = false;
int trackObject = 0;
Point origin;
Rect selection;
DJI_event g_event;
DJI_lock g_lock;
#define WIDTH 320
#define HEIGHT 240
#define IMAGE_SIZE (HEIGHT * WIDTH)


Mat g_imleft(HEIGHT, WIDTH, CV_8U);
Mat g_imright(HEIGHT, WIDTH, CV_8U);
Mat	g_depth(HEIGHT, WIDTH, CV_16SC1);
e_vbus_index selected_vbus = e_vbus1;  // 选择前面的摄像头
string winDemoName = "Guidance Demo";
string depthName;
string leftName;
string rightName;

int my_callback(int data_type, int data_len, char *content)
{
	printf("enter callback..\n");
	g_lock.enter();
	if (e_image == data_type && NULL != content)
	{
		printf("callback: type is image..\n");
		image_data data;
		memcpy((char*)&data, content, sizeof(data));
		memcpy(g_imleft.data, data.m_greyscale_image_left[selected_vbus], IMAGE_SIZE);
		memcpy(g_imright.data, data.m_greyscale_image_right[selected_vbus], IMAGE_SIZE);
		memcpy(g_depth.data, data.m_depth_image[selected_vbus], IMAGE_SIZE * 2);
	}
	g_lock.leave();
	g_event.set_event();
	return 0;
}


#define RETURN_IF_ERR(err_code) { if( err_code ){ printf( "USB error code:%d in file %s %d\n", err_code, __FILE__, __LINE__ );}}
#define RELEASE_IF_ERR(err_code) { if( err_code ){ release_transfer(); printf( "USB error code:%d in file %s %d\n", err_code, __FILE__, __LINE__ );}}


string intToString(int i)
{
	stringstream ss;
	ss << i;
	return ss.str();
}

int main(int arc, char** argv)
{
	// Connect to Guidance and subscribe data
	reset_config();
	int err_code = init_transfer();
	RETURN_IF_ERR(err_code);

	err_code = select_greyscale_image(selected_vbus, true);
	RELEASE_IF_ERR(err_code);

	err_code = select_greyscale_image(selected_vbus, false);
	RELEASE_IF_ERR(err_code);

	err_code = select_depth_image(selected_vbus);
	RELEASE_IF_ERR(err_code);

	err_code = set_sdk_event_handler(my_callback);
	RELEASE_IF_ERR(err_code);

	err_code = start_transfer();
	RELEASE_IF_ERR(err_code);

	Mat depth(HEIGHT, WIDTH, CV_8UC1);

	while (1)
	{
		/* 等待数据到来 */
		g_event.wait_event();

		// convert 16 bit depth image to 8 bit
		g_depth.convertTo(depth, CV_8UC1);
		/* 显示图 */
		imshow("depth", depth);
		imshow("left", g_imleft);
		imshow("right", g_imright);

		struct timeb tb;
		ftime(&tb); /* 求得当前时间 */
		struct tm *t = ::localtime(&tb.time);

		string time = intToString(t->tm_mon + 1) + "-"+
			intToString(t->tm_mday) + "-" +
			intToString(t->tm_hour) + "-" +
			intToString(t->tm_min) + "-" +
			intToString(t->tm_sec) + "-" +
			intToString(tb.millitm) + "-";

		
		depthName = "data/" + time + "depth.jpg";
		leftName = "data/" + time + "left.jpg";
		rightName = "data/" + time + "right.jpg";

		/*cout << depthName << endl;*/

		//保存
		imwrite(depthName, depth);
		imwrite(leftName, g_imleft);
		imwrite(rightName, g_imright);


		char c = (char)waitKey(10);
		if (c == 27 || c == 'q')
		{
			break;
		}
	}

	/* 停止数据传输 */
	err_code = stop_transfer();
	RELEASE_IF_ERR(err_code);
	/* 确保停止 */
	sleep(100000);
	/* 释放传输线程 */
	err_code = release_transfer();
	RETURN_IF_ERR(err_code);
	return 0;
	return 0;
}