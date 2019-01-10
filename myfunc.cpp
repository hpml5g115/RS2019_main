//M_PIを使うのに必要
#define _USE_MATH_DEFINES
#include <stdio.h>
#include <stdlib.h>
//#include <iostream>
//#include <fstream>
#include <cmath>
// #include <wiringPi.h>
// #include <softPwm.h>
// #include <signal.h>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "myfunc.h"
#include "class.h"


void GraphGain(double x, double y, int *xr, int *yr) {
	//収まるようにゲイン
	double xgain = x / dis_range*X_half;
	double ygain = y / dis_range*Y_half;

	//値補正
	*xr = (int)(xgain + X_half);
	*yr = (int)(Y_half - ygain);
}

void ExitProcess(RPlidarDriver * drv) {
	drv->stop();
	drv->stopMotor();

	RPlidarDriver::DisposeDriver(drv);
}

bool Initialize(RPlidarDriver * drv) {
	//以下、RPlidarの初期化
	const char * opt_com_path = "/dev/ttyUSB0";
	_u32         opt_com_baudrate = 115200;
	u_result     op_result;
	rplidar_response_device_health_t healthinfo;
	rplidar_response_device_info_t devinfo;
	// try to connect
	if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
		printf("Error, cannot bind to the specified serial port %s.\n"
			, opt_com_path);
		return false;
	}

	// retrieving the device info
	////////////////////////////////////////
	op_result = drv->getDeviceInfo(devinfo);

	if (IS_FAIL(op_result)) {
		if (op_result == RESULT_OPERATION_TIMEOUT) {
			// you can check the detailed failure reason
			printf("Error, operation time out.\n");
		}
		else {
			printf("Error, unexpected error, code: %x\n", op_result);
			// other unexpected result
		}
		return false;
	}

	// print out the device serial number, firmware and hardware version number..
	printf("RPLIDAR S/N: ");
	for (int pos = 0; pos < 16; ++pos) {
		printf("%02X", devinfo.serialnum[pos]);
	}

	printf("\n"
		"Version: " RPLIDAR_SDK_VERSION"\n"
		"Firmware Ver: %d.%02d\n"
		"Hardware Rev: %d\n"
		, devinfo.firmware_version >> 8
		, devinfo.firmware_version & 0xFF
		, (int)devinfo.hardware_version);


	// check the device health
	////////////////////////////////////////
	op_result = drv->getHealth(healthinfo);
	if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
		printf("RPLidar health status : ");
		switch (healthinfo.status) {
		case RPLIDAR_STATUS_OK:
			printf("OK.");
			break;
		case RPLIDAR_STATUS_WARNING:
			printf("Warning.");
			break;
		case RPLIDAR_STATUS_ERROR:
			printf("Error.");
			break;
		}
		printf(" (errorcode: %d)\n", healthinfo.error_code);

	}
	else {
		printf("Error, cannot retrieve the lidar health code: %x\n", op_result);
		return false;
	}


	if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
		printf("Error, rplidar internal error detected. Please reboot the device to retry.\n");
		// enable the following code if you want rplidar to be reboot by software
		// drv->reset();
		return false;
	}

	return true;
}

//測定用関数
bool FinderMeasure(RPlidarDriver * drv, measure * result) {
	u_result ans;
	rplidar_response_measurement_node_t nodes[360 * 2];
	size_t   count = _countof(nodes);

	//データ消去
	// result->erase();

	// fetech extactly one 0-360 degrees' scan
	ans = drv->grabScanData(nodes, count);
	if (IS_OK(ans) || ans == RESULT_OPERATION_TIMEOUT) {
		drv->ascendScanData(nodes, count);

		//データ取得
		for (int pos = 0; pos < count; ++pos) {
			rp_datas dt;
			dt.deg = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
			dt.distance = nodes[pos].distance_q2 / 4.0f;
			result->data.push_back(dt);
			// result->deg.push_back((nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f);
			// result->distance.push_back(nodes[pos].distance_q2 / 4.0f);
		}
		result->size_in();
		return true;
	}
	else {
		return false;
	}
}



void PictureGrid(cv::Mat img) {
	//軸などのもろもろ
	const int grid_width = 1;
	const int grid_type = 8;
	const int grid_font = cv::FONT_HERSHEY_COMPLEX;

	//空な文字列
	static const std::string empty_str;
	//グリッド線
	const int grid_count = 500;
	int j = 1;
	std::stringstream display_num;
	std::string str;
	for (float i = Y_half + grid_count / dis_range*Y_half; i < Y_max; i += grid_count / dis_range*Y_half) {
		cv::line(img, cv::Point(0, i), cv::Point(X_max, i), cv::Scalar(100, 100, 100), grid_width, grid_type);
		display_num << -1 * grid_count * j;
		j++;
		str = display_num.str();
		cv::putText(img, str, cv::Point(X_half, i), grid_font, 0.5, cv::Scalar(100, 100, 100), 1, CV_AA);
		display_num.str(empty_str);
	}
	j = 1;
	for (float i = Y_half - grid_count / dis_range*Y_half; i > 0; i -= grid_count / dis_range*Y_half) {
		cv::line(img, cv::Point(0, i), cv::Point(X_max, i), cv::Scalar(100, 100, 100), grid_width, grid_type);
		display_num << grid_count * j;
		j++;
		str = display_num.str();
		cv::putText(img, str, cv::Point(X_half, i), grid_font, 0.5, cv::Scalar(100, 100, 100), 1, CV_AA);
		display_num.str(empty_str);
	}
	j = 1;
	for (float i = X_half + grid_count / dis_range*X_half; i < X_max; i += grid_count / dis_range*X_half) {
		cv::line(img, cv::Point(i, 0), cv::Point(i, Y_max), cv::Scalar(100, 100, 100), grid_width, grid_type);
		display_num << grid_count * j;
		j++;
		str = display_num.str();
		cv::putText(img, str, cv::Point(i, Y_half), grid_font, 0.5, cv::Scalar(100, 100, 100), 1, CV_AA);
		display_num.str(empty_str);
	}
	//display_num = 0;
	j = 1;
	for (float i = X_half - grid_count / dis_range*X_half; i > 0; i -= grid_count / dis_range*X_half) {
		cv::line(img, cv::Point(i, 0), cv::Point(i, Y_max), cv::Scalar(100, 100, 100), grid_width, grid_type);
		display_num << -1 * grid_count * j;
		j++;
		str = display_num.str();
		cv::putText(img, str, cv::Point(i, Y_half), grid_font, 0.5, cv::Scalar(100, 100, 100), 1, CV_AA);
		display_num.str(empty_str);
	}
	//x軸
	cv::line(img, cv::Point(0, Y_half), cv::Point(X_max, Y_half), cv::Scalar(150, 150, 150), grid_width, grid_type);
	//y軸
	cv::line(img, cv::Point(X_half, 0), cv::Point(X_half, Y_max), cv::Scalar(150, 150, 150), grid_width, grid_type);
}
