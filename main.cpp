/* simple_route…
 * 最小二乗法を用いたボール・壁の検知がうまくいってない
 * グループの点数で分けてしまう
 */

//M_PIを使うのに必要
#define _USE_MATH_DEFINES
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
//#include <fstream>
#include <cmath>
#include <algorithm>
#include <unistd.h>	//ファイル移動など
//#include <wiringPi.h>
//#include <softPwm.h>
//#include <signal.h>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

//GUIなしで実行
// #define _NO_GUI

#ifndef _NO_GUI
	#include <opencv2/core/core.hpp>
	#include <opencv2/highgui/highgui.hpp>
	#include <opencv2/opencv.hpp>
#endif

#include "myfunc.h"
#include "move.h"
#include "grouping.h"
#include "class.h"

// #define _ARM_TEST
// #define _GOAL_TEST

#ifdef _ARM_TEST
int main(void) {
	for(int i =0;i<2;i++){
		Sig_Initialize();
		robomove mov;
		mov.BallDetect();
		std::cout<<"ball searching..."<<std::endl;
		while(mov.ChkBallState() == false||mov.Busy()==true);
		delay(1000);
		if(mov.ChkBallState() == true){
			mov.LiftUp();
			std::cout<<"lift up..."<<std::endl;
			while(mov.Busy()==true);
			mov.FreeMode();
			delay(1000);
			mov.Shoot();
			std::cout<<"shoot..."<<std::endl;
			while(mov.Busy()==true);
		}

		mov.FreeMode();
		delay(1000);
	}
	return 0;
}
#else

int main(void) {
	Sig_Initialize();
	robomove mov;
	mov.Th_start();

	RPlidarDriver * drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
	//double goal_angle = 90;

	if (!drv) {
		printf("insufficent memory, exit\n");
		exit(-2);
	}

	if (Initialize(drv) == false) {
		ExitProcess(drv);
		return 0;
	}

	//そのまま開始すると、アームが保持状態のままになることがある
	//そのため、一度射出してフリーの状態にする。
	// BallShoot();

	printf("走行を開始するにはEnterキーを押してください\n");
	getchar();

    group gr[50];
    int gr_num = 0;

#ifndef _NO_GUI
	cv::namedWindow("group_image", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
#endif

	drv->startMotor();

	// take only one 360 deg scan and display the result as a histogram
	////////////////////////////////////////////////////////////////////////////////
	if (IS_FAIL(drv->startScan( /* true */))) // you can force rplidar to perform scan operation regardless whether the motor is rotating
	{
		printf("Error, cannot start the scan operation.\n");
		ExitProcess(drv);
		return 0;
	}

    //ボール検知→回収
    int continue_flag = 1;
	while(1){
//debug
#ifndef _GOAL_TEST
		int rev_count = 0;
		std::cout<<"------ball capture phase-------"<<std::endl;
	    do{
			mov.BallDetect();
			//リセット
			measure current_result;
			if (MeasureTwice(drv, &current_result) == true){
				//リセット
	            continue_flag = 0;
				//初期化
				for(int pos = 0; pos < gr_num;pos++){
					gr[pos].erase();
				}
				gr_num = 0;
	    		//座標変換
	    		current_result.convert();

				gr_num = Grouping(&current_result, gr);
				int line_count = ClassifyGroup(gr, gr_num);
				int ball_count = 0;
				// std::cout<<"line     ball"<<std::endl;
				for (int i = 0; i < gr_num;i++){
					// std::cout<<std::boolalpha<<gr[i].line<<"   "<<gr[i].ball<<std::endl;
					if (gr[i].ball == true){
						ball_count++;
					}
				}

#ifndef _NO_GUI
				cv::Mat group_img = cv::Mat::zeros(X_max, Y_max, CV_8UC3);
				group_img = cv::Scalar(0, 0, 0);
	    		PictureGrid(group_img);
	    		std::stringstream con_str;
	    		con_str << "Object" << gr_num;
	    		std::string str =con_str.str();
	    		con_str << " Lines" <<line_count;
				con_str << " Balls" << ball_count;
				str = con_str.str();

	    		cv::putText(group_img, str, cv::Point(0, 30), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255, 255, 255), 2, CV_AA);
	    		for (int pos = 0; pos < gr_num; pos++) {
	    			if (gr[pos].line == true) {
	    				for (int i = 0; i < gr[pos].count; i++) {
	    					int xr, yr;
							GraphGain(gr[pos].data[i].x, gr[pos].data[i].y, &xr, &yr);
							cv::circle(group_img, cv::Point(xr, yr), 1, cv::Scalar(255, 0, 0), -1, 8);
	    				}
	    			}
					else if (gr[pos].ball == true){
						for (int i = 0; i < gr[pos].count; i++){
							int xr, yr;
							GraphGain(gr[pos].data[i].x, gr[pos].data[i].y, &xr, &yr);
							//B G R
							cv::circle(group_img, cv::Point(xr, yr), 1, cv::Scalar(211, 0, 148), -1, 8);
						}
					}
	    			else {
	    				for (int i = 0; i < gr[pos].count; i++) {
	    					int xr, yr;
							GraphGain(gr[pos].data[i].x, gr[pos].data[i].y, &xr, &yr);
	    					cv::circle(group_img, cv::Point(xr, yr), 1, cv::Scalar(0, 0, 255), -1, 8);
	    				}
	    			}
	    		}
#endif
	    		int min_gr = 0;
				bool ball_found = false;
	    		for(int pos = 0;pos < gr_num;pos++){
					if(gr[pos].ball == true){
						if(ball_found == false){
							ball_found = true;
							min_gr = pos;
						}

						//距離の平均値が一番近い物体を検知する
						// if (gr[pos].data[0].distance < gr[min_gr].data[0].distance){
						if(gr[pos].ave_distance() < gr[min_gr].ave_distance()){
							min_gr = pos;
						}
					}
	    		}
				double dest_x, dest_y, dest_r, dest_theta;
				dest_x = 0.;
				dest_y = 0.;
				dest_r = 0.;
				dest_theta = 0.;

				if(ball_found == true){
					std::cout<<"target gr_num="<<min_gr<<std::endl;
					// for(int i = 0; i < gr[min_gr].count;i++){
					// 	dest_x += gr[min_gr].data[i].x;
					// 	dest_y += gr[min_gr].data[i].y;
					// }
					// dest_x = dest_x / gr[min_gr].count;
					// dest_y = dest_y / gr[min_gr].count;

					dest_x = gr[min_gr].ave_x();
					dest_y = gr[min_gr].ave_y();

					dest_r = sqrt(pow(dest_x, 2) + pow(dest_y, 2));
					dest_theta = atan2(dest_y, dest_x);
					//radをdegに変換
					dest_theta = dest_theta * 180 / M_PI;

					//デバッグ用
					std::cout << "dest_x=" << dest_x << ", dest_y=" << dest_y << "\n";
					std::cout << "dest_r=" << dest_r << ", dest_theta=" << dest_theta << std::endl;

					//移動距離を微調整
					dest_r -= 50;
				}
				//ボールが見つからなかったとき
				else{
					std::cout << "can't find balls!" << std::endl;
					dest_r = 100.;
					dest_theta = 0.;
				}
#ifndef _NO_GUI
				//debug
				// cv::Mat raw_img = cv::Mat::zeros(X_max, Y_max, CV_8UC3);
				// raw_img = cv::Scalar(0, 0, 0);
	    		// PictureGrid(raw_img);
				// for(int i=0;i<current_result.data.size();i++){
				// 	int xr, yr;
				// 	GraphGain(current_result.data[i].x, current_result.data[i].y, &xr, &yr);
				// 	cv::circle(raw_img, cv::Point(xr, yr), 1, cv::Scalar(255, 0, 0), -1, 8);
				// }
				// cv::imshow("group_image",raw_img);
				// cv::waitKey(0);

	    		int dest_xr, dest_yr;
	    		GraphGain(dest_x, dest_y, &dest_xr, &dest_yr);
	    		cv::circle(group_img, cv::Point(dest_xr, dest_yr), 3, cv::Scalar(0, 255, 0), -1, 8);
				cv::imshow("group_image",group_img);
				cv::waitKey(0);
				//目的地→緑
				//Line→青
				//ボール→紫
				//それ以外→赤
#endif

				mov.ConvertToMove(dest_r, dest_theta,4);
	    	}
	        // delay(1000);
			bool ball_captured = false;
			while (mov.ChkMoveState() == false){
				//ボールを確保したかチェック
				if (mov.ChkBallState() == true){
					mov.Stop();
					ball_captured = true;
					std::cout << "Ball is Captured." << std::endl;
					rev_count = 0;
					delay(1000);
					//途中でボールをリリースしてしまった場合
					if(mov.ChkBallState() == false){
						continue_flag = 1;
						rev_count++;
					}


					//持ち上げ
					// mov.LiftUp();
					// while(mov.Busy() == true);

					// //途中でボールをリリースしてしまった場合
					// if(mov.ChkBallState() == false){
					// 	mov.Shoot();
					// 	while(mov.Busy() == true);
					// 	continue_flag = 1;
					// 	rev_count++;
					// }

					//debug
					break;
				}
			}
			if(ball_captured == false){
				std::cout << "Ball is missing." << std::endl;
				//機構が変わったので強制ホールド不可
				continue_flag = 1;
				rev_count++;
			}

			if(rev_count > 2){
				// rev_count = 0;
				delay(100);
				//事故回数に応じて後退
				mov.Rev(200*rev_count,3);
				while(mov.ChkMoveState() == false);
			}
	    }while(continue_flag == 1);
		// mov.Rev(300);
		// while(mov.ChkMoveState()==false);

#endif

        // current_result.erase();
		std::cout<<"------goal phase-------"<<std::endl;
		int line_count = 0;
	    //回収→ゴール
		//移動した座標を保存
		// std::vector<double> log_r,log_theta;
		std::vector<double> log_r,log_theta;
		int step = 0;
		while(step < 3){
			do{
				//リセット
				measure current_result;
				if (MeasureTwice(drv, &current_result) == true){
					//初期化
					for(int pos = 0; pos < gr_num;pos++){
						gr[pos].erase();
					}

					//座標変換
					current_result.convert();

					// gr_num = 0;
					// line_count = 0;
					gr_num = Grouping(&current_result, gr);
					line_count = ClassifyGroup(gr, gr_num);

					if(line_count == 0){
						// right(45);
						mov.Right(45);
						while(mov.ChkMoveState()==false);
					}
				}
			}while(line_count == 0);
			std::cout << "data update" << std::endl;

#ifndef _NO_GUI
			int ball_count = 0;
			for (int i = 0; i < gr_num;i++){
				// std::cout<<std::boolalpha<<gr[i].line<<"   "<<gr[i].ball<<std::endl;
				if (gr[i].ball == true){
					ball_count++;
				}
			}
			cv::Mat group_img = cv::Mat::zeros(X_max, Y_max, CV_8UC3);
			group_img = cv::Scalar(0, 0, 0);
			PictureGrid(group_img);
			std::stringstream con_str;
			con_str << "Object" << gr_num;
			std::string str =con_str.str();
			con_str << " Lines" <<line_count;
			con_str << " Balls" << ball_count;
			str = con_str.str();
			cv::putText(group_img, str, cv::Point(0, 30), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255, 255, 255), 2, CV_AA);
			for (int pos = 0; pos < gr_num; pos++) {
				if (gr[pos].line == true) {
					for (int i = 0; i < gr[pos].count; i++) {
						int xr, yr;
						GraphGain(gr[pos].data[i].x, gr[pos].data[i].y, &xr, &yr);
						cv::circle(group_img, cv::Point(xr, yr), 1, cv::Scalar(255, 0, 0), -1, 8);
					}
				}
				else if (gr[pos].ball == true){
					for (int i = 0; i < gr[pos].count; i++){
						int xr, yr;
						GraphGain(gr[pos].data[i].x, gr[pos].data[i].y, &xr, &yr);
						//B G R
						cv::circle(group_img, cv::Point(xr, yr), 1, cv::Scalar(211, 0, 148), -1, 8);
					}
				}
				else {
					for (int i = 0; i < gr[pos].count; i++) {
						int xr, yr;
						GraphGain(gr[pos].data[i].x, gr[pos].data[i].y, &xr, &yr);
						cv::circle(group_img, cv::Point(xr, yr), 1, cv::Scalar(0, 0, 255), -1, 8);
					}
				}
			}
#endif

			int line_num = 0;
			//直線の配列番号を取得
			bool line_first_update = false;
			for(int pos = 0; pos < gr_num; pos++) {
				if(gr[pos].line == true) {
					if(line_first_update == false){
						line_num = pos;
						line_first_update = true;
						std::cout << pos << " " << GroupLength(&gr[pos]) << std::endl;
					}
					else{
						double max_length = GroupLength(&gr[line_num]);
						double now_length = GroupLength(&gr[pos]);
						std::cout << pos << " " << now_length << std::endl;
						if(max_length < now_length){
							line_num = pos;
						}
					}
				}
			}
			std::cout << "line_num=" << line_num << std::endl;

			double dest_x, dest_y, dest_r, dest_theta;
			dest_x = 0.;
			dest_y = 0.;
			dest_r = 0.;
			dest_theta = 0.;

			bool front_adjust = false;

			//角度補正の許容値
			const double target_angle = 180.;
			const double angle_diff = 5.;
			const double angle_min = target_angle - angle_diff;
			const double angle_max = target_angle + angle_diff;

			//途中まで移動
			if(step == 0){
				std::cout << "close mode" << std::endl;
				const double shrink_distance = 900.;
				dest_x = gr[line_num].ave_x();
				dest_y = gr[line_num].ave_y();
				dest_r = sqrt(pow(dest_x, 2) + pow(dest_y, 2)) - shrink_distance;
				dest_theta = atan2(dest_y, dest_x);
				//GUI用に再変換
				dest_x = dest_r * cos(dest_theta);
				dest_y = dest_r * sin(dest_theta);
				//radをdegに変換
				dest_theta = dest_theta * 180 / M_PI;
			}
			else if(step == 1){
				std::cout << "------angle adjust mode------" << std::endl;
				//一番近い部分の角度分旋回→後退
				dest_r = gr[line_num].min_distance();
				double raw_deg = 0.;
				for(int i = 0; i<gr[line_num].data.size();i++){
					if(dest_r == gr[line_num].data[i].distance){
						dest_theta = -1. * (gr[line_num].data[i].deg - target_angle);
						raw_deg = gr[line_num].data[i].deg;
						// if(tmp_deg > 360.){
						// 	tmp_deg -= 360.;
						// 	tmp_deg *= -1.;
						// }
						// else if(tmp_deg > 180.){
						// 	tmp_deg -= 180.;
						// }
						// else{
						// 	tmp_deg *= -1.;
						// }
						// dest_theta = -1. * tmp_deg;

						break;
					}
				}
				//反転させる
				std::cout << "raw_deg:" << raw_deg << std::endl;
				if(raw_deg > angle_min && raw_deg < angle_max){
					front_adjust = true;
				}
			}
			else if(step == 2){
				std::cout << "------Reverse-----" <<std::endl;
				//一番近い部分の角度分旋回→後退
				const double target_angle = 180.;
				dest_r = gr[line_num].min_distance();
				double raw_deg = 0.;
				for(int i = 0; i<gr[line_num].data.size();i++){
					if(dest_r == gr[line_num].data[i].distance){
						dest_theta = -1. * (gr[line_num].data[i].deg - target_angle);
						raw_deg = gr[line_num].data[i].deg;
						break;
					}
				}
				//反転させる
				std::cout << "raw_deg:" << raw_deg << std::endl;
				if(raw_deg > angle_min && raw_deg < angle_max){
					front_adjust = true;
				}
				else{
					front_adjust = false;
					step--;
				}
			}


			//デバッグ用
			std::cout << "dest_x=" << dest_x << ", dest_y=" << dest_y << "\n";
			std::cout << "dest_r=" << dest_r << ", dest_theta=" << dest_theta << std::endl;

#ifndef _NO_GUI
			int dest_xr, dest_yr;
			GraphGain(dest_x, dest_y, &dest_xr, &dest_yr);
			cv::circle(group_img, cv::Point(dest_xr, dest_yr), 3, cv::Scalar(0, 255, 0), -1, 8);
			cv::imshow("group_image",group_img);
			cv::waitKey(0);
			//目的地→緑
			//Line→赤
			//それ以外→青
#endif
			if(step == 0){
				log_r.push_back(dest_r);
				// log_theta.push_back(0.);
				mov.ConvertToMove(dest_r, dest_theta, 5);
				while(mov.ChkMoveState() == false);
				step++;
				mov.LiftUp();
				while(mov.Busy() == true);
			}
			else if(step == 1){
				if(front_adjust == true){
					//後退して位置合わせ
					const double wall_diff = 550.;
					double rev_length = dest_r - wall_diff;
					std::cout << "rev_length=" << rev_length << std::endl;
					if(rev_length>0.){
						mov.Rev(rev_length, 5);
						while(mov.ChkMoveState() == false);
						log_r.push_back(rev_length);
						// log_theta.push_back(0.);
					}
					step++;
				}
				else{
					log_r.push_back(0.);
					// log_theta.push_back(dest_theta);
					mov.ConvertToMove(0., dest_theta, 5);
					while(mov.ChkMoveState() == false);
				}
			}
			else if(step == 2){
				if(mov.ChkMoveState() == true){
					break;
				}
				delay(500);
			}
		}

		//持ち上げ
		// delay(500);
        mov.Shoot();
	    while(mov.Busy() == true);
	    mov.FreeMode();
		std::cout << "shoot completed.\n"
				  << std::endl;
		delay(100);
		std::cout << "log_size=" << log_r.size() << std::endl;
		double distance_total = 0.;
		for (int i = log_r.size() - 1; i >= 0; i--){
			// std::cout << "dest_r=" << log_r[i] << ", dest_theta=" << -log_theta[i] << std::endl;
			distance_total += log_r[i];
			std::cout << "dest_r=" << log_r[i] << std::endl;
			// mov.ConvertToMove(log_r[i], 0.);
			// while(mov.ChkMoveState() == false);
			// delay(500);
		}
		mov.ConvertToMove(distance_total*0.7, 0.,4);
		while(mov.ChkMoveState() == false);
#ifndef _NO_GUI
		//debug
		cv::waitKey(0);
#endif
	}
	ExitProcess(drv);

	// Exit_pin();

	return 0;
}
#endif