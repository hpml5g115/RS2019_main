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
//#include <wiringPi.h>
//#include <softPwm.h>
//#include <signal.h>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "myfunc.h"
#include "move.h"
#include "grouping.h"
#include "class.h"


int main(void) {
	RPlidarDriver * drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);

	//double goal_angle = 90;

	if (!drv) {
		printf("insufficent memory, exit\n");
		exit(-2);
	}

	Sig_Initialize();
	Pin_Initialize();

	if (Initialize(drv) == false) {
		ExitProcess(drv);
		return 0;
	}

	//そのまま開始すると、アームが保持状態のままになることがある
	//そのため、一度射出してフリーの状態にする。
	BallShoot();

	printf("走行を開始するにはEnterキーを押してください\n");
	getchar();

    group gr[50];
    int gr_num = 0;
    int line_count = 0;
    double dest_x, dest_y, dest_r, dest_theta;
    dest_x = 0;
    dest_y = 0;
    dest_r = 0;
    dest_theta = 0;

	drv->startMotor();

	// take only one 360 deg scan and display the result as a histogram
	////////////////////////////////////////////////////////////////////////////////
	if (IS_FAIL(drv->startScan( /* true */))) // you can force rplidar to perform scan operation regardless whether the motor is rotating
	{
		printf("Error, cannot start the scan operation.\n");
		ExitProcess(drv);
		return 0;
	}
	//リセット
	measure current_result;

    //ボール検知→回収
    int continue_flag = 1;
	while(1){
		int rev_count = 0;
	    do{
	    	if (FinderMeasure(drv, &current_result) == true) {
	            //リセット
	            continue_flag = 0;
				//初期化
				for(int pos = 0; pos < gr_num;pos++){
					gr[pos].erase();
				}
				gr_num = 0;
	            line_count = 0;
				dest_x = 0;
				dest_y = 0;
				dest_r = 0;
				dest_theta = 0;
	    		//座標変換
	    		current_result.convert();

	    		measure reduce_result;
	    		ZeroRemove(&current_result, &reduce_result);


	    		gr_num = MakeGroup(&reduce_result, gr);
				Connect(&gr[0], &gr[gr_num - 1]);


	    		line_count = ClassifyGroup(gr, gr_num);

	    		int min_gr = 0;
/*
				double min_gr_ave = 0;
				for(int i = 0; i < gr[min_gr].count; i++){
					min_gr_ave = gr[min_gr].distance[i];
				}
				min_gr_ave = min_gr_ave / gr[min_gr].count;
*/
	    		for(int pos = 1;pos < gr_num;pos++){
					if(gr[pos].line != true){
						/*
						double ave;
						for(int i = 0; i < gr[pos].count; i++){
							ave += gr[pos].distance[i];
						}
						ave = ave / gr[pos].count;
						*/
		    			//距離の平均値が一番近い物体を検知する
		    			//if(gr[pos].line == false && gr[pos].distance[0] < gr[min_gr].distance[0]) {
						if(gr[pos].distance[0] < gr[min_gr].distance[0]){
		        		//if(ave < min_gr_ave){
							//min_gr_ave = ave;
							min_gr = pos;
		                }
					}
	    		}

				for(int i = 0; i < gr[min_gr].count;i++){
					dest_x += gr[min_gr].x[i];
					dest_y += gr[min_gr].y[i];
				}
				dest_x = dest_x / gr[min_gr].count;
				dest_y = dest_y / gr[min_gr].count;


	    		//dest_x = (gr[min_gr].x[0] + gr[min_gr].x[gr[min_gr].count - 1]) / 2.0;
	    		//dest_y = (gr[min_gr].y[0] + gr[min_gr].y[gr[min_gr].count - 1]) / 2.0;
	    		dest_r = sqrt(pow(dest_x, 2) + pow(dest_y, 2));
	    		dest_theta = atan2(dest_y, dest_x);
	    		//radをdegに変換
	    		dest_theta = dest_theta * 180 / M_PI;

	    		//デバッグ用
	    		std::cout << "dest_x=" << dest_x << ", dest_y=" << dest_y << "\n";
	    		std::cout << "dest_r=" << dest_r << ", dest_theta=" << dest_theta << std::endl;

	    		//移動距離を微調整
	    		dest_r -= 50;

	            moving(dest_r, dest_theta);
				/*
				if(dest_theta < 0){
					goal_angle -= dest_theta;
				}
				else{
					goal_angle += dest_theta;
				}
				*/
	    	}
	        delay(1000);

	        //ボールを確保したかチェック
	        if(BallCaptured() == true){
	            std::cout << "Ball is Captured." <<std::endl;
				rev_count = 0;
	        }
	        else{
	            std::cout << "Ball is missing." << std::endl;
	            std::cout << "force capture..." << std::endl;
	            ForceCapture();
	            if(BallCaptured() == true){
	                std::cout << "Ball is Captured." << std::endl;
					rev_count = 0;
	            }
	            else{
	                std::cout <<"Ball is still missing." << std::endl;
					BallShoot();
					delay(500);
	                continue_flag = 1;
					rev_count++;
	            }
	        }
			if(rev_count > 2){
				rev_count = 0;
				rev(200);
			}
	    }while(continue_flag == 1);
		rev(300);


        current_result.erase();
	    //回収→ゴール
	    do{
	        if (FinderMeasure(drv, &current_result) == true) {
				//リセット
	            //初期化
	            for(int pos = 0; pos < gr_num;pos++){
	                gr[pos].erase();
	            }
	            gr_num = 0;
	            line_count = 0;

	            //座標変換
	            current_result.convert();

	            //ゼロの点を削除
	            measure reduce_result;
	            //int reduce_cnt = 0;
	            ZeroRemove(&current_result, &reduce_result);

	            gr_num = MakeGroup(&reduce_result, gr);

	            line_count = ClassifyGroup(gr, gr_num);

                const int turn_lim = 4;
                if(line_count == 0){
                    right(45);
                }

	        }

	/*			if(dest_theta < 0){
					goal_angle -= dest_theta;
				}
				else{
					goal_angle += dest_theta;
				}
	*/
	    }while(line_count == 0);

		//直線を複数検知した場合


	    int line_num = 0;
		//直線の配列番号を取得
		for(int pos = 0; pos < gr_num; pos++) {
			if(gr[pos].line == true) {
				line_num = pos;
				break;
			}
		}

	    double max_x,min_x,max_y,min_y;
		max_x = gr[line_num].x[0];
		min_x = gr[line_num].x[0];
		max_y = gr[line_num].y[0];
		min_y = gr[line_num].y[0];

		//最大値・最小値を探す
		for(int i = 0; i < gr[line_num].count; i++) {
			if(max_x < gr[line_num].x[i]) {
				max_x = gr[line_num].x[i];
			}
			if(min_x > gr[line_num].x[i]) {
				min_x = gr[line_num].x[i];
			}
			if(max_y < gr[line_num].y[i]) {
				max_y = gr[line_num].y[i];
			}
			if(min_y > gr[line_num].y[i]) {
				min_y = gr[line_num].y[i];
			}
		}

		double abs_x,abs_y;
		abs_x = fabs(max_x - min_x);
		abs_y = fabs(max_y - min_y);

		const double wall_length = 900;
		if(abs_x > abs_y) {
			dest_x = max_x;
			for(int i = 0; i < gr[line_num].count; i++) {
				if(gr[line_num].x[i] == dest_x) {
					dest_y = gr[line_num].y[i];
					break;
				}
			}
			//x座標が長いので壁の長さ=x座標
			if(dest_y >= 0){
				dest_y -= wall_length;
			}
			else{
				dest_y += wall_length;
			}
		}
		else {
			dest_y = max_y;
			for(int i = 0; i < gr[line_num].count; i++) {
				if(gr[line_num].y[i] == dest_y) {
					dest_x = gr[line_num].x[i];
					break;
				}
			}

			//y座標が長いので壁の長さ=y座標
			if(dest_x >= 0){
				dest_x -= wall_length;
			}
			else{
				dest_x += wall_length;
			}
		}


		dest_r = sqrt(pow(dest_x, 2) + pow(dest_y, 2));
		dest_theta = atan2(dest_y, dest_x);
		//radをdegに変換
		dest_theta = dest_theta * 180 / M_PI;

		//デバッグ用
		std::cout << "dest_x=" << dest_x << ", dest_y=" << dest_y << "\n";
		std::cout << "dest_r=" << dest_r << ", dest_theta=" << dest_theta << std::endl;

	    moving(dest_r, dest_theta);
	    delay(1000);

	    BallShoot();
        std::cout << "shoot completed.\n" << std::endl;
		delay(1000);
		rev(1000);
	}
	ExitProcess(drv);
	Exit_pin();

	return 0;
}
