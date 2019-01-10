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


int main(void) {
	Sig_Initialize();
	robomove mov;

	mov.BallDetect();
	std::cout<<"ball searching..."<<std::endl;
	while(mov.ChkBallState() == false||mov.Busy()==true);
	delay(1000);
	mov.LiftUp();
	std::cout<<"lift up..."<<std::endl;
	while(mov.Busy()==true);
	mov.FreeMode();
	delay(1000);
	mov.Shoot();
	std::cout<<"shoot..."<<std::endl;
	while(mov.Busy()==true);
	mov.FreeMode();

	return 0;
}