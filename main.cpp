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

//GUIなしで実行
#define _NO_GUI

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
	// Pin_Initialize();

	// fwd(300);

	robomove mov;
	mov.Th_start();

	// std::cout << "fwd" << std::endl;
	// mov.Fwd(300);
	// delay(1);
	// while(mov.ChkState()==false);
	// delay(1000);

	std::cout << "rev" << std::endl;
	// mov.Rev(300);
	// delay(1);
	// while(mov.ChkState()==false);
	// delay(1000);

	std::cout << "right" << std::endl;
	mov.Right(90.);
	delay(1);
	while(mov.ChkState()==false);
	delay(1000);

	std::cout << "left" << std::endl;
	mov.Left(90.);
	delay(1);
	while(mov.ChkState()==false);
	delay(1000);

	mov.Th_end();



	// Exit_pin();

	return 0;
}
