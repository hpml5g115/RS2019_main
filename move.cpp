#include <wiringPi.h>
#include <signal.h>
#include <cmath>
#include <iostream>

#include "move.h"

void sigcatch(int sig){
	printf(" detected.\n");
	printf("motor stopping...");
	digitalWrite(DIR_0,0);
	digitalWrite(DIR_1,0);
	digitalWrite(PULSE,0);
	digitalWrite(SIG_SHOOT, 0);
	digitalWrite(SIG_FORCE, 0);
  	printf("done.\n");

	exit(1);
}
/*
int move_conv(double value) {
	int num;
	num = value / one_move;
	double tmp = value / one_move;
	double reminder;
	reminder = tmp - num;
//	std::cout << "function" << std::endl;
//	std::cout << "num=" << num <<", tmp=" << tmp << std::endl;
	if(reminder < 0.5) {
		return num;
	}
	else {
		return num + 1;
	}
}


int turn_conv(double value) {
	int num;
	num = value / one_turn;
	double tmp = value / one_turn;
	double reminder;
	reminder = tmp - num;
	if(reminder < 0.5) {
		return num;
	}
	else {
		return num + 1;
	}
}



void fwd(double length){
	int move_cnt;
    digitalWrite(DIR_0, 0);
    digitalWrite(DIR_1, 0);
    move_cnt = move_conv(length);
    delay(10);
    for(int i = 0; i < move_cnt; i++){
        digitalWrite(PULSE,1);
        //COMPLETEまで待機
        while(digitalRead(COMPLETE) == 0);
        //PULSEを下げる
        digitalWrite(PULSE, 0);
        delay(10);
    }
}
void rev(double length){
	int move_cnt;
    digitalWrite(DIR_0, 1);
    digitalWrite(DIR_1, 1);
    move_cnt = move_conv(length);
    delay(10);
    for(int i = 0; i < move_cnt; i++){
        digitalWrite(PULSE,1);
        //COMPLETEまで待機
        while(digitalRead(COMPLETE) == 0);
        //PULSEを下げる
        digitalWrite(PULSE, 0);
        delay(10);
    }
}
void right(double angle){
	int deg_cnt;
    digitalWrite(DIR_0, 1);
    digitalWrite(DIR_1, 0);
    deg_cnt = turn_conv(angle);

    delay(10);
    for(int i = 0; i < deg_cnt; i++){
        digitalWrite(PULSE,1);
        //COMPLETEまで待機
        while(digitalRead(COMPLETE) == 0);
        //PULSEを下げる
        digitalWrite(PULSE, 0);
        delay(10);
    }
}
void left(double angle){
	int deg_cnt;
    digitalWrite(DIR_0, 0);
    digitalWrite(DIR_1, 1);
    deg_cnt = turn_conv(angle);

    delay(10);
    for(int i = 0; i < deg_cnt; i++){
        digitalWrite(PULSE,1);
        //COMPLETEまで待機
        while(digitalRead(COMPLETE) == 0);
        //PULSEを下げる
        digitalWrite(PULSE, 0);
        delay(10);
    }
}

void Pin_Initialize(void){
	if(wiringPiSetupGpio() == -1){
		printf("wiringPi setup eroor.\n");
		exit(1);
	}

	//ピン初期化
	pinMode(DIR_0,OUTPUT);
	pinMode(DIR_1,OUTPUT);
	pinMode(PULSE,OUTPUT);
	pinMode(COMPLETE,INPUT);
	pinMode(SIG_SHOOT, OUTPUT);
	pinMode(SIG_CAP, INPUT);
	pinMode(SIG_FORCE, OUTPUT);

	digitalWrite(DIR_0,0);
	digitalWrite(DIR_1,0);
	digitalWrite(PULSE,0);
	digitalWrite(SIG_SHOOT, 0);
	digitalWrite(SIG_FORCE, 0);
}
*/

void Sig_Initialize(void){
    if(SIG_ERR == signal(SIGINT, sigcatch)){
        printf("failed to set signal handler.\n");
        exit(1);
    }
}

void Exit_pin(void){
	digitalWrite(DIR_0,0);
	digitalWrite(DIR_1,0);
	digitalWrite(PULSE,0);
	digitalWrite(SIG_SHOOT, 0);
	digitalWrite(SIG_FORCE, 0);
}

bool BallCaptured(void){
	if(digitalRead(SIG_CAP) != 0){
		return true;
	}
	else{
		return false;
	}
}


void BallShoot(void){
	digitalWrite(SIG_SHOOT, 1);
	delay(1000);
	digitalWrite(SIG_SHOOT, 0);
	delay(500);
}


bool ForceCapture(void){
	digitalWrite(SIG_FORCE, 1);
	delay(1500);
	digitalWrite(SIG_FORCE, 0);

	if(digitalRead(SIG_CAP) != 0){
		return true;
	}
	else{
		return false;
	}
}

/*
void moving(double distance, double angle){
	if(angle < 0){
		double abs_tmp = fabs(angle);
		right(abs_tmp);
	//			std::cout << "right" << std::endl;
	}
	else{
		double abs_tmp = fabs(angle);
		left(abs_tmp);
	//			std::cout << "left" << std::endl;
	}
	fwd(distance);
}
*/

robomove::robomove(){
	move_update = false;
	thread_continue = false;
	move_finished = false;
	move_abort = false;
	dir = M_STOP;
	pulse_num = 0;
	if(wiringPiSetupGpio() == -1){
		std::cerr << "wiringPi setup eroor!" << std::endl;
		exit(1);
	}

	//ピン初期化
	pinMode(DIR_0,OUTPUT);
	pinMode(DIR_1,OUTPUT);
	pinMode(PULSE,OUTPUT);
	pinMode(COMPLETE,INPUT);
	pinMode(SIG_SHOOT, OUTPUT);
	pinMode(SIG_CAP, INPUT);
	pinMode(SIG_FORCE, OUTPUT);

	digitalWrite(DIR_0,0);
	digitalWrite(DIR_1,0);
	digitalWrite(PULSE,0);
	digitalWrite(SIG_SHOOT, 0);
	digitalWrite(SIG_FORCE, 0);
	std::cout << "setup finished." << std::endl;
}

robomove::~robomove(){
	if(thread_continue==true){
		thread_continue = false;
		move_th.join();
	}
	digitalWrite(DIR_0,0);
	digitalWrite(DIR_1,0);
	digitalWrite(PULSE,0);
	digitalWrite(SIG_SHOOT, 0);
	digitalWrite(SIG_FORCE, 0);
}

void robomove::Fwd(double distance){
	std::cout<<"fwd"<<std::endl;
	dir = M_FWD;
	pulse_num = MmToPulse(distance);
	std::cout<<pulse_num<<std::endl;
	move_update = true;
}

void robomove::Rev(double distance){
	dir = M_REV;
	pulse_num = MmToPulse(distance);
	move_update = true;
}

void robomove::Right(double angle){
	dir = M_RIGHT;
	pulse_num = AngleToPulse(angle);
	move_update = true;
}

void robomove::Left(double angle){
	dir = M_LEFT;
	pulse_num = AngleToPulse(angle);
	move_update = true;
}

void robomove::Stop(void){
	move_abort = false;
}

bool robomove::ChkState(void){
	return move_finished;
}

void robomove::Th_start(void){
	thread_continue = true;
	std::thread th(&robomove::Run, this);
	move_th = std::move(th);
}

void robomove::Th_end(void){
	thread_continue = false;
	move_th.join();
}

long robomove::MmToPulse(double distance){
	//進みに対してどれくらいのパルスが必要か計算
	long num;
	num = distance / one_move;
	double tmp = distance / one_move;
	double reminder;
	reminder = tmp - num;
//	std::cout << "function" << std::endl;
//	std::cout << "num=" << num <<", tmp=" << tmp << std::endl;
	if(reminder < 0.5) {
		return num;
	}
	else {
		return num + 1;
	}
}

long robomove::AngleToPulse(double angle){
	//回転に対してどれくらいのパルスが必要か計算
	long num;
	num = angle / one_turn;
	double tmp = angle / one_turn;
	double reminder;
	reminder = tmp - num;
	if(reminder < 0.5) {
		return num;
	}
	else {
		return num + 1;
	}
}

void robomove::Run(void){
	long now_pulse_num = 0;
	thread_continue = true;
	while(thread_continue == true){
		//移動の指令値が出たとき
		if(move_update == true){
			now_pulse_num = 0;
			move_update = false;
			move_finished = false;
			switch(dir){
				case M_FWD:
					std::cout<<"fwd move in thread"<<std::endl;
					digitalWrite(DIR_0, 0);
					digitalWrite(DIR_1, 0);
					break;
				case M_REV:
					digitalWrite(DIR_0, 1);
					digitalWrite(DIR_1, 1);
					break;
				case M_RIGHT:
					digitalWrite(DIR_0, 1);
					digitalWrite(DIR_1, 0);
					break;
				case M_LEFT:
					digitalWrite(DIR_0, 0);
					digitalWrite(DIR_1, 1);
					break;
				default:
					digitalWrite(DIR_0, 0);
					digitalWrite(DIR_1, 0);
					break;
			}
		}
		//旧版のPICに合わせているので、いずれパルスに応じて要修正
		//移動中
		if(move_finished == false){
			digitalWrite(PULSE, 1);
			//COMPLETEまで待機
			while(digitalRead(COMPLETE) == 0){
				std::cout<<"wait..."<<std::endl;
			}
			//PULSEを下げる
			digitalWrite(PULSE, 0);
			now_pulse_num++;
			std::cout<<now_pulse_num<<std::endl;
			if(now_pulse_num == pulse_num){
				std::cout<<now_pulse_num<<std::endl;
				move_finished = true;
			}
		}
		// delay(10);
	}
	std::cout << "move thread finished" << std::endl;
}