#include <wiringPi.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
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
