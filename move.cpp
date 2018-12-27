#include <wiringPi.h>
#include <signal.h>
#include <cmath>
#include <iostream>

#include "move.h"

void sigcatch(int sig){
	printf(" detected.\n");
	printf("motor stopping...");
	digitalWrite(L_PULSE, 0);
	digitalWrite(L_DIR, 0);
	digitalWrite(R_PULSE, 0);
	digitalWrite(R_DIR, 0);
	digitalWrite(SIG_SHOOT, 0);
	digitalWrite(SIG_FORCE, 0);
  	printf("done.\n");

	exit(1);
}

void Sig_Initialize(void){
    if(SIG_ERR == signal(SIGINT, sigcatch)){
        printf("failed to set signal handler.\n");
        exit(1);
    }
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

//新作部分
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
	pinMode(R_DIR, OUTPUT);
	pinMode(L_DIR, OUTPUT);
	pinMode(R_PULSE, OUTPUT);
	pinMode(L_PULSE, OUTPUT);
	pinMode(SIG_SHOOT, OUTPUT);
	pinMode(SIG_CAP, INPUT);
	pinMode(SIG_FORCE, OUTPUT);

	digitalWrite(L_PULSE, 0);
	digitalWrite(L_DIR, 0);
	digitalWrite(R_PULSE, 0);
	digitalWrite(R_DIR, 0);
	digitalWrite(SIG_SHOOT, 0);
	digitalWrite(SIG_FORCE, 0);
	std::cout << "setup finished." << std::endl;
}

robomove::~robomove(){
	if(thread_continue == true){
		thread_continue = false;
		move_th.join();
	}
	digitalWrite(L_PULSE, 0);
	digitalWrite(L_DIR, 0);
	digitalWrite(R_PULSE, 0);
	digitalWrite(R_DIR, 0);
	digitalWrite(SIG_SHOOT, 0);
	digitalWrite(SIG_FORCE, 0);
}

void robomove::Fwd(double distance){
	dir = M_FWD;
	pulse_num = MmToPulse(distance);
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
	dir = M_STOP;
	pulse_num = 0;
	move_update = true;
}

void robomove::ConvertToMove(double distance, double angle){
	if(angle<0.){
		double abs_angle = abs(angle);
		Right(abs_angle);
	}
	else{
		double abs_angle = abs(angle);
		Left(abs_angle);
	}
	//旋回完了まで待機
	while(move_finished==false);
	Fwd(distance);
}

bool robomove::ChkState(void){
	if(move_update == true){
		return false;
	}
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
	/*
	タイヤの直径58mm
	円周：182.12mm
	1パルスあたり182.12/360*1.8=0.9106mm
	*/
	// const double dirPerPulse = 58. * M_PI / 360. * 1.8;
	// std::cout<<dirPerPulse<<std::endl;
	// std::cout<<"pulse:"<<(distance/dirPerPulse)<<std::endl;
	// std::cout<<"pulse:"<<round(distance/dirPerPulse)<<std::endl;
	return round(distance / dirPerPulse);
}

long robomove::AngleToPulse(double angle){
	//回転角=トレッド(タイヤ間の距離)/タイヤの直径*角度？
	//210/58=3.620
	//90度旋回=3.620*90=325度
	//232/1.8=128.88
	//232/0.45=
	/* 1パルス1.8度
	1回転→200パルス
	実測→200パルスで90度
	*/
	// const double anglePerPulse = 210. / 58. / 1.8;
	// std::cout<<anglePerPulse<<std::endl;
	// std::cout<<"pulse:"<<(anglePerPulse*angle)<<std::endl;
	// std::cout<<"cast_pulse:"<<round(anglePerPulse*angle)<<std::endl;
	return round(anglePerPulse * angle);
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
					digitalWrite(R_DIR, 0);
					digitalWrite(L_DIR, 0);
					break;
				case M_REV:
					digitalWrite(R_DIR, 1);
					digitalWrite(L_DIR, 1);
					break;
				case M_RIGHT:
					digitalWrite(R_DIR, 1);
					digitalWrite(L_DIR, 0);
					break;
				case M_LEFT:
					digitalWrite(R_DIR, 0);
					digitalWrite(L_DIR, 1);
					break;
				default:
					//動作を中断
					move_finished = true;
					digitalWrite(R_DIR, 0);
					digitalWrite(L_DIR, 0);
					break;
			}
		}

		//移動中
		if(move_finished == false){
			digitalWrite(R_PULSE, 1);
			digitalWrite(L_PULSE, 1);
			delay(4);
			now_pulse_num++;
			if(now_pulse_num == pulse_num){
				move_finished = true;
			}
		}
		//PULSEを下げる
		digitalWrite(R_PULSE, 0);
		digitalWrite(L_PULSE, 0);
		delay(4);
	}
	std::cout << "move thread finished" << std::endl;
}