//M_PIを使うのに必要
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <fstream>
#include <iostream>	//debug

#include "class.h"

rp_datas::rp_datas(){
	deg = 0.;
	distance = 0.;
	x = 0.;
	y = 0.;
}

void rp_datas::convert(void){
	x = distance * cos(deg * M_PI / 180.0);
	y = -1 * distance * sin(deg * M_PI / 180.0);
}

void measure::convert(void) {
	for (int i = 0; i < data.size();i++){
		data[i].convert();
	}
}

void measure::size_in(void) {
	count = data.size();
}

// void measure::erase(void) {
// 	deg.clear();
// 	distance.clear();
// 	x.clear();
// 	y.clear();
// }

void measure::add(const measure &next_data){
	for (int i = 0; i < next_data.data.size();i++){
		data.push_back(next_data.data[i]);
	}
}

void measure::sort(void){
	// std::ofstream before_data;
	// before_data.open("before.csv",std::ios::out);
	// before_data<<"deg,distance,x,y"<<std::endl;
	// for(int i=0;i<data.size();i++){
	// 	// std::cout<< data[i].deg<<" ";
	// 	before_data<<data[i].deg<<',';
	// 	before_data<<data[i].distance<<',';
	// 	before_data<<data[i].x<<',';
	// 	before_data<<data[i].y<<std::endl;
	// }
	//https://codezine.jp/article/detail/6020
	std::sort(data.begin(), data.end(), [](const rp_datas &x, const rp_datas &y) { return x.deg < y.deg; });
	// std::cout<<"---------------------------"<<std::endl;
	// std::ofstream after_data;
	// after_data.open("after.csv",std::ios::out);
	// after_data<<"deg,distance,x,y"<<std::endl;
	// for(int i=0;i<data.size();i++){
	// 	// std::cout<< data[i].deg<<" ";
	// 	after_data<<data[i].deg<<',';
	// 	after_data<<data[i].distance<<',';
	// 	after_data<<data[i].x<<',';
	// 	after_data<<data[i].y<<std::endl;
	// }
	// std::cout<<std::endl;
}

group::group(){
	count = 0;
	line = false;
	ball = false;
	a = 0.;
	b = 0.;
}

void group::size_in(void) {
	count = data.size();
}

void group::erase(void) {
	count = 0;
	line = false;
	ball = false;
	a = 0.;
	b = 0.;
	data.clear();
}

double group::max_x(void){
	std::vector<double> x_data;
	for (int i = 0; i < data.size();i++){
		x_data.push_back(data[i].x);
	}
	double max = *std::max_element(x_data.begin(), x_data.end());
	return max;
}

double group::min_x(void){
	std::vector<double> x_data;
	for (int i = 0; i < data.size();i++){
		x_data.push_back(data[i].x);
	}
	double min = *std::min_element(x_data.begin(), x_data.end());
	return min;
}

double group::max_y(void){
	std::vector<double> y_data;
	for (int i = 0; i < data.size();i++){
		y_data.push_back(data[i].y);
	}
	double max = *std::max_element(y_data.begin(), y_data.end());
	return max;
}

double group::min_y(void){
	std::vector<double> y_data;
	for (int i = 0; i < data.size();i++){
		y_data.push_back(data[i].y);
	}
	double max = *std::min_element(y_data.begin(), y_data.end());
	return max;
}

double group::ave_distance(void){
	double ave=0.;
	for(int i=0;i<data.size();i++){
		// std::cout<<data[i].distance<<" ";
		ave+=data[i].distance;
	}
	ave/=data.size();
	// std::cout<<std::endl;
	// std::cout<<"ave="<<ave<<std::endl;
	// std::cout<<"----------------"<<std::endl;
	
	return ave;
}

double group::ave_x(void){
	double ave=0.;
	for(int i=0;i<data.size();i++){
		ave+=data[i].x;
	}
	ave/=data.size();
	return ave;
}

double group::ave_y(void){
	double ave=0.;
	for(int i=0;i<data.size();i++){
		ave+=data[i].y;
	}
	ave/=data.size();
	return ave;
}