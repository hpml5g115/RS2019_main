//M_PIを使うのに必要
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
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

void group::size_in(void) {
	count = data.size();
}

void group::erase(void) {
	data.erase();
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