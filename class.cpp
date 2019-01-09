//M_PIを使うのに必要
#define _USE_MATH_DEFINES
#include <cmath>
#include "class.h"

void measure::convert(void) {
	for (int i = 0; i < count; i++) {
		x.push_back(distance[i] * cos(deg[i] * M_PI / 180.0));
		y.push_back(-1 * distance[i] * sin(deg[i] * M_PI / 180.0));
	}
}

void measure::size_in(void) {
	count = deg.size();
}

void measure::erase(void) {
	deg.clear();
	distance.clear();
	x.clear();
	y.clear();
}

void measure::add(const measure &data){
	for (int i = 0; i < data.count;i++){
		deg.push_back(data.deg[i]);
		distance.push_back(data.distance[i]);
	}
}

void group::size_in(void) {
	count = deg.size();
}

void group::erase(void) {
	deg.clear();
	distance.clear();
	x.clear();
	y.clear();
}
