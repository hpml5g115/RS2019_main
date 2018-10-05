#ifndef _CLASS
#define _CLASS

#include <vector>

class measure {
public:
	//double deg[360 * 2];
	//double distance[360 * 2];
	//double x[360 * 2];
	//double y[360 * 2];
	std::vector<double> deg;
	std::vector<double> distance;
	std::vector<double> x;
	std::vector<double> y;
	int count;
	void convert(void);
	void size_in(void);
	void erase(void);
};

//グループ化
class group {
public:
	std::vector<double> x;
	std::vector<double> y;
	int count;
	std::vector<double> deg;
	std::vector<double> distance;
	bool line;
	double a;
	double b;

	void size_in(void);
	void erase(void);
};

#endif
