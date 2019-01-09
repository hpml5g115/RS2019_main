#ifndef _CLASS
#define _CLASS

#include <vector>

class rp_datas{
public:
	double deg;
	double distance;
	double x;
	double y;
	void convert(void);

private:
	rp_datas();
};

class measure {
public:
	// std::vector<double> deg;
	// std::vector<double> distance;
	// std::vector<double> x;
	// std::vector<double> y;
  std::vector<rp_datas> data;
  int count;
  void convert(void);
  void size_in(void);
//   void erase(void);
  void add(const measure &next_data);
  void sort(void);
};

//グループ化
class group {
public:
	// std::vector<double> x;
	// std::vector<double> y;
	// std::vector<double> deg;
	// std::vector<double> distance;
	std::vector<rp_datas> data;
	int count;
	bool line;
	double a;
	double b;

	void size_in(void);
	void erase(void);
	double max_x(void);
	double min_x(void);
	double max_y(void);
	double min_y(void);
};

#endif
