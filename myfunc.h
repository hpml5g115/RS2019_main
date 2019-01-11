#ifndef _MYFUNC
#define _MYFUNC

#include "class.h"

// ピン番号を定義
// #define R_PULSE 16
// #define R_DIR 12
// #define L_PULSE 21
// #define L_DIR 20

const int X_max = 600;
const int Y_max = 600;
const int X_half = (int)(X_max / 2);
const int Y_half = (int)(Y_max / 2);
//表示するレンジ(これ以上は切り捨て)
const double dis_range = 6000.;

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(_word_size_t ms) {
	while (ms >= 1000) {
		usleep(1000 * 1000);
		ms -= 1000;
	};
	if (ms != 0)
		usleep(ms * 1000);
}
#endif



using namespace rp::standalone::rplidar;

//グラフ描画用プログラム
void GraphGain(double x, double y, int *xr, int *yr);
#ifndef _NO_GUI
	void PictureGrid(cv::Mat img);
#endif

//初期化関数
bool Initialize(RPlidarDriver * drv);
//データ取得用関数
bool FinderMeasure(RPlidarDriver * drv, measure * result);
//終了時の処理関数
void ExitProcess(RPlidarDriver * drv);

#endif
