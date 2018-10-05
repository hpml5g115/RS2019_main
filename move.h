#ifndef _MOVE
#define _MOVE

//昔の配置
//PIC中心部より12,16,20,21
#define DIR_0 26
#define DIR_1 19
#define PULSE 13
#define COMPLETE 6

//サーボ系PICの接続
#define SIG_CAP 20
#define SIG_SHOOT 16
#define SIG_FORCE 12


const double one_move = 55;
const double one_turn = 6.0;

//Ctrl+Cのシグナルハンドラー
void sigcatch(int sig);

//wiringPiの初期化
void Pin_Initialize(void);
//シグナルハンドラーの初期化
void Sig_Initialize(void);

void Exit_pin(void);

//ステッピングモーターの移動制御
void fwd(double length);
void rev(double length);
void right(double angle);
void left(double angle);

//四捨五入用関数
int move_conv(double value);
int turn_conv(double value);


//サーボ関連
bool BallCaptured(void);
void BallShoot(void);

//強制回収
bool ForceCapture(void);

//動作命令の関数
void moving(double distance, double angle);

#endif
