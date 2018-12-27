#ifndef _MOVE
#define _MOVE

#include <thread>

const int L_PULSE = 26;
const int L_DIR = 19;
const int R_PULSE = 13;
const int R_DIR = 6;

//サーボ系PICの接続
#define SIG_CAP 20
#define SIG_SHOOT 16
#define SIG_FORCE 12

const double dirPerPulse = 58. * M_PI / 360. * 1.8;
const double anglePerPulse = 210. / 58. / 1.8;

const int M_STOP = 0;
const int M_FWD = 1;
const int M_REV = 2;
const int M_RIGHT = 3;
const int M_LEFT = 4;

//Ctrl+Cのシグナルハンドラー
void sigcatch(int sig);

//wiringPiの初期化
// void Pin_Initialize(void);
//シグナルハンドラーの初期化
void Sig_Initialize(void);

//サーボ関連
bool BallCaptured(void);
void BallShoot(void);

//強制回収
bool ForceCapture(void);

//新版　マルチスレッド化によりパルス出しっぱなしにする
class robomove{
public:
    robomove();
    ~robomove();
    //前進(mm)
    void Fwd(double distance);
    void Rev(double distance);
    void Right(double angle);
    void Left(double angle);
    void Stop(void);
    void ConvertToMove(double distance, double angle);
    bool ChkState(void);

    void Th_start(void);
    void Th_end(void);
    
private:
  bool move_update;
  std::thread move_th;
  bool thread_continue;
  bool move_finished;
  int dir;
  long pulse_num;

  long MmToPulse(double distance);
  long AngleToPulse(double angle);
  void Run(void);
};
#endif
