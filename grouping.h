#ifndef _GROUPING
#define _GROUPING

#include "class.h"

//しきい値 [mm]
const double group_lim = 100.0;
//グループ内の点の距離で判別する
const double ball_max_lim = 200.;
const double ball_min_lim = 100.;
const double wall_lim = 300.;
//除外しない最低点の数
const int dot_num_lim = 5;


//ゼロの点を除去
void ZeroRemove(measure *result, measure *reduce);
//グループ作成
int MakeGroup(measure *result, group gr[50]);

int ClassifyGroup(group gr[50], int gr_num);

void Connect(group *first, group *last);

double GroupLength(const group &gr);

#endif
