#ifndef _GROUPING
#define _GROUPING

#include "class.h"

//しきい値 [mm]
const double group_lim = 100.0;
//グループ内の点の数で判別する
const int ball_lim = 200;


//ゼロの点を除去
void ZeroRemove(measure *result, measure *reduce);
//グループ作成
int MakeGroup(measure *result, group gr[50]);

int ClassifyGroup(group gr[50], int gr_num);

void Connect(group *first, group *last);

double GroupLength(group *gr);

#endif
