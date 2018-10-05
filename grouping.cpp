//M_PIを使うのに必要
#define _USE_MATH_DEFINES
//#include <stdio.h>
//#include <stdlib.h>
#include <iostream>
//#include <fstream>
#include <cmath>

//#include "myfunc.h"
#include "grouping.h"


//ゼロの点を除去
void ZeroRemove(measure *result, measure *reduce){
    for (int pos = 0; pos < result->count; pos++) {
        if (result->distance[pos] != 0) {
            reduce->deg.push_back(result->deg[pos]);
            reduce->distance.push_back(result->distance[pos]);
            reduce->x.push_back(result->x[pos]);
            reduce->y.push_back(result->y[pos]);

        }
    }
    reduce->size_in();
}
//グループ作成
int MakeGroup(measure *result, group gr[50]){
    int num = 0;
    for (int pos = 0; pos < result->count - 1; ++pos) {
        double tmp_x = 0;
        double tmp_y = 0;
        tmp_x = fabs(result->x[pos] - result->x[pos + 1]);
        tmp_y = fabs(result->y[pos] - result->y[pos + 1]);

        if(tmp_x < group_lim && tmp_y < group_lim) {
            gr[num].deg.push_back(result->deg[pos]);
            gr[num].distance.push_back(result->distance[pos]);
            gr[num].x.push_back(result->x[pos]);
            gr[num].y.push_back(result->y[pos]);
        }
        else {
            //3点より少ないものは無視
            gr[num].size_in();
            if (gr[num].count < 3) {
                gr[num].erase();
            }
            else {
                num++;
            }
        }
    }

    gr[num].size_in();
    //3点より少ないものは無視
    if (gr[num].count < 3) {
        gr[num].erase();
    }
    else {
        num++;
    }

    return num;
}

void Connect(group *first, group *last){
    double tmp_x = 0;
    double tmp_y = 0;
    tmp_x = fabs(first->x[0] - last->x[last->count -1]);
    tmp_y = fabs(first->y[0] - last->y[last->count -1]);

    if(tmp_x < group_lim && tmp_y < group_lim) {
        for(int i = 0; i < last->count;i++){
            first->deg.push_back(last->deg[i]);
            first->distance.push_back(last->distance[i]);
            first->x.push_back(last->x[i]);
            first->y.push_back(last->y[i]);
        }
    }
}

double GroupLength(group *gr){
    double max_x,min_x,max_y,min_y;
    max_x = gr->x[0];
    min_x = gr->x[0];
    max_y = gr->y[0];
    min_y = gr->y[0];

    //最大値・最小値を探す
    for(int i = 0; i < gr->count; i++) {
        if(max_x < gr->x[i]) {
            max_x = gr->x[i];
        }
        if(min_x > gr->x[i]) {
            min_x = gr->x[i];
        }
        if(max_y < gr->y[i]) {
            max_y = gr->y[i];
        }
        if(min_y > gr->y[i]) {
            min_y = gr->y[i];
        }
    }

    double abs_x,abs_y;
    abs_x = fabs(max_x - min_x);
    abs_y = fabs(max_y - min_y);

    double length;
    if(abs_x > abs_y) {
        for(int i = 0; i < gr->count; i++) {
            if(gr->x[i] == max_x) {
                max_y = gr->y[i];
            }
            if(gr->x[i] == min_x){
                min_y = gr->y[i];
            }
        }
    }
    else {
        for(int i = 0; i < gr->count; i++) {
            if(gr->y[i] == max_y) {
                max_x = gr->x[i];
            }
            if(gr->y[i] == min_y){
                min_x = gr->x[i];
            }
        }
    }
    length = sqrt(pow(max_x - min_x, 2) + pow(max_y - min_y, 2));
    return length;
}

int ClassifyGroup(group gr[50], int gr_num){
    int num = 0;

    for(int pos = 0;pos < gr_num; pos++){
        if(GroupLength(&gr[pos]) > ball_lim){
            gr[pos].line = true;
            num++;
        }
        else{
            gr[pos].line = false;
        }
    }
    return num;
}
