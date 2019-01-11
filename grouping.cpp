//M_PIを使うのに必要
#define _USE_MATH_DEFINES
//#include <stdio.h>
//#include <stdlib.h>
#include <iostream>
//#include <fstream>
#include <cmath>
#include <algorithm>

//#include "myfunc.h"
#include "grouping.h"


//ゼロの点を除去
void ZeroRemove(measure *result, measure *reduce){
    for (int pos = 0; pos < result->count; pos++) {
        if (result->data[pos].distance != 0) {
            reduce->data.push_back(result->data[pos]);
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
        tmp_x = fabs(result->data[pos].x - result->data[pos + 1].x);
        tmp_y = fabs(result->data[pos].y - result->data[pos + 1].y);
        double tmp_dis=sqrt(tmp_x*tmp_x+tmp_y*tmp_y);
        // if(tmp_x < group_lim && tmp_y < group_lim) {
        if(tmp_dis < group_lim) {
            gr[num].data.push_back(result->data[pos]);
        }
        else {
            //3点より少ないものは無視
            gr[num].size_in();
            if (gr[num].count < dot_num_lim) {
                gr[num].erase();
            }
            else {
                num++;
            }
        }
    }

    gr[num].size_in();
    //3点より少ないものは無視
    if (gr[num].count < dot_num_lim) {
        gr[num].erase();
    }
    else {
        num++;
    }

    return num;
}

int Connect(group *first, group *last, int gr_num){
    double tmp_x = 0;
    double tmp_y = 0;
    tmp_x = fabs(first->data[0].x - last->data[last->count - 1].x);
    tmp_y = fabs(first->data[0].y - last->data[last->count - 1].y);

    double tmp_dis=sqrt(tmp_x*tmp_x+tmp_y*tmp_y);
    // if(tmp_x < group_lim && tmp_y < group_lim) {
    if(tmp_dis < group_lim) {
        for(int i = 0; i < last->count;i++){
            first->data.push_back(last->data[i]);
            first->size_in();
        }
        std::cout<<"last is connected with first"<<std::endl;
        last->erase();
        return gr_num-1;
    }
    return gr_num;
}

double GroupLength(group *gr){
    double max_x,min_x,max_y,min_y;
    // max_x = gr->x[0];
    // min_x = gr->x[0];
    // max_y = gr->y[0];
    // min_y = gr->y[0];

    // //最大値・最小値を探す
    // for(int i = 0; i < gr->count; i++) {
    //     if(max_x < gr->x[i]) {
    //         max_x = gr->x[i];
    //     }
    //     if(min_x > gr->x[i]) {
    //         min_x = gr->x[i];
    //     }
    //     if(max_y < gr->y[i]) {
    //         max_y = gr->y[i];
    //     }
    //     if(min_y > gr->y[i]) {
    //         min_y = gr->y[i];
    //     }
    // }
    max_x = gr->max_x();
    min_x = gr->min_x();
    max_y = gr->max_y();
    min_y = gr->min_y();

    double abs_x,abs_y;
    abs_x = fabs(max_x - min_x);
    abs_y = fabs(max_y - min_y);

    double length;
    if(abs_x > abs_y) {
        for(int i = 0; i < gr->count; i++) {
            if(gr->data[i].x == max_x) {
                max_y = gr->data[i].y;
            }
            if(gr->data[i].x == min_x){
                min_y = gr->data[i].y;
            }
        }
    }
    else {
        for(int i = 0; i < gr->count; i++) {
            if(gr->data[i].y == max_y) {
                max_x = gr->data[i].x;
            }
            if(gr->data[i].y == min_y){
                min_x = gr->data[i].x;
            }
        }
    }
    length = sqrt(pow(max_x - min_x, 2) + pow(max_y - min_y, 2));
    return length;
}

int ClassifyGroup(group gr[50], int gr_num){
    int line_num = 0;

    for(int pos = 0;pos < gr_num; pos++){
        gr[pos].line = false;
        gr[pos].ball = false;
        double length = GroupLength(&gr[pos]);
        if (length > wall_lim){
            gr[pos].line = true;
            line_num++;
        }
        if (length < ball_max_lim && length > ball_min_lim){
            // std::cout<<length<<" ";
            std::cout<<pos<<" ave:"<<gr[pos].average()<<std::endl;
            gr[pos].ball = true;
        }
    }
    return line_num;
}
