//
// Created by 0-0 mashuo on 2023/5/13.
//



#include "BasicFuns.h"


// 后续需要设置参数配置文件读取
extern double ELLIPSOID_a = 6378137;
extern double ELLIPSOID_b = 6356752.3141;
extern double ELLIPSOID_OMEGA = 7.292115e-5;
extern double ELLIPSOID_GM = 3.986005e14;
extern double ELLIPSOID_E2 = (ELLIPSOID_a * ELLIPSOID_a - ELLIPSOID_b * ELLIPSOID_b) / (ELLIPSOID_a * ELLIPSOID_a);


void Multiply_q(const double p[],const double q[],double res[]){
    res[0] = p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3];
    res[1] = p[0]*q[1] + p[1]*q[0] + p[2]*q[3] - p[3]*q[2];
    res[2] = p[0]*q[2] + p[2]*q[0] + p[3]*q[1] - p[1]*q[3];
    res[3] = p[0]*q[3] + p[3]*q[0] + p[1]*q[2] - p[2]*q[1];
}

void Angle2RV(const double angle1[],const double angle2[],double RV[]){
    RV[0] = angle2[0] + (-angle1[2] * angle2[1] + angle1[1] * angle2[2]) / 12;
    RV[1] = angle2[1] + (angle1[2] * angle2[0] - angle1[0] * angle2[2]) / 12;
    RV[2] = angle2[2] + (-angle1[1] * angle2[0] + angle1[0] * angle2[1]) / 12;
}





