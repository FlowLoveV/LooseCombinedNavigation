//
// Created by 0-0 mashuo on 2023/5/13.
//

#ifndef COMBINEDNAVIGATION_BASICFUNS_H
#define COMBINEDNAVIGATION_BASICFUNS_H

#include <cmath>
#include "INSType.h"

// 常量的定义
extern double ELLIPSOID_a;
extern double ELLIPSOID_b;
extern double ELLIPSOID_OMEGA;
extern double ELLIPSOID_GM;
extern double ELLIPSOID_E2;



/*!
 * 线型外推半历元函数
 * @param v_2   input       double          k-2历元时刻值
 * @param v_1   input       double          k-1历元时刻值
 * @return                  double          k-0.5历元时刻值
 */
inline double linExtrapolateHalf(const double & v_2,const double & v_1) {return 1.5*v_2 - 0.5*v_1;}

/*!
 * 求出三维向量模的函数
 * @param v     input       double[3]       输入数组
 * @return                  double          输入三维向量数组的模值
 */
inline double array_norm(const double v[]) {return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);}




/*!
 * 四元数相乘 p * q = res
 * @param p     input       double[3]       左四元数
 * @param q     input       double[3]       右四元数
 * @param res   output      double[3]       res = p * q
 */
void Multiply_q(const double p[],const double q[],double res[]);




/*!
 * 根据k-1历元、k历元角增量得到k-1历元到k历元的转等效旋转矢量 tested
 * @param angle1    input       double[3]       k-1历元角增量 [x,y,z] rad
 * @param angle2    input       double[3]       k历元角增量   [x,y,z] rad
 * @param RV        output      double[3]       k-1历元到k历元的等效旋转矢量
 */
void Angle2RV(const double angle1[],const double angle2[],double RV[]);


































#endif //COMBINEDNAVIGATION_BASICFUNS_H
