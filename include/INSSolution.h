//
// Created by 0-0 mashuo on 2023/5/13.
//

#ifndef COMBINEDNAVIGATION_INSSOLUTION_H
#define COMBINEDNAVIGATION_INSSOLUTION_H

#include <cmath>
#include "INSData.h"

// 常量的定义
extern double ELLIPSOID_a;
extern double ELLIPSOID_b;
extern double ELLIPSOID_OMEGA;
extern double ELLIPSOID_GM;
extern double ELLIPSOID_E2;


#define MY_PROJECT_PI 3.141592653589793
#define RAD2DEG 57.295779513082323
#define DEG2RAD 0.017453292519943

/*!
 * 弧度转角度函数
 * @param rad   input       double      弧度[rad]
 * @return                  double      角度[百分度]
 */
inline double rad2deg(const double & rad){ return RAD2DEG * rad;}

/*!
 * 角度转弧度函数
 * @param deg   input       double      角度[百分度]
 * @return                  double      弧度[rad]
 */
inline double deg2rad(const double & deg){ return DEG2RAD * deg;}

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




/*!
 * 等效旋转矢量转为姿态四元数 tested
 * @param RV    input       double[3]       k-1历元到k历元载体的等效旋转矢量 rad
 * @param q     output      double[4]       姿态四元数[q0,q1,q2,q3]
 */
void RV2Quaternion(const double RV[],double q[]);




/*!
 * 根据位置、速度计算 e系相对于i系旋转角速度wie,以及n系相对于e系旋转角速度wen(在n系下的投影)  tested
 * @param pos   input       double[3]       [lat,lon,h] rad m
 * @param v     input       double[3]       [vn,ve,vd] m/s
 * @param wie   output      double[3]       e系相对于i系旋转角速度wie rad/s
 * @param wen   output      double[3]       n系相对于e系旋转角速度wen rad/s
 * @param Rm    output      double          该位置子午圈半径[m]
 * @param Rn    output      double          该位置卯酉圈半径[m]
 */
void calculateRotationSpeed(const double pos[],const double v[],double wie[],double wen[],double & Rm,double & Rn);




/*!
 * 姿态矩阵转为欧拉角  tested
 * @param Cnb   input       double[9]     姿态矩阵，按行排列[c11,c12,c13,c21,c22,c23,c31,c32,c33]
 * @param Euler output      double[3]     欧拉角组[roll,pitch,yaw](横滚、俯仰，航向) [rad]
 */
void EMatrix2Euler(const double Cnb[],double Euler[]);




/*!
 * 欧拉家组转姿态矩阵
 * @param Euler input       double[3]     欧拉角组[roll,pitch,yaw](横滚、俯仰，航向) [rad]
 * @param C     output      double[9]     姿态矩阵，按行排列[c11,c12,c13,c21,c22,c23,c31,c32,c33]
 */
void Euler2EMatrix(const double Euler[],double C[]);




/*!
 * 欧拉角组转姿态四元数函数
 * @param Euler input       double[3]     欧拉角组[roll,pitch,yaw](横滚、俯仰，航向) [rad]
 * @param q     output      double[4]     姿态四元数[q0,q1,q2,q3]
 */
void Euler2Quaternion(const double Euler[],double q[]);



/*!
 * 姿态四元数转姿态矩阵函数
 * @param q input       double[4]     姿态四元数[q0,q1,q2,a3]
 * @param C output      double[9]     姿态矩阵，按行排列[c11,c12,c13,c21,c22,c23,c31,c32,c33]
 */
void Quaternion2EMatrix(const double q[],double C[]);



/*!
 * GRS80 地球椭球模型正常重力的计算  tested
 * @param lat input     double      纬度[rad]
 * @param h   input     double      高程[m]
 * @return              double      该纬度、高程下的正常重力值g[m/s/s]
 */
double calculate_g(const double & lat, const double & h);






















#endif //COMBINEDNAVIGATION_INSSOLUTION_H
