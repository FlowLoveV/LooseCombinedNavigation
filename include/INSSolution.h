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


#define PI 3.141592653589793
#define RAD2DEG 57.295779513082323
#define DEG2RAD 0.017453292519943

// 常用小函数 -- 内联

// 角度弧度相互转换
inline double rad2deg(const double & rad){ return RAD2DEG * rad;}
inline double deg2rad(const double & deg){ return DEG2RAD * deg;}
// 线型外推半历元函数
inline double linExtrapolateHalf(const double & v_2,const double & v_1) {return 1.5*v_2 - 0.5*v_1;}
// 求模函数
inline double array_norm(const double v[]) {return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);}



// 四元数相乘
/* res = p * q   tested
 */
void Multiply_q(const double p[],const double q[],double res[]);



// 角增量转等效旋转矢量 tested
/* input1 - angle1 : k-1历元角增量 [x,y,z] rad
 * input2 - angle2 : k历元角增量   [x,y,z] rad
 * output1 - RV     : k-1历元到k历元载体的等效旋转矢量 rad
 */
void Angle2RV(const double angle1[],const double angle2[],double RV[]);



// 等效旋转矢量转为姿态四元数 tested
/* input - RV : k-1历元到k历元载体的等效旋转矢量 rad
 * output - q :  q = qv + qs
 *               qv = cos(||0.5RV||)
 *               qs = sin(||0.5RV||)/||0.5RV|| · 0.5RV
 */
void RV2Quaternion(const double RV[],double q[]);



// 根据位置、速度计算 e系相对于i系旋转角速度wie,以及n系相对于e系旋转角速度wen(在n系下的投影)  test
/* input1 - pos : 第k-1历元或者外推得到第k-0.5历元(视输入而定，可自己选择精度)时刻载体位置 [lat,lon,h] rad m
 * input2 - v :  同上，速度   [vn,ve,vd] m/s
 * output1 - wie : e系相对于i系旋转角速度wie rad/s
 * output2 - wen : n系相对于e系旋转角速度wen rad/s
 * output3 - Rm
 * output4 - Rn
 */
void calculateRotationSpeed(const double pos[],const double v[],double wie[],double wen[],double & Rm,double & Rn);



// 姿态矩阵转为欧拉角  tested
/* input1 - Cnb : Xn = Cnb * Xb ,b系转n系的姿态矩阵,按行排列 9*1 [c11,c12,c13,c21,c22,c23,c31,c32,c33]
 * output1 - Euler : 输入姿态矩阵对应的欧拉角组              3*1 [roll,pitch,yaw](横滚、俯仰，航向) rad
 */
void EMatrix2Euler(const double Cnb[],double Euler[]);



// 欧拉家转姿态矩阵 tested
/* input1 - Euler : 欧拉角组                              3*1 [roll,pitch,yaw](横滚、俯仰，航向) rad
 * output1 - C    : 输入欧拉角组对应的姿态矩阵,      按行排列 9*1 [c11,c12,c13,c21,c22,c23,c31,c32,c33]
 */
void Euler2EMatrix(const double Euler[],double C[]);



// 欧拉角转姿态四元数  tested
/* input1 - Euler : 欧拉角组  3*1 [roll,pitch,yaw](横滚、俯仰，航向) rad
 * output1 - q    : 输入欧拉角组对应的姿态四元数 4*1 [q0,q1,q2,q3]
 */
void Euler2Quaternion(const double Euler[],double q[]);



// 四元数转姿态矩阵  tested
/* input1 - q : 姿态四元数  4*1 [q0,q1,q2,q3]
 * output1 - C : 输入姿态四元数对应姿态矩阵,        按行排列 9*1 [c11,c12,c13,c21,c22,c23,c31,c32,c33]
 */
void Quaternion2EMatrix(const double q[],double C[]);



// GRS80 地球椭球模型正常重力的计算  tested
/* input1 - lat : 纬度[rad]
 * input2 - h   : 高程[m] 正常重力 g[m/s/s]
 */
double calculate_g(const double & lat, const double & h);






















#endif //COMBINEDNAVIGATION_INSSOLUTION_H
