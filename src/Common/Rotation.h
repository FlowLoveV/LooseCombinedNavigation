//
// Created by 0-0 mashuo on 2023/6/2.
//

#ifndef COMBINEDNAVIGATION_ROTATION_H
#define COMBINEDNAVIGATION_ROTATION_H

#include "BasicFuns.h"
#include "Angle.h"


class Rotation {

public:
    /*!
    * 等效旋转矢量转为姿态四元数 tested
    * @param RV    input       double[3]       k-1历元到k历元载体的等效旋转矢量 rad
    * @param q     output      double[4]       姿态四元数[q0,q1,q2,q3]
    */
    static void RV2Quaternion(const double RV[],double q[]){
        double mod = array_norm(RV) / 2;
        // 容错处理
        if(mod==0) {q[0] = 1;q[1] = q[2] = q[3] = 0;}
        else{
            q[0] = cos(mod);
            double t = sin(mod) / mod / 2;
            q[1] = RV[0] * t;
            q[2] = RV[1] * t;
            q[3] = RV[2] * t;
        }
    }




    /*!
     * 姿态矩阵转为欧拉角  tested
     * @param Cnb   input       double[9]     姿态矩阵，按行排列[c11,c12,c13,c21,c22,c23,c31,c32,c33]
     * @param Euler output      double[3]     欧拉角组[roll,pitch,yaw](横滚、俯仰，航向) [rad]
     */
    static void EMatrix2Euler(const double Cnb[],double Euler[]){
        Euler[0] = atan2(Cnb[7],Cnb[8]);
        Euler[1] = atan(-Cnb[6] / sqrt( pow(Cnb[7],2) + pow(Cnb[8],2) ));
        Euler[2] = atan2(Cnb[3],Cnb[0]);
        // 将横滚角范围限定在0-360°
        Euler[2] = Euler[2] < 0 ? Euler[2] + 2 * MY_PROJECT_PI : Euler[2];

    }


    /*!
     * 欧拉角组转姿态矩阵
     * @param Euler input       double[3]     欧拉角组[roll,pitch,yaw](横滚、俯仰，航向) [rad]
     * @param C     output      double[9]     姿态矩阵，按行排列[c11,c12,c13,c21,c22,c23,c31,c32,c33]
     */
    static void Euler2EMatrix(const double Euler[],double C[]){
        double c1 = cos(Euler[0]), s1 = sin(Euler[0]);
        double c2 = cos(Euler[1]), s2 = sin(Euler[1]);
        double c3 = cos(Euler[2]), s3 = sin(Euler[2]);
        C[0] = c2 * c3;
        C[1] = -c1 * s3 + s1 * s2 * c3;
        C[2] = s1 * s3 + c1 * s2 * c3;
        C[3] = c2 * s3;
        C[4] = c1 * c3 + s1 * s2 * s3;
        C[5] = -s1 * c3 + c1 * s2 * s3;
        C[6] = -s2;
        C[7] = s1 * c2;
        C[8] = c1 * c2;
    }



    /*!
     * 欧拉角组转姿态四元数函数
     * @param Euler input       double[3]     欧拉角组[roll,pitch,yaw](横滚、俯仰，航向) [rad]
     * @param q     output      double[4]     姿态四元数[q0,q1,q2,q3]
     */
    static void Euler2Quaternion(const double Euler[],double q[]){
        double c1 = cos(Euler[0]/2), s1 = sin(Euler[0]/2);
        double c2 = cos(Euler[1]/2), s2 = sin(Euler[1]/2);
        double c3 = cos(Euler[2]/2), s3 = sin(Euler[2]/2);
        q[0] = c1*c2*c3 + s1*s2*s3;
        q[1] = s1*c2*c3 - c1*s2*s3;
        q[2] = c1*s2*c3 + s1*c2*s3;
        q[3] = c1*c2*s3 - s1*s2*c3;
    }



    /*!
     * 姿态四元数转姿态矩阵函数
     * @param q input       double[4]     姿态四元数[q0,q1,q2,a3]
     * @param C output      double[9]     姿态矩阵，按行排列[c11,c12,c13,c21,c22,c23,c31,c32,c33]
     */
    static void Quaternion2EMatrix(const double q[],double C[]){
        double q00 = q[0]*q[0], q01 = q[0]*q[1], q02 = q[0]*q[2], q03 = q[0]*q[3];
        double q11 = q[1]*q[1], q12 = q[1]*q[2], q13 = q[1]*q[3];
        double q22 = q[2]*q[2], q23 = q[2]*q[3];
        double q33 = q[3]*q[3];
        C[0] = q00 + q11 - q22 - q33;
        C[1] = 2*(q12 - q03);
        C[2] = 2*(q13 + q02);
        C[3] = 2*(q12 + q03);
        C[4] = q00 - q11 + q22 - q33;
        C[5] = 2*(q23 - q01);
        C[6] = 2*(q13 - q02);
        C[7] = 2*(q23 + q01);
        C[8] = q00 - q11 - q22 + q33;
    }


};

struct Attitude{
    double euler[3];
    double quaternion[4];
    double eMatrix[9];
};


#endif //COMBINEDNAVIGATION_ROTATION_H
