//
// Created by 0-0 mashuo on 2023/5/13.
//



#include "INSSolution.h"


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

void RV2Quaternion(const double RV[],double q[]){
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

void calculateRotationSpeed(const double pos[],const double v[],double wie[],double wen[],double & Rm,double & Rn){
    double slat = sin(pos[0]), clat = cos(pos[0]), tlat = tan(pos[0]);
    // 输出wie
    wie[0] = ELLIPSOID_OMEGA * clat;
    wie[1] = 0;
    wie[2] = -ELLIPSOID_OMEGA * slat;
    // 输出Rm,Rn
    double t = sqrt(1 - ELLIPSOID_E2 * pow(slat,2));
    Rm = ELLIPSOID_a * (1 - ELLIPSOID_E2) / pow(t,3);
    Rn = ELLIPSOID_a / t;
    // 输出wen
    wen[0] = v[1] / (Rn + pos[2]);
    wen[1] = -v[0] / (Rm + pos[2]);
    wen[2] = -v[1] * tlat / (Rn + pos[2]);
}

void EMatrix2Euler(const double Cnb[],double Euler[]){
    Euler[0] = atan2(Cnb[7],Cnb[8]);
    Euler[1] = atan(-Cnb[6] / sqrt( pow(Cnb[7],2) + pow(Cnb[8],2) ));
    Euler[2] = atan2(Cnb[3],Cnb[0]);
    // 将横滚角范围限定在0-360°
    Euler[2] = Euler[2] < 0 ? Euler[2] + 2*PI : Euler[2];

}

void Euler2Quaternion(const double Euler[],double q[]){
    double c1 = cos(Euler[0]/2), s1 = sin(Euler[0]/2);
    double c2 = cos(Euler[1]/2), s2 = sin(Euler[1]/2);
    double c3 = cos(Euler[2]/2), s3 = sin(Euler[2]/2);
    q[0] = c1*c2*c3 + s1*s2*s3;
    q[1] = s1*c2*c3 - c1*s2*s3;
    q[2] = c1*s2*c3 + s1*c2*s3;
    q[3] = c1*c2*s3 - s1*s2*c3;
}

void Euler2EMatrix(const double Euler[],double C[]){
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

void Quaternion2EMatrix(const double q[],double C[]){
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

double calculate_g(const double & lat, const double & h){
    double slat2 = pow(sin(lat) , 2);
    double g0 = 9.7803267715 * ( 1 + 0.0052790414 * slat2 + 0.0000232718 * pow(slat2,2) );
    return g0 - (3.087691089 * 1e-6 - 4.397731 * 1e-9 * slat2) * h + 0.721 * 1e-12 * pow(h,2);
}




