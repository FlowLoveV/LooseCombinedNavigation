//
// Created by 0-0 mashuo on 2023/6/2.
//

#ifndef COMBINEDNAVIGATION_EARTH_H
#define COMBINEDNAVIGATION_EARTH_H

// 地球参数的配置-WGS84
const double WGS84_WIE = 7.2921151467E-5;       /* 地球自转角速度*/
const double WGS84_F   = 0.0033528106647474805; /* 扁率 */
const double WGS84_RA  = 6378137.0000000000;    /* 长半轴a */
const double WGS84_RB  = 6356752.3142451793;    /* 短半轴b */
const double WGS84_GM0 = 398600441800000.00;    /* 地球引力常数 */
const double WGS84_E1  = 0.0066943799901413156; /* 第一偏心率平方 */
const double WGS84_E2  = 0.0067394967422764341; /* 第二偏心率平方 */

class Earth{
public:
    /*!
     * 根据位置、速度计算 e系相对于i系旋转角速度wie,以及n系相对于e系旋转角速度wen(在n系下的投影)  tested
     * @param pos   input       double[3]       [lat,lon,h] rad m
     * @param v     input       double[3]       [vn,ve,vd] m/s
     * @param wie   output      double[3]       e系相对于i系旋转角速度wie rad/s
     * @param wen   output      double[3]       n系相对于e系旋转角速度wen rad/s
     * @param Rm    output      double          该位置子午圈半径[m]
     * @param Rn    output      double          该位置卯酉圈半径[m]
     */
    static void calculateRotationSpeed(const double pos[],const double v[],double wie[],double wen[],double & Rm,double & Rn){
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

    /*!
     * GRS80 地球椭球模型正常重力的计算  tested
     * @param lat input     double      纬度[rad]
     * @param h   input     double      高程[m]
     * @return              double      该纬度、高程下的正常重力值g[m/s/s]
     */
    static double calculate_g(const double & lat, const double & h){
        double slat2 = pow(sin(lat) , 2);
        double g0 = 9.7803267715 * ( 1 + 0.0052790414 * slat2 + 0.0000232718 * pow(slat2,2) );
        return g0 - (3.087691089 * 1e-6 - 4.397731 * 1e-9 * slat2) * h + 0.721 * 1e-12 * pow(h,2);
    }
};





#endif //COMBINEDNAVIGATION_EARTH_H
