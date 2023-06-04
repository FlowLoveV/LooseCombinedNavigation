//
// Created by 0-0 mashuo on 2023/6/2.
//

#ifndef COMBINEDNAVIGATION_GNSSTYPE_H
#define COMBINEDNAVIGATION_GNSSTYPE_H

#include "TimeSys.h"
#include "CoorSys.h"

// 该文件用于定义与GNSS数据相关的类

class GnssRes{
public:
    GPST m_gpst;
    XYZ m_XYZ;
    double m_pXYZStd[3]{};
    double m_pV[3]{};
    double m_pVStd[3]{};
    bool m_isvalid = true;

    GnssRes();

    GnssRes(GPST &gpst,double *xyz,double *xyz_std,double *v_xyz,double *v_xyz_std,bool valid = true);


} ;



#endif //COMBINEDNAVIGATION_GNSSTYPE_H
