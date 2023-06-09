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
    double m_pBLH[3]{};
    double m_pBLHStd[3]{};
    double m_pVned[3]{};
    double m_pVnedStd[3]{};
    bool m_isvalid = true;

    GnssRes();

    GnssRes(GPST &gpst,double *blh,double *blh_std,double *v_ned,double *v_ned_std,bool valid = true);


} ;



#endif //COMBINEDNAVIGATION_GNSSTYPE_H
