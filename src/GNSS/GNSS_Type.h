//
// Created by 0-0 mashuo on 2023/6/2.
//

#ifndef COMBINEDNAVIGATION_GNSS_TYPE_H
#define COMBINEDNAVIGATION_GNSS_TYPE_H

#include "TimeSys.h"
#include "CoorSys.h"

// 该文件用于定义与GNSS数据相关的类

typedef struct {
    GPST m_gpst;
    XYZ m_XYZ;
    double m_pXYZStd[3];
    double m_pV[3];
    double m_pVStd[3];
    bool isvalid;
} tagGnssRes;

#endif //COMBINEDNAVIGATION_GNSS_TYPE_H
