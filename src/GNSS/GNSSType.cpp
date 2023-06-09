//
// Created by 0-0 mashuo on 2023/6/4.
//


#include "GNSSType.h"


GnssRes::GnssRes() = default;

GnssRes::GnssRes(GPST &gpst, double *blh, double *blh_std, double *v_ned, double *v_ned_std, bool valid) {
    m_gpst = gpst;
    memcpy(m_pBLH,blh,3*sizeof(double));
    memcpy(m_pBLHStd,blh_std,3*sizeof(double));
    memcpy(m_pVned,v_ned,3*sizeof(double));
    memcpy(m_pVnedStd,v_ned_std,3*sizeof(double));
    m_isvalid = valid;
}
