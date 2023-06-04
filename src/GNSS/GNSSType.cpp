//
// Created by 0-0 mashuo on 2023/6/4.
//


#include "GNSSType.h"


GnssRes::GnssRes() = default;

GnssRes::GnssRes(GPST &gpst, double *xyz, double *xyz_std, double *v_xyz, double *v_xyz_std, bool valid) {
    m_gpst = gpst;
    m_XYZ  = XYZ(xyz);
    memcpy(m_pXYZStd,xyz_std,3*sizeof(double));
    memcpy(m_pV,v_xyz,3*sizeof(double));
    memcpy(m_pVStd,v_xyz_std,3*sizeof(double));
    m_isvalid = valid;
}
