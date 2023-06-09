//
// Created by 0-0 mashuo on 2023/6/3.
//

#include "cFileConvertor.h"


IMUData_SingleEpoch cFileConvertor::toImuData(const std::vector<double> &vec) {
    GPST gpst(vec[0],vec[1]);
    double acc[3],gyr[3];
    memcpy(acc,&vec[2],IMUDATA_SIZE);
    memcpy(gyr,&vec[5],IMUDATA_SIZE);
    return {acc,gyr,gpst};
}

GnssRes cFileConvertor::toGnssResData(const std::vector<double> &vec) {
    int len = vec.size();
    GnssRes res;
    switch (len) {
        case 8:
            res.m_gpst.weeks = static_cast<unsigned short>(vec[0]);
            res.m_gpst.second = vec[1];
            memcpy(res.m_pBLH,&vec[2],3*sizeof(double));
            memcpy(res.m_pBLHStd,&vec[5],3*sizeof(double));
            break;
        case 14:
            res.m_gpst.weeks = vec[0];
            res.m_gpst.second = vec[1];
            memcpy(res.m_pBLH,&vec[2],3*sizeof(double));
            memcpy(res.m_pBLHStd,&vec[5],3*sizeof(double));
            memcpy(res.m_pVned,&vec[8],3*sizeof(double));
            memcpy(res.m_pVnedStd,&vec[11],3*sizeof(double));
            break;
        default:
            std::cerr << "error in cFileConvertor::toGnssResData(const std::vector<double> &vec)\n"
            << "输入数组的大小应该为7或者13!";
    }
    return res;
}
