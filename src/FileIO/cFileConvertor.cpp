//
// Created by 0-0 mashuo on 2023/6/3.
//

#include "cFileConvertor.h"
#include "Angle.h"

IMUData_SingleEpoch cFileConvertor::toImuData(const std::vector<double> &vec) {
    int len = vec.size();
    GPST gpst;
    double acc[3],gyr[3];
    switch (len) {
        case 7:
            gpst.weeks = 0;
            gpst.second = vec[0];
            memcpy(gyr,&vec[1],3*sizeof(double));
            memcpy(acc,&vec[4],3*sizeof(double));
            break;
        case 8:
            gpst.weeks = vec[0];
            gpst.second = vec[1];
            memcpy(acc,&vec[2],IMUDATA_SIZE);
            memcpy(gyr,&vec[5],IMUDATA_SIZE);
            break;
        default:
            std::cerr << "error in cFileConvertor::toImuData(const std::vector<double> &vec)\n"
                      << "输入数组的大小应该为7或者8!";
    }
    return {acc,gyr,gpst};
}

GnssRes cFileConvertor::toGnssResData(const std::vector<double> &vec) {
    int len = vec.size();
    GnssRes res;
    switch (len) {
        case 7:
            res.m_gpst.weeks = 0;
            res.m_gpst.second = vec[0];
            memcpy(res.m_pBLH,&vec[1],3*sizeof(double));
            memcpy(res.m_pBLHStd,&vec[4],3*sizeof(double));
            break;
        case 8:
            res.m_gpst.weeks = static_cast<unsigned short>(vec[0]);
            res.m_gpst.second = vec[1];
            memcpy(res.m_pBLH,&vec[2],3*sizeof(double));
            memcpy(res.m_pBLHStd,&vec[5],3*sizeof(double));
            break;
        case 13:
            res.m_gpst.weeks = 0;
            res.m_gpst.second = vec[0];
            memcpy(res.m_pBLH,&vec[1],3*sizeof(double));
            memcpy(res.m_pBLHStd,&vec[4],3*sizeof(double));
            memcpy(res.m_pVned,&vec[7],3*sizeof(double));
            memcpy(res.m_pVnedStd,&vec[10],3*sizeof(double));
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
            << "输入数组的大小应该为6、7、13或者14!";
    }
    // 单位转换
    res.m_pBLH[0] *= DEG2RAD;
    res.m_pBLH[1] *= DEG2RAD;
    return res;
}
