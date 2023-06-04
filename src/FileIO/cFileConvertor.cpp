//
// Created by 0-0 mashuo on 2023/6/3.
//

#include "cFileConvertor.h"


IMUData_SingleEpoch cFileConvertor::toImuData(std::vector<double> &vec) {
    GPST gpst(vec[0],vec[1]);
    double acc[3],gyr[3];
    memcpy(acc,&vec[2],IMUDATA_SIZE);
    memcpy(gyr,&vec[5],IMUDATA_SIZE);
    return {acc,gyr,gpst};
}

GnssRes cFileConvertor::toGnssResData(std::vector<double> &vec) {
    return GnssRes();
}
