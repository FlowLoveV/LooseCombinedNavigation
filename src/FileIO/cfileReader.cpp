//
// Created by 0-0 mashuo on 2023/6/1.
//

#include "cfileReader.h"


cfileReader::cfileReader(const std::string &filename, const int type, const std::string &format) {
    open(filename,type);
    fileFormat = format;
}

cfileReader::cfileReader() = default;

void cfileReader::deleteData() {
    if(!data.empty()) data.clear();
}

IMUData_SingleEpoch cfileReader::move() {
    IMUData_SingleEpoch imu(data);
    return imu;
}

std::vector<double> &cfileReader::readline() {
    // 根据fileformat读取文件

    return data;
}
