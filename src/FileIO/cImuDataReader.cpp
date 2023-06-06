//
// Created by 0-0 mashuo on 2023/6/6.
//

#include "cImuDataReader.h"

cImuDataReader::cImuDataReader()  = default;


cImuDataReader::cImuDataReader(const std::string &filename, const int &filetype, const int & fileformat) {
    if(!open(filename,filetype)){
        std::cerr << "can't open file : " << filename;
    }
    m_ifileType = filetype;
    m_sfileName = filename;
    m_ifileFormat = fileformat;
}

bool cImuDataReader::readline(IMUData_SingleEpoch &date) {
    switch (m_ifileFormat) {
        // 默认格式
        case NOVATEL_OME7_IMU_ACSII:
            read_NOVATEL_OME7_IMU_ACSII_line();
            break;
        // 其他格式...

        default:
            std::cerr << "无输入类型的文本，请选择正确类型的文本!\n";
            break;
    }
    return empty;
}

void cImuDataReader::read_NOVATEL_OME7_IMU_ACSII_line() {

}

