//
// Created by 0-0 mashuo on 2023/6/6.
//

#include "cImuDataReader.h"
#include "cFileConvertor.h"

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
            read_NOVATEL_OME7_IMU_ACSII_line(date);
            break;
        // 其他格式...
        case IMR_BINARY:
            std::cerr << "程序目前尚未添加解码imr文件的功能!";
            break;
        case LINE_TXT_ASCII:
            // 行排列的
            try{
                date = cFileConvertor::toImuData(cfileReader::readline(7));
            }
            catch (const std::exception & e){

            }
            break;
        default:
            std::cerr << "无输入类型的文本，请选择正确类型的文本!\n";
            break;
    }
    return !empty;
}

void cImuDataReader::read_NOVATEL_OME7_IMU_ACSII_line(IMUData_SingleEpoch & data) {
    if(is_eof()) {
        empty = true;
    }else{
        ::std::string line;
        char delimiter[] = ",;*";
        ::std::getline(m_fileFp,line);   // 按行读取
        if(line.find(HEADSTRING) != ::std::string::npos){
            char* str = new char[line.length()+1];
            ::std::strcpy(str,line.c_str());
            // 解码数据
            char* token = strtok(str, delimiter);
            token = strtok(NULL,delimiter);
            token = strtok(NULL,delimiter);
            token = strtok(NULL,delimiter);      data.t.weeks = ::std::atof(token);
            token = strtok(NULL,delimiter);      data.t.second = ::std::atof(token);
            token = strtok(NULL,delimiter);
            token = strtok(NULL,delimiter);      data.m_pAcc[2] = - ::std::atof(token) * NOVATEL_ACC_SCALE;     // +z-U -> +z-D
            token = strtok(NULL,delimiter);      data.m_pAcc[0] = - ::std::atof(token) * NOVATEL_ACC_SCALE;     // -y-N -> +x->N
            token = strtok(NULL,delimiter);      data.m_pAcc[1] = ::std::atof(token) * NOVATEL_ACC_SCALE;       // +x-E -> +y->E
            token = strtok(NULL,delimiter);      data.m_pGyr[2] = - ::std::atof(token) * NOVATEL_GYR_SCALE;
            token = strtok(NULL,delimiter);      data.m_pGyr[0] = - ::std::atof(token) * NOVATEL_GYR_SCALE;
            token = strtok(NULL,delimiter);      data.m_pGyr[1] = ::std::atof(token) * NOVATEL_GYR_SCALE;
            delete []str;
            empty = false;
            m_imuData = data;
        }else{
            empty = true;
        }
    }
}

