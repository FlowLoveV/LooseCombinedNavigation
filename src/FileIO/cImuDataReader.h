//
// Created by 0-0 mashuo on 2023/6/6.
//

#ifndef COMBINEDNAVIGATION_CIMUDATAREADER_H
#define COMBINEDNAVIGATION_CIMUDATAREADER_H

#include "cfileReader.h"
#include "INSType.h"

/**< NOVATEL格式的imu数据文件           */
#define NOVATEL_OME7_IMU_ACSII 22
#define IMR_BINARY             33
#define LINE_TXT_ASCII         44

class cImuDataReader : public cfileReader{
public:
    using cfileReader::open;

    /*!
     * 默认构造函数
     */
    cImuDataReader();

    /*!
     * 根据文件名，文件编码格式构造一个IMU数据阅读器
     * 关于fileformat的其他格式，请查看源文件中的宏定义!
     * @param filename      input               std::string           文件名
     * @param filetype      input(optional)     int                   记录文件是ASCII还是BINARY，默认ASCII，如果要读取BINARY，请输入1
     * @param fileformat    input(optional)     int                   文件编码格式，默认为22，对应NOVATEL格式
     */
    explicit cImuDataReader(const std::string &filename,const int &filetype = cFileBase::ASCIITYPE,const int & fileformat = 22);


    /*!
     * 读取文件的一行数据，获得一个单历元的IMU数据
     * @param date          output              IMUData_SingleEpoch     单历元IMU数据
     * @return                                  bool                    是否成功获取数据
     */
    bool readline(IMUData_SingleEpoch & date);


protected:
    // 在此定义读取每种格式文件的函数
    virtual void read_NOVATEL_OME7_IMU_ACSII_line(IMUData_SingleEpoch & data);


    IMUData_SingleEpoch m_imuData;
    bool empty = true;
};


#endif //COMBINEDNAVIGATION_CIMUDATAREADER_H
