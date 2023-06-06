//
// Created by 0-0 mashuo on 2023/6/3.
//

#ifndef COMBINEDNAVIGATION_CFILECONVERTOR_H
#define COMBINEDNAVIGATION_CFILECONVERTOR_H

#include "INSType.h"
#include "GNSSType.h"

// 文件转换器
class cFileConvertor{
public:
    /*!
     * 根据读取得到的向量，赋值得到一个单历元IMU原始观测数据
     * @param vec
     * @return
     */
    static IMUData_SingleEpoch toImuData(std::vector<double> & vec);

    static GnssRes toGnssResData(std::vector<double> & vec);

protected:



};

#endif //COMBINEDNAVIGATION_CFILECONVERTOR_H
