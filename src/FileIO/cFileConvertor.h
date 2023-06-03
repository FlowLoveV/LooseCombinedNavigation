//
// Created by 0-0 mashuo on 2023/6/3.
//

#ifndef COMBINEDNAVIGATION_CFILECONVERTOR_H
#define COMBINEDNAVIGATION_CFILECONVERTOR_H

#include "INSData.h"
#include "GNSS_Type.h"

// 文件转换器
class cFileConvertor{
public:
    static IMUData_SingleEpoch toImuData(std::vector<double> & vec);

    static tagGnssRes toGnssResData(std::vector<double> & vec);

protected:



};

#endif //COMBINEDNAVIGATION_CFILECONVERTOR_H
