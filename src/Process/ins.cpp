//
// Created by 0-0 mashuo on 2023/6/10.
//

#include "TerminalMessage.h"
#include "INSType.h"
#include "cfileSaver.h"
#include "cImuDataReader.h"
#include "Rotation.h"

// 纯惯导程序

int main(int argc,char *argv[]){
    /*if(argc !=2){
        TerminalMessage::displayErrorMessage("参数数目应为两个!");
        return -1;
    }*/
    // 读取配置文件
    YAML::Node config;
    try{
        config = YAML::LoadFile("/Users/0-0mashuo/Desktop/Clion/CombinedNavigation/config.yaml");
    } catch (const std::exception & e) {
        TerminalMessage::displayErrorMessage("读取配置文件失败!请检查配置文件路径及格式");
    }
    // 路径获取
    std::string inPath , outPath;
    int imuFormat, imu_encode;
    try{
        inPath = config["imupath"].as<std::string>();
        outPath = config["outputpath"].as<std::string>();
        imuFormat = config["imufileformat"].as<int>();
    }catch (const std::exception & e){
        TerminalMessage::displayErrorMessage("读取配置路径失败!请检查配置路径字段");
    }
    // 打开文件
    switch (imuFormat) {
        case 22:
            imu_encode = cFileBase::ASCIITYPE;
            break;
        case 33:
            imu_encode = cFileBase::BINARYTYPE;
            break;
        default:
            TerminalMessage::displayErrorMessage("imu文件格式设置错误!");
            return -1;
    }
    cImuDataReader imuReader(inPath,imu_encode,imuFormat);
    cfileSaver resWriter(outPath+"/pureIns.txt");

    INSRes_SingleEpoch res0;
    IMUData_SingleEpoch imuData;
    // 获得处理时间
    double startTime,endTime;
    try {
        startTime = config["starttime"].as<double>();
        endTime = config["endtime"].as<double>();
    }catch (const std::exception & e){
        TerminalMessage::displayErrorMessage("配置文件处理时间段设置字段错误!无法读取");
        return -1;
    }

    imuReader.readline(imuData);
    if(endTime > 604800 || (startTime > endTime && endTime != -1) || startTime < imuData.t.second){
        TerminalMessage::displayErrorMessage("处理时间端设置错误!请重新设置");
    }
    // 时间对齐
    while(startTime > imuData.t.second){
        imuReader.readline(imuData);
    }
    // 获得起始信息
    try{
        memcpy(res0.m_pPos,config["initpos"].as<std::vector<double>>().data(),3*sizeof(double));
        memcpy(res0.m_pSpeed,config["initvel"].as<std::vector<double>>().data(),3*sizeof(double));
        memcpy(res0.m_pEuler,config["initatt"].as<std::vector<double>>().data(),3*sizeof(double));
    }catch (const std::exception & e){
        TerminalMessage::displayErrorMessage("起始位置、姿态、速度字段无法获取!");
    }
    // 改变单位，便于计算
    res0.changeUnitD2R();
    Rotation::Euler2Quaternion(res0.m_pEuler,res0.m_pQuaternion);
    Rotation::Euler2EMatrix(res0.m_pEuler,res0.m_pEMatrix);
    std::vector<IMUData_SingleEpoch> data(3);
    std::vector<INSRes_SingleEpoch> res(3);
    data.assign(3,imuData);
    res0.t = imuData.t;
    res.assign(3,res0);
    std::vector<double> v(11);
    while(true){
        // 文件到达结尾或者处理时间段完成
        if(imuReader.is_eof() || imuData.t.second > endTime){
            break;
        }
        imuReader.readline(imuData);
        data[0] = data[1], data[1] = data[2], data[2] = imuData;
        res[0] = res[1], res[1] = res[2];     res[2].t = imuData.t;
        // 机械编排算法更新
        PureIns::updateSinEpoch(data[0],data[1],data[2],
                                res[0],res[1],res[2]);
        // 改变单位，方便输出
        res[2].changeUnitR2D();
        v = res[2].toVector();
        resWriter.write(v);
        // 输出后，需将单位转回来
        res[2].changeUnitD2R();

    }
    TerminalMessage::displaySuccessMessage("文件处理完成!");
}


