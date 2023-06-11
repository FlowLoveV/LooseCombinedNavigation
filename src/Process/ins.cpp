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
    if(argc !=2){
        TerminalMessage::displayErrorMessage("参数数目应为两个!");
        return -1;
    }
    // 读取配置文件
    YAML::Node config;
    try{
        config = YAML::LoadFile(argv[1]);
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
        case 44:
            imu_encode = cFileBase::ASCIITYPE;
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
    // 输出配置信息
    std::cout << "INS机械编排解算信息如下:" << std::endl;
    std::cout << "IMU数据文件:" << inPath << std::endl;
    std::cout << "解算配置信息:" << std::endl;
    std::cout << "\t" << "解算起始历元 : " << std::setprecision(9) << startTime << std::endl;
    std::cout << "\t" << "解算终止历元 : " << std::setprecision(9) << endTime << std::endl;
    std::cout << "\t" << "起始位置 : " << std::setprecision(14) << res0.m_pPos[0] << "deg , " << res0.m_pPos[1] << "deg , " << res0.m_pPos[2] << "m ——[lat lon H]\n";
    std::cout << "\t" << "起始速度 : " << res0.m_pSpeed[0] << "m/s , "
    << res0.m_pSpeed[1] << "m/s , " << res0.m_pSpeed[2] << "m/s ——[North East Down]\n";
    std::cout << "\t" << "起始姿态 : " << res0.m_pEuler[0] << "deg , " << res0.m_pEuler[1] << "deg , " <<
    res0.m_pEuler[2] << "deg ——[Roll Pitch Yaw]\n";
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

    double TotalTime = endTime - startTime;
    int thisPercent = 0 , lastPercent = 0;
    std::cout << std::endl;
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

        thisPercent = int((imuData.t.second - startTime) / TotalTime * 100);

        if (thisPercent - lastPercent >= 1) {
            std::cout << "当前处理进度: " << std::setw(3) << thisPercent << "%\r" << std::flush;
            lastPercent = thisPercent;
        }

    }
    TerminalMessage::displaySuccessMessage("\n处理完成!\n");
    TerminalMessage::displaySuccessMessage("结果文件保存在"+outPath+"pureins.txt");
    std::string formatMessage = "结果文件格式如下:\nGPST-week\tGPST-second(s)\tLatitude(deg)\tLongitude(deg)\tHeight(m)"
                                "\tV-north(m/s)\tV-east(m/s)\tV-down(m/s)\tRoll(deg)\tPitch(deg)\tYaw(deg)\n";
    TerminalMessage::displaySuccessMessage(formatMessage);
}


