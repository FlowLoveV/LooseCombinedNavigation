//
// Created by 0-0 mashuo on 2023/6/9.
//

/* 本文件用于进行kalman滤波的松组合处理
 *
 */

#include "LooseCombination.h"
#include "TimeSys.h"
#include "TerminalMessage/TerminalMessage.h"
#include "cImuDataReader.h"
#include "cfileReader.h"
#include "cfileSaver.h"
#include "cFileConvertor.h"


int main(int argc, char *argv[]){
    // 参数限定
    if(argc !=2){
        TerminalMessage::displayErrorMessage("参数数目应为两个!");
        return -1;
    }
    // 获得时间
    auto timeBegin = getCurrentTime();
    auto timeStyle = "%Y-%m-%d %H:%M:%S";
    // 读取配置文件
    YAML::Node config;
    try {
        config = YAML::LoadFile(argv[1]);
    }catch (const std::exception & e){
        TerminalMessage::displayErrorMessage("读取配置文件失败!请检查文件路径和配置文件格式.");
        return -1;
    }

    // 读取文件路径配置
    std::string imupath, gnsspath, outputpath;
    int imu_file_format,gnss_file_format,imu_encode,gnss_encode;
    try {
        imupath    = config["imupath"].as<std::string>();
        gnsspath   = config["gnsspath"].as<std::string>();
        outputpath = config["outputpath"].as<std::string>();
        imu_file_format = config["imufileformat"].as<int>();
        gnss_file_format = config["gnssfileformat"].as<int>();
    } catch (YAML::Exception &exception) {
        std::cout << "读取配置文件失败!请检查配置数据路径配置是否正确" << std::endl;
        return -1;
    }
    // 根据imu文件格式选择解码方式
    switch (imu_file_format) {
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
    // 根据gnss文件格式选择解码方式
    switch (gnss_file_format) {
        case 1:
            gnss_encode = cFileBase::ASCIITYPE;
            break;
        default:
            TerminalMessage::displayErrorMessage("gnss文件格式设置错误!");
            return -1;
    }

    // 打开文件、创建输出文件
    cImuDataReader imuReader(imupath,imu_encode,imu_file_format);
    cfileReader gnssReader(gnsspath,gnss_encode);
    std::string navpath = outputpath + "/LCNavRes.txt";
    std::string inserrorpath = outputpath + "/InsRes.txt";
    std::string navStdpath = outputpath + "/LCNavStd.txt";
    cfileSaver navSaver(navpath),insSaver(inserrorpath),navStdSaver(navStdpath);
    if(!imuReader.is_open()) {
        TerminalMessage::displayErrorMessage("imu数据文件路径设置错误!无法打开文件");
        return -1;
    }
    if(!gnssReader.is_open()) {
        TerminalMessage::displayErrorMessage("gnss数据文件路径设置错误!无法打开文件");
        return -1;
    }
    if(!navSaver.is_open() || !navStdSaver.is_open() || !insSaver.is_open()){
        TerminalMessage::displayErrorMessage("文件输出路径设置错误!无法创建结果文件");
        return -1;
    }

    // 读取处理时间段
    double startTime,endTime;
    try {
        startTime = config["starttime"].as<double>();
        endTime = config["endtime"].as<double>();
    }catch (const std::exception & e){
        TerminalMessage::displayErrorMessage("处理时间段设置错误!请重新设置");
        return -1;
    }

    // 读取文件
    IMUData_SingleEpoch imudata;
    GnssRes gnssRes;
    imuReader.readline(imudata);
    // 结束时间大于一个周的秒数 或者 开始时间大于结束时间 或者 开始时间小于imu第一历元时间
    // 都会报错，无法处理
    if(endTime > 604800 || (startTime > endTime && endTime != -1) || startTime < imudata.t.second){
        TerminalMessage::displayErrorMessage("处理时间端设置错误!请重新设置");
    }

    // 时间对齐
    while(startTime > imudata.t.second){
        imuReader.readline(imudata);
    }
    do{
        auto vec = gnssReader.readline(14);
        // 读取到非格式化的行，则重新读取下一行（一般在文件开头可能会遇到）
        if(vec.empty()) continue;
        gnssRes = cFileConvertor::toGnssResData(vec);
    }while(gnssRes.m_gpst.second <= startTime);

    // 生成LC处理器
    using namespace ns_GINS;
    LooseCombination LC(config);
    // 前两历元数据填充
    LC.addImuData(imudata);
    LC.addImuData(imudata);
    // gnss数据填充
    LC.addGnssResData(gnssRes);

    // 处理结果
    GPST resTime;
    NavState imuState{};
    Matrix imuStateStd;
    // vector暂存结果
    std::vector<double> vecImuState(23);
    std::vector<double> vecImuStateStd(23);

    // 显示处理进程
    double TotalTime = endTime - startTime;
    int thisPercent = 0 , lastPercent = 0;

    while(1){
        // 当gnss时间落后当前IMU历元时，需要读取
        if(gnssRes.m_gpst.second < imudata.t.second && !gnssReader.is_eof()){
            gnssRes = cFileConvertor::toGnssResData(gnssReader.readline(14));
            LC.addGnssResData(gnssRes);
        }

        // 读取并添加新的imu数据
        imuReader.readline(imudata);
        if(imuReader.is_eof() || imudata.t.second > endTime){
            break;
        }
        LC.addImuData(imudata);
        LC.newProcess();

        // 得到处理结果
        resTime = LC.getTime();
        imuState = LC.getNavState();
        imuStateStd = LC.getStateVariance();

        if(!imuStateStd.checkDiagPositive()){
            TerminalMessage::displayErrorMessage("运行过程中状态方差阵非正定!");
            std::exit(EXIT_FAILURE);
        }

        // 将结果写入文件
        vecImuState = imuState.toVector(resTime);
        navSaver.write(vecImuState);

        thisPercent = int((imudata.t.second - startTime) / TotalTime * 100);
        if (thisPercent - lastPercent >= 1) {
            std::cout << "当前处理进度: " << std::setw(3) << thisPercent << "%\r" << std::flush;
            lastPercent = thisPercent;
        }
    }

    imuReader.close();
    gnssReader.close();
    navSaver.close();
    navStdSaver.close();
    insSaver.close();

    // 处理时间段显示
    auto timeEnd = getCurrentTime();
    auto t2 = std::put_time(&timeEnd,timeStyle);
    char buff1[80],buff2[80];
    strftime(buff1,80,timeStyle,&timeBegin);
    strftime(buff2,80,timeStyle,&timeEnd);
    TerminalMessage::displaySuccessMessage("处理时间 : " + std::string(buff1) + "——————" + std::string(buff2));
    TerminalMessage::displaySuccessMessage("程序处理成功!");
    TerminalMessage::displaySuccessMessage("结果文件已保存在"+outputpath+"下");

}

