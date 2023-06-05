//
// Created by 0-0 mashuo on 2023/6/5.
//

#include <iomanip>
#include "GINSType.h"
#include "Angle.h"

void GinsOptions::print(std::ostream &stream) {
    stream << "# --------------- GNSS/INS solution options ------------------- #\n";
    // 打印初始状态
    stream << "# Initial State : " << std::endl;
    stream << "\t" << "initial position [latitude longitude height]: [" << std::setprecision(12)
           << initState.pos[0] * RAD2DEG << ",";
    stream << std::setprecision(12) << initState.pos[1] * RAD2DEG << ",";
    stream << std::setprecision(6) << initState.pos[2] << "]  # [deg deg m]\n";
    stream << "\t" << "initial velocity [V-north V-east V-down]: [" << std::setprecision(6) << initState.vel[0]
           << "," << initState.vel[1] << "," << initState.vel[2] << "] # [m/s m/s m/s]\n";
    stream << "\t" << "initial attitude [Roll Pitch Yaw]: [" << std::setprecision(9) << initState.euler[0] * RAD2DEG
           << "," << initState.euler[1] * RAD2DEG << "," << initState.euler[2] * RAD2DEG << "] # [deg deg deg]\n";
    double t1 = RAD2DEG * 3600;
    stream << "\t" << "initial gyrbias [x y z]: [" << std::setprecision(6) << initState.imuError.gBias[0] * t1
           << "," << initState.imuError.gBias[1] * t1 << "," << initState.imuError.gBias[2] * t1 << "] # [deg/h deg/h deg/h]\n";
    stream << "\t" << "initial accbias [x y z]: [" << std::setprecision(6) << initState.imuError.aBias[0] * 1e5
           << "," << initState.imuError.aBias[1] * 1e5 << "," << initState.imuError.aBias[2] * 1e5 << "] # [mGal mGal mGal]\n";
    stream << "\t" << "initial gyrscale [x y z]: [" << std::setprecision(6) << initState.imuError.gScale[0] * 1e6
           << "," << initState.imuError.gScale[1] * 1e6<< "," << initState.imuError.gScale[2] * 1e6<< "] # [ppm ppm ppm]\n";
    stream << "\t" << "initial accscale [x y z]: [" << std::setprecision(6) << initState.imuError.aScale[0] * 1e6
           << "," << initState.imuError.aScale[1] * 1e6<< "," << initState.imuError.aScale[2] * 1e6<< "] # [ppm ppm ppm]\n";
    // 打印初始状态标准差
    stream << "# Initial State Std: " << std::endl;
    stream << "\t" << "initial position std[latitude longitude height]: [" << std::setprecision(6)
           << initStateStd.pos[0]  << ",";
    stream << std::setprecision(6) << initStateStd.pos[1]  << ",";
    stream << std::setprecision(6) << initStateStd.pos[2] << "]  # [deg deg m]\n";
    stream << "\t" << "initial velocity std[V-north V-east V-down]: [" << std::setprecision(6) << initStateStd.vel[0]
           << "," << initStateStd.vel[1] << "," << initStateStd.vel[2] << "] # [m/s m/s m/s]\n";
    stream << "\t" << "initial attitude std[Roll Pitch Yaw]: [" << std::setprecision(9) << initStateStd.euler[0] * RAD2DEG
           << "," << initStateStd.euler[1] * RAD2DEG << "," << initStateStd.euler[2] * RAD2DEG << "] # [deg deg deg]\n";
    stream << "\t" << "initial gyrbias std[x y z]: [" << std::setprecision(6) << initStateStd.imuError.gBias[0] * t1
           << "," << initStateStd.imuError.gBias[1] * t1 << "," << initStateStd.imuError.gBias[2] * t1 << "] # [deg/h deg/h deg/h]\n";
    stream << "\t" << "initial accbias std[x y z]: [" << std::setprecision(6) << initStateStd.imuError.aBias[0] * 1e5
           << "," << initStateStd.imuError.aBias[1] * 1e5 << "," << initStateStd.imuError.aBias[2] * 1e5 << "] # [mGal mGal mGal]\n";
    stream << "\t" << "initial gyrscale std[x y z]: [" << std::setprecision(6) << initStateStd.imuError.gScale[0] * 1e6
           << "," << initStateStd.imuError.gScale[1] * 1e6<< "," << initStateStd.imuError.gScale[2] * 1e6<< "] # [ppm ppm ppm]\n";
    stream << "\t" << "initial accscale std[x y z]: [" << std::setprecision(6) << initStateStd.imuError.aScale[0] * 1e6
           << "," << initStateStd.imuError.aScale[1] * 1e6<< "," << initStateStd.imuError.aScale[2] * 1e6<< "] # [ppm ppm ppm]\n";
    // 打印IMU噪声误差建模参数
    double t2 = RAD2DEG * 60;
    stream << "# IMU noise parameters" << std::endl;
    stream << '\t' << "arw [x y z]: " << imuNoise.gyrArw[0] * t2 << " " << imuNoise.gyrArw[1] * t2
           << " " << imuNoise.gyrArw[2] * t2 << " [deg/s/sqrt(h)] " << std::endl;
    stream << '\t' << "vrw [x y z]: " << imuNoise.accVrw[0] * 60 << " " << imuNoise.accVrw[1] * 60
           << " " << imuNoise.accVrw[2] * 60 << " [m/s/sqrt(h)] " << std::endl;
    stream << '\t' << "gyrbias std [x y z]: " << imuNoise.gBiasStd[0] * t1 << " " << imuNoise.gBiasStd[1] * t1
           << " " << imuNoise.gBiasStd[2] * t1<< " [deg/h] " << std::endl;
    stream << '\t' << "accbias  std [x y z]: " << imuNoise.aBiasStd[0] * 1e5 << " " << imuNoise.aBiasStd[1] * 1e5
           << " " << imuNoise.aBiasStd[2] * 1e5<< " [mGal] " << std::endl;
    stream << '\t' << "gyrscale std [x y z]: " << imuNoise.gScaleStd[0] * 1e6 << " " << imuNoise.gScaleStd[1] * 1e6
           << " " << imuNoise.gScaleStd[2] * 1e6 << " [ppm] " << std::endl;
    stream << '\t' << "accscale std [x y z]: " << imuNoise.aScaleStd[0] * 1e6 << " " << imuNoise.aScaleStd[1] * 1e6
           << " " << imuNoise.aScaleStd[2] * 1e6 << " [ppm] " << std::endl;
    stream << '\t' << "correlation time [gB aB gS aS]: " << imuNoise.corrTime[0] / 3600 << " "
           << imuNoise.corrTime[1] /3600 << " " << imuNoise.corrTime[2] / 3600<< " " << imuNoise.corrTime[3] /3600
           << " [h] " << std::endl;
    // 打印安装参数-天线杆臂
    stream << "# Antenna LeverArm [x y z]: " << AntennaLeverArm[0] << " " << AntennaLeverArm[1] << " "
           << AntennaLeverArm[2] << "[m]" << std::endl << std::endl;
}

GinsOptions::GinsOptions(const YAML::Node &config) {
    double t1 = RAD2DEG / 3600;
    // 初始状态初始化
    memcpy(initState.pos,config["initpos"].as<std::vector<double>>().data(),3*sizeof(double));
    memcpy(initState.vel,config["initvel"].as<std::vector<double>>().data(),3*sizeof(double));
    memcpy(initState.euler,config["initatt"].as<std::vector<double>>().data(),3*sizeof(double));
    memcpy(initState.imuError.gBias,config["initgyrbias"].as<std::vector<double>>().data(),3*sizeof(double));
    memcpy(initState.imuError.gScale,config["initgyrscale"].as<std::vector<double>>().data(),3*sizeof(double));
    memcpy(initState.imuError.aBias,config["initaccbias"].as<std::vector<double>>().data(),3*sizeof(double));
    memcpy(initState.imuError.aScale,config["initaccscale"].as<std::vector<double>>().data(),3*sizeof(double));
    // 初始状态方差初始化
    memcpy(initStateStd.pos,config["initposstd"].as<std::vector<double>>().data(),3*sizeof(double));
    memcpy(initStateStd.vel,config["initvelstd"].as<std::vector<double>>().data(),3*sizeof(double));
    memcpy(initStateStd.euler,config["initattstd"].as<std::vector<double>>().data(),3*sizeof(double));
    memcpy(initStateStd.imuError.gBias,config["initbgstd"].as<std::vector<double>>().data(),3*sizeof(double));
    memcpy(initStateStd.imuError.gScale,config["initsgstd"].as<std::vector<double>>().data(),3*sizeof(double));
    memcpy(initStateStd.imuError.aBias,config["initbastd"].as<std::vector<double>>().data(),3*sizeof(double));
    memcpy(initStateStd.imuError.aScale,config["initsastd"].as<std::vector<double>>().data(),3*sizeof(double));
    // IMU误差参数初始化
    memcpy(imuNoise.gyrArw,config["imunoise"]["arw"].as<std::vector<double>>().data(),3*sizeof(double));
    memcpy(imuNoise.accVrw,config["imunoise"]["vrw"].as<std::vector<double>>().data(),3*sizeof(double));
    memcpy(imuNoise.gScaleStd,config["imunoise"]["gsstd"].as<std::vector<double>>().data(),3*sizeof(double));
    memcpy(imuNoise.gBiasStd,config["imunoise"]["gbstd"].as<std::vector<double>>().data(),3*sizeof(double));
    memcpy(imuNoise.aBiasStd,config["imunoise"]["abstd"].as<std::vector<double>>().data(),3*sizeof(double));
    memcpy(imuNoise.aScaleStd,config["imunoise"]["asstd"].as<std::vector<double>>().data(),3*sizeof(double));
    memcpy(imuNoise.corrTime,config["imunoise"]["corrtime"].as<std::vector<double>>().data(),4*sizeof(double));
    // 安装参数
    memcpy(AntennaLeverArm,config["antlever"].as<std::vector<double>>().data(),3*sizeof(double));

    // 调整单位
    adjustUnit();
}

void GinsOptions::adjustUnit() {
    // 调整零偏系数单位
    double g_biasFactor = DEG2RAD / 3600.0;
    double a_biasFactor = 1e-5;
    for ( auto &it: initState.imuError.aBias) it *= a_biasFactor;
    for ( auto &it: initState.imuError.gBias) it *= g_biasFactor;
    for ( auto &it: initStateStd.imuError.aBias) it *= a_biasFactor;
    for ( auto &it: initStateStd.imuError.gBias) it *= g_biasFactor;
    for ( auto &it: imuNoise.aBiasStd) it *= a_biasFactor;
    for ( auto &it: imuNoise.gBiasStd) it *= g_biasFactor;
    // 调整比例因子系数单位
    double scaleFactor = 1e-6;
    for ( auto &it: initState.imuError.aScale) it *= scaleFactor;
    for ( auto &it: initState.imuError.gScale) it *= scaleFactor;
    for ( auto &it: initStateStd.imuError.aScale) it *= scaleFactor;
    for ( auto &it: initStateStd.imuError.gScale) it *= scaleFactor;
    for ( auto &it: imuNoise.aScaleStd) it *= scaleFactor;
    for ( auto &it: imuNoise.gScaleStd) it *= scaleFactor;
    // 调整IMU噪声单位
    double arwFactor = DEG2RAD / 60.0;
    double vrwFactor = 1 / 60.0 ;
    for ( auto &it: imuNoise.gyrArw) it *= arwFactor;
    for ( auto &it: imuNoise.accVrw) it *= vrwFactor;
    for ( auto &it: imuNoise.corrTime) it *= 3600;
    // 调整角度、位置等单位
    initState.pos[0] *= DEG2RAD , initState.pos[1] *= DEG2RAD;
    for ( auto &it: initState.euler) it *= DEG2RAD;
    for ( auto &it: initStateStd.euler) it *= DEG2RAD;
}
