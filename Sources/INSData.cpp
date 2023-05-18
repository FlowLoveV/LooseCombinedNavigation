//
// Created by 0-0 mashuo on 2023/5/12.
//

#include <sstream>
#include "INSData.h"
#include "cstring"
#include "iostream"


IMUData_SingleEpoch::IMUData_SingleEpoch() = default ;

IMUData_SingleEpoch::IMUData_SingleEpoch(const IMUData_SingleEpoch & another_epoch) {
    memcpy(mAcc,another_epoch.mAcc,IMUDATA_SIZE);
    memcpy(mGyr,another_epoch.mGyr,IMUDATA_SIZE);
    memcpy(&t,&another_epoch.t,GPST_SIZE);
}

IMUData_SingleEpoch::IMUData_SingleEpoch(const double *acc, const double *gyr, const GPST & gpst) {
    memcpy(mAcc,acc,IMUDATA_SIZE);
    memcpy(mGyr,gyr,IMUDATA_SIZE);
    memcpy(&t,&gpst,GPST_SIZE);
}

bool readASCall(const ::std::string & filename, ::std::vector<IMUData_SingleEpoch> & rawData){
    ::std::fstream file(filename);
    if(!file.is_open()){
        ::std::cerr<<"can't open file:"<<filename;
        return false;
    }
    // 按行读取
    ::std::string line;
    char delimiter[] = ",;*";
    int i = 0;
    while(::std::getline(file,line)){
        if(line.find(HEADSTRING) != ::std::string::npos){
            char* str = new char[line.length()+1];
            ::std::strcpy(str,line.c_str());
            // 解码数据
            char* token = strtok(str, delimiter);
            token = strtok(NULL,delimiter);
            token = strtok(NULL,delimiter);
            token = strtok(NULL,delimiter);      rawData[i].t.weeks = ::std::atof(token);
            token = strtok(NULL,delimiter);      rawData[i].t.second = ::std::atof(token);
            token = strtok(NULL,delimiter);
            token = strtok(NULL,delimiter);      rawData[i].mAcc[2] = - ::std::atof(token) * NOVATEL_ACC_SCALE;     // +z-U -> +z-D
            token = strtok(NULL,delimiter);      rawData[i].mAcc[0] = - ::std::atof(token) * NOVATEL_ACC_SCALE;     // -y-N -> +x->N
            token = strtok(NULL,delimiter);      rawData[i].mAcc[1] = ::std::atof(token) * NOVATEL_ACC_SCALE;       // +x-E -> +y->E
            token = strtok(NULL,delimiter);      rawData[i].mGyr[2] = - ::std::atof(token) * NOVATEL_GYR_SCALE;
            token = strtok(NULL,delimiter);      rawData[i].mGyr[0] = - ::std::atof(token) * NOVATEL_GYR_SCALE;
            token = strtok(NULL,delimiter);      rawData[i].mGyr[1] = ::std::atof(token) * NOVATEL_GYR_SCALE;
            delete []str;
            i++;
        }
        else continue;
    }
    file.close();
    return true;
}


int measureFileSize(const ::std::string & filename){
    ::std::fstream file(filename);
    if(!file.is_open()){
        ::std::cerr<<"can't open file:"<<filename;
        return 0;
    }
    int line = 0;
    ::std::string str_line;
    while(::std::getline(file,str_line)){
        if(str_line.find(HEADSTRING) != ::std::string::npos){
            line++;
        }
    }
    return line;
}

void PureIns::gyrAlignment(const ::std::vector<IMUData_SingleEpoch> & rawData, double *euler) {
    int len = rawData.size();
    double average_f[3] = {0};
    double average_omega[3] = {0};
    for (auto &i : rawData){
        average_f[0] += i.mAcc[0];
        average_f[1] += i.mAcc[1];
        average_f[2] += i.mAcc[2];
        average_omega[0] += i.mGyr[0];
        average_omega[1] += i.mGyr[1];
        average_omega[2] += i.mGyr[2];
    }
    mFrequency = (len - 1) / abs(rawData[0].t - rawData[len-1].t);
    for (int i = 0; i < 3; ++i) {
        average_f[i] = average_f[i] / len * mFrequency;
        average_omega[i] = average_omega[i] / len * mFrequency;
    }
    double lat = mStartInfo.mPos[0], h = mStartInfo.mPos[2];
    Matrix g_n(3,1,{0,0, calculate_g(lat,h)});
    Matrix omega_n_ie(3,1,{ELLIPSOID_OMEGA*cos(lat),0,-ELLIPSOID_OMEGA*sin(lat)});
    Matrix omega_b_ie(3,1,average_omega);
    Matrix g_b(3,1,average_f);
    g_b = 0 - g_b;
    // 开始求解
    Matrix v[3],w[3];
    v[1] = cross(g_n,omega_n_ie);
    v[2] = cross(v[1],g_n);
    w[1] = cross(g_b,omega_b_ie);
    w[2] = cross(w[1],g_b);
    v[0] = g_n / array_norm(g_n.p);
    v[1] = v[1] / array_norm(v[1].p);
    v[2] = v[2] / array_norm(v[2].p);
    w[0] = g_b / array_norm(g_b.p);
    w[1] = w[1] / array_norm(w[1].p);
    w[2] = w[2] / array_norm(w[2].p);
    w[0].reshape(1,3);
    w[1].reshape(1,3);
    w[2].reshape(1,3);
    Matrix C = horizontal_stack_array(v,3) * vertical_stack_array(w,3);
    EMatrix2Euler(C.p,euler);
    // 将对准姿态结果存储到初始信息中，以姿态四元数及姿态矩阵的形式
    Euler2Quaternion(euler,mStartInfo.mQuaternion);
    Euler2EMatrix(euler,mStartInfo.mEMatrix);
}

void PureIns::setStartInfo(const double *pos, const double *speed) {
    memcpy(&mStartInfo.mPos,pos,IMUDATA_SIZE);
    memcpy(&mStartInfo.mSpeed,speed,IMUDATA_SIZE);
}

void PureIns::standardINSSolver(InsConfigure & configure) {
    setStartInfo(configure.getStartPos(),configure.getStartV());
    // 读取ASCII文本
    size_t epoch_num = measureFileSize(configure.getImuFileDir());
    ::std::vector<IMUData_SingleEpoch> imuData(epoch_num);
    readASCall(configure.getImuFileDir(),imuData);
    double_t euler[3];
    gyrAlignment(imuData,euler);
}

PureIns::PureIns() = default;

INSRes_SingleEpoch::INSRes_SingleEpoch() = default;

INSRes_SingleEpoch::INSRes_SingleEpoch(const INSRes_SingleEpoch & another) {
    memcpy(&t,&another.t,GPST_SIZE);
    memcpy(&mPos,&another.mPos,IMUDATA_SIZE);
    memcpy(&mSpeed,&another.mSpeed,IMUDATA_SIZE);
    memcpy(&mQuaternion,&another.mQuaternion,QUATERNION_SIZE);
    memcpy(&mEMatrix,&another.mEMatrix,3*IMUDATA_SIZE);
}

InsConfigure::InsConfigure() = default;


void PureIns::updateSinEpoch(const IMUData_SingleEpoch & obs2,const IMUData_SingleEpoch & obs1,
                             const IMUData_SingleEpoch & obs, const INSRes_SingleEpoch & res2,
                             const INSRes_SingleEpoch & res1,INSRes_SingleEpoch & res) {
    double_t rv[3],qb[4],qn[4];
    Angle2RV(obs2.mGyr,obs1.mGyr,rv); // 双子样法更新等效旋转矢量
    RV2Quaternion(rv,qb);                         // 等效旋转矢量更新b系
    double_t pos[3],speed[3],wie[3],wen[3],Rm,Rn;
    double_t delta_t = obs.t - obs1.t; // 观测间隔
    // 利用k-2、k-1历元位置、速度外推得到k-0.5历元位置、速度
    for (int i = 0; i < 3; ++i) {
        pos[i] = linExtrapolateHalf(res2.mPos[i],res1.mPos[i]);
        speed[i] = linExtrapolateHalf(res2.mSpeed[i],res1.mSpeed[i]);
    }
    calculateRotationSpeed(pos,speed,wie,wen,Rm,Rn);
    Matrix m_wie(3,1,wie),m_wen(3,1,wen);
    Matrix m_rv = (m_wie + m_wen) ^ delta_t;
    RV2Quaternion(m_rv.p,qn);
    // 需要将qn的虚部反号
    for (int i = 1; i < 4; ++i) {
        qn[i] = -qn[i];
    }
    // 更新姿态 - 姿态四元数及姿态矩阵
    double_t q_temp[4];
    Multiply_q(qn,res1.mQuaternion,q_temp);
    Multiply_q(q_temp,qb,res.mQuaternion);
    Quaternion2EMatrix(res.mQuaternion,res.mEMatrix);
    // 更新速度
    double_t g = calculate_g(pos[0],pos[2]);   // 重力
    Matrix m_g(3,1,{0,0,g}),m_v(3,1,speed);           // 构造重力、k-0.5历元速度矩阵
    Matrix m_cor = ( m_g - cross( ((2 * m_wie) + m_wen),m_v ) ) * delta_t;  // 哥氏积分项矩阵
    Matrix m_vk(3,1,obs.mAcc),m_thetak(3,1,obs.mGyr); // k历元原始观测值矩阵
    Matrix m_vk_1(3,1,obs1.mAcc),m_thetak_1(3,1,obs1.mGyr); // k-1历元原始观测值矩阵
    Matrix m_v_fk_1_b = m_vk + cross(m_thetak,m_vk) / 2 +
            ( cross(m_thetak_1,m_vk) + cross(m_vk_1,m_thetak) ) / 12;
    Matrix m_Cnbk_1(3,3,res1.mEMatrix);   // 上一历元姿态矩阵
    Matrix m_v_fk_n = (eye(3) - antiVector(m_rv) / 2) * m_Cnbk_1 * m_v_fk_1_b;
    Matrix m_res_vk = Matrix(3,1,res1.mSpeed) + m_cor + m_v_fk_n;
    memcpy(res.mSpeed,m_res_vk.p,IMUDATA_SIZE);
    // 更新位置
    res.mPos[2] = res1.mPos[1] - (res.mSpeed[2] + res1.mSpeed[2]) / 2 * delta_t;
    double_t average_h = (res.mPos[2] + res.mPos[2]) / 2;
    res.mPos[0] = res1.mPos[0] + (res.mSpeed[0] + res1.mSpeed[0]) / 2 / (Rm + average_h) * delta_t;
    double_t average_lat = (res.mPos[0] + res.mPos[0]) / 2;
    res.mPos[1] = res1.mPos[1] + (res.mSpeed[1] + res1.mSpeed[1]) / 2 / (Rn + average_h) / cos(average_lat) * delta_t;
}

void PureIns::outputResFile(const INSRes_SingleEpoch & res, ::std::ostream & outputfile) const {
    double_t euler[3];
    EMatrix2Euler(res.mEMatrix,euler);
    outputfile << res.t.weeks << "," << res.t.second << "," <<
    res.mPos[0] << "," << res.mPos[1] << "," << res.mPos[2] << "," <<
    res.mSpeed[0] << "," << res.mSpeed[1] << "," << res.mSpeed[2] << "," <<
    euler[2] << "," << euler[1] << "," << euler[0] << '\n';
}

::std::ostream *PureIns::createResFile(const ::std::string & fileDir) {
    // 创建文件流，设定输出格式
    ::std::ofstream* output = new std::ofstream(fileDir);
    *output << ::std::fixed << ::std::setprecision(5);

    return output;
}


void PureIns::releaseFileStream(::std::ostream *output) {
    delete output;
}
