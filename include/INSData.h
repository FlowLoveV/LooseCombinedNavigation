//
// Created by 0-0 mashuo on 2023/5/12.
//

#ifndef COMBINEDNAVIGATION_INSDATA_H
#define COMBINEDNAVIGATION_INSDATA_H

#include "TimeSys.h"
#include "INSSolution.h"
#include "fstream"
#include "ostream"
#include "vector"
#include "Matrix.h"

const int IMUDATA_SIZE = 3*sizeof(double);  // 一组加速度或者陀螺仪数据量大小
const int GPST_SIZE = sizeof(GPST);         // 一个GPST结构体的大小
const int QUATERNION_SIZE = 4*sizeof(double); // 姿态四元数数据量大小
const ::std::string HEADSTRING = "%RAWIMUSA";  // ACSII头字符串

const double NOVATEL_ACC_SCALE = 0.05 / pow(2,15);
const double NOVATEL_GYR_SCALE = 0.1 / 3600 / pow(2,8);


/* 本文件用于读取IMU、GNSS数据
 *
 *
 *
 */
class IMUData_SingleEpoch;
class INSRes_SingleEpoch;
class PureIns;

// IMU 原始数据类
class IMUData_SingleEpoch{
public:
    GPST t;                       // 观测时间
    double mAcc[3] = {0};         // 速度增量 x,y,z  m/s
    double mGyr[3] = {0};         // 角增量   x,y,z rad/s

    IMUData_SingleEpoch();
    IMUData_SingleEpoch(const IMUData_SingleEpoch &);
    IMUData_SingleEpoch(const double[],const double[],const GPST &);
};

// 惯导机械编排算法解算结果
class INSRes_SingleEpoch{
public:
    GPST t;
    double_t mPos[3] = {0};         // 位置 [lat,lon,h]
    double_t mSpeed[3] = {0};       // 速度 [Vn,Ve,Vd]
    double_t mQuaternion[4] = {0};  // 姿态四元数
    double_t mEMatrix[4] = {0};     // 姿态角矩阵

    INSRes_SingleEpoch();
    INSRes_SingleEpoch(const INSRes_SingleEpoch &);

};

// 解算INS配置器
class InsConfigure{
    ::std::string mImuDataFileDir;        // Imu数据文件路径
    ::std::string mGnssDataFileDir;       // Gnss数据文件路径
    ::std::string moutputResFileDir;      // 输出结果文件路径
    double_t mStartPos[3] = {0};          // 起始位置信息
    double_t mStartV[3] = {0};            // 起始速度信息
    double_t mStartEuler[3] = {0};        // 起始欧拉角组（这个信息不一定可以在配置中给出，因为一般初始姿态需要对准后才能获得）

public:
    InsConfigure();

    // set、get函数
    void setImuFileDir(const ::std::string dir) {mImuDataFileDir = dir;}
    ::std::string getImuFileDir() const {return mImuDataFileDir;}
    void setGnssFileDir(const ::std::string dir) {mGnssDataFileDir = dir;}
    ::std::string getGnssFileDir() const {return mGnssDataFileDir;}
    void setStartPos(const double_t pos[]) {memcpy(mStartPos,pos,IMUDATA_SIZE);}
    const double_t *const getStartPos() const {return mStartPos;}
    void setStartV(const double_t v[]) {memcpy(mStartV,v,IMUDATA_SIZE);}
    const double_t *const getStartV() const {return mStartV;}
    void setStartEuler(const double_t euler[]) {memcpy(mStartEuler,euler,IMUDATA_SIZE);}
    const double_t *const getStartEuler() const {return mStartEuler;}
    void setOutputDir(const ::std::string dir) {moutputResFileDir = dir;}
    ::std::string getOutputDir() const {return moutputResFileDir;}
};

// 纯惯导解算器
class PureIns{
    INSRes_SingleEpoch mStartInfo;   // 初始位置信息
    double_t mFrequency;   // 采样频率

public:
    PureIns();
    // 设定惯导初始信息
    void setStartInfo(const double pos[], const double speed[]);
    // 初始粗对准 -- 完成了初始姿态、采样频率的确定
    void gyrAlignment(const ::std::vector<IMUData_SingleEpoch> & rawData,double *euler);
    // k-2,k-1,k历元的观测数据，k-2,k-1历元的位置、速度、姿态数据，计算的到第k历元的结果
    void updateSinEpoch(const IMUData_SingleEpoch & obs2,const IMUData_SingleEpoch & obs1,
                        const IMUData_SingleEpoch & obs, const INSRes_SingleEpoch & res2,
                        const INSRes_SingleEpoch & res1,INSRes_SingleEpoch & res);
    // 手动配置自己的惯导机械编排算法
    virtual void standardINSSolver(InsConfigure & configure);
    // 创建输出结果文件流
    ::std::ofstream * createResFile(const ::std::string & fileDir);
    // 输出结果文件
    void outputResFile(const INSRes_SingleEpoch & res,::std::ofstream & outputfile) const;
    // 销毁输出结果的文件流
    void releaseFileStream(::std::ofstream * output);
};



// *********************************************读取IMU数据函数***********************************************************

// 得到ASCII观测文件总历元数
/* input : file_directory
 * output : total epochs
 */
int measureFileSize(const ::std::string & filename);


// 读取ASCII观测文件原始观测数据
/* input1 : file_directory
 * output1 : rawData
 */
bool readASCall(const ::std::string & filename , ::std::vector<IMUData_SingleEpoch> & rawData);




#endif //COMBINEDNAVIGATION_INSDATA_H
