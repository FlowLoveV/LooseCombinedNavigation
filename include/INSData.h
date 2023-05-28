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
#include <deque>

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
    double m_pAcc[3] = {0};         // 速度增量 x,y,z  m/s
    double m_pGyr[3] = {0};         // 角增量   x,y,z rad/s

    IMUData_SingleEpoch();
    IMUData_SingleEpoch(const IMUData_SingleEpoch &);
    IMUData_SingleEpoch(const double[],const double[],const GPST &);
    ~IMUData_SingleEpoch();
};

// 惯导机械编排算法解算结果
class INSRes_SingleEpoch{
public:
    GPST t;
    double_t m_pPos[3] = {0};         // 位置 [lat,lon,h]
    double_t m_pSpeed[3] = {0};       // 速度 [Vn,Ve,Vd]
    double_t m_pQuaternion[4] = {0};  // 姿态四元数
    double_t m_pEMatrix[4] = {0};     // 姿态角矩阵

    INSRes_SingleEpoch();
    INSRes_SingleEpoch(const INSRes_SingleEpoch &);
    ~INSRes_SingleEpoch();
};

// 解算INS配置器
class InsConfigure{
    ::std::string m_strImuDataFileDir;        // Imu数据文件路径
    ::std::string m_strGnssDataFileDir;       // Gnss数据文件路径
    ::std::string m_strOutputResFileDir;      // 输出结果文件路径
    double_t m_pStartPos[3] = {0};          // 起始位置信息
    double_t m_pStartV[3] = {0};            // 起始速度信息
    double_t m_pStartEuler[3] = {0};        // 起始欧拉角组（这个信息不一定可以在配置中给出，因为一般初始姿态需要对准后才能获得）
    GPST *m_gpstBeginTime;                     // 开始处理的时间(nullptr表示默认第一行数据开始)
    GPST *m_gpstEndTime;                       // 结束处理的时间(nullptr表示默认最后一行数据结束)

public:
    InsConfigure();

    // set、get函数
    void setImuFileDir(const ::std::string dir) { m_strImuDataFileDir = dir;}
    ::std::string getImuFileDir() const {return m_strImuDataFileDir;}

    void setGnssFileDir(const ::std::string dir) { m_strGnssDataFileDir = dir;}
    ::std::string getGnssFileDir() const {return m_strGnssDataFileDir;}

    void setStartPos(const double_t pos[]) {memcpy(m_pStartPos, pos, IMUDATA_SIZE);}
    const double_t *const getStartPos() const {return m_pStartPos;}

    void setStartV(const double_t v[]) {memcpy(m_pStartV, v, IMUDATA_SIZE);}
    const double_t *const getStartV() const {return m_pStartV;}

    void setStartEuler(const double_t euler[]) {memcpy(m_pStartEuler, euler, IMUDATA_SIZE);}
    const double_t *const getStartEuler() const {return m_pStartEuler;}

    void setOutputDir(const ::std::string dir) { m_strOutputResFileDir = dir;}
    ::std::string getOutputDir() const {return m_strOutputResFileDir;}

    void setBeginTime(GPST * tm);
    GPST * getBeginTime();

    void setEndTime(GPST * tm);
    GPST * getEndTime();
};

// 纯惯导解算器
class PureIns{
    INSRes_SingleEpoch mStartInfo;                  // 初始位置信息
    double_t mFrequency;                            // 采样频率
    ::std::deque<IMUData_SingleEpoch> mObsData;     // 观测数据

public:
    PureIns();
    // 设定惯导初始信息
    void setStartInfo(const double pos[], const double speed[]);
    // 初始粗对准 -- 完成了初始姿态、采样频率的确定 tested
    void gyrAlignment(const ::std::vector<IMUData_SingleEpoch> & rawData,double *euler);
    // k-2,k-1,k历元的观测数据，k-2,k-1历元的位置、速度、姿态数据，计算的到第k历元的结果
    void updateSinEpoch(const IMUData_SingleEpoch & obs2,const IMUData_SingleEpoch & obs1,
                        const IMUData_SingleEpoch & obs, const INSRes_SingleEpoch & res2,
                        const INSRes_SingleEpoch & res1,INSRes_SingleEpoch & res);
    // 手动配置自己的惯导机械编排算法
    // (asc文本、全部读取后处理)
    virtual void insSolver_asc_all(InsConfigure & configure);
    // (bin，逐行处理)


    // 结果文件输出函数
    // 创建输出结果文件流
    ::std::ofstream * createResFile(const ::std::string & fileDir);
    // 输出结果文件
    void outputResFile(const INSRes_SingleEpoch & res,::std::ofstream & outputfile) const;
    // 销毁输出结果的文件流
    void releaseFileStream(::std::ofstream * output);
};



// *********************************************读取IMU数据函数***********************************************************


// ************************************************ASCII ***************************************************************
/*!
 *
 * @param filename input    string     待测量asc文本文件大小
 * @return                  int        目标文本文件中记录惯导数据的历元数
 * @note           该函数用于测量asc文本的IMU数据量大小，%RAWIMUSA为每一行开始的标识符，返回值为IMU测量的总历元数
 */
int asc_measureFileSize(const ::std::string & filename);



/*!
 *
 * @param filename input    string                      待读取asc文本文件的路径
 * @param rawData  output   vector<IMUData_SingleEpoch> 读取后的结果
 * @return                  bool                        是否读取成功
 * @note           该函数用于读取asc文本的IMU数据，%RAWIMUSA为每一行开始的标识符
 */
bool asc_readAll(const ::std::string & filename , ::std::vector<IMUData_SingleEpoch> & rawData);



// ************************************************ BIN ****************************************************************






#endif //COMBINEDNAVIGATION_INSDATA_H
