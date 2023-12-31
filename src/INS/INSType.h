//
// Created by 0-0 mashuo on 2023/5/12.
//

#ifndef COMBINEDNAVIGATION_INSTYPE_H
#define COMBINEDNAVIGATION_INSTYPE_H

#include "TimeSys.h"
#include "BasicFuns.h"
#include "fstream"
#include "ostream"
#include "vector"
#include "Matrix.h"
#include <deque>
#include "GINSType.h"

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

/*!
 * @brief IMU单历元原始观测数据类
 */
class IMUData_SingleEpoch{
public:
    GPST t;                         /**< 观测时间,GPST */
    double m_pAcc[3] = {0};         /**< 速度增量 x,y,z  m/s */
    double m_pGyr[3] = {0};         /**< 角增量  x,y,z rad/s */

    /*!
     * @brief 默认构造函数
     */
    IMUData_SingleEpoch();

    /*!
     * @brief 实例构造函数
     * 从另一个IMU原始观测实例构造生成一个相同的IMU原始观测实例
     * @param another_epoch input   IMUData_SingleEpoch     一个IMU原始观测实例
     */
    IMUData_SingleEpoch(const IMUData_SingleEpoch &another_epoch);

    /*!
     * @brief 成员构造函数
     * 根据IMU原始观测数据、时间构造一个IMU原始观测实例
     * @param acc   input       double[3]       原始观测的三轴速度增量，x,y,z m/s
     * @param gyr   input       double[3]       原始观测的三轴角增量，  x,y,z rad/s
     * @param gpst  input       GPST            观测时间
     */
    IMUData_SingleEpoch(const double *acc, const double *gyr,const GPST &gpst);

    /*!
     * 根据长度为8，依次记录了GPST-week,GPST-second,delta_v_x,delta_v_y,delta_v_z,delta_w_x,delta_w_y,delta_w_z
     * 的vector构造一个IMU_Data_SingleEpoch实例
     * @param t     input       std::vector<double>     
     */
    IMUData_SingleEpoch(std::vector<double> &t);

    /*!
     * @brief 默认析构函数
     */
    ~IMUData_SingleEpoch();

    /*!
     * 根据两个历元的IMU数据，线型内插出指定时刻的IMU数据
     * @param data0         input       IMUData_SingleEpoch         上一历元数据
     * @param data          output      IMUData_SingleEpoch         指定时刻内插得到的IMU数据
     * @param timestamp     input       GPST                        指定时刻
     */
    void interpolationImuData(const IMUData_SingleEpoch & data0,IMUData_SingleEpoch & data,const GPST & timestamp);

    /*!
     * 补偿IMU原始数据的误差
     * @param error         input       ImuError                    imu零偏、比例因子误差类
     * @param dt            input       double                      补偿时间
     */
    void compensate(const ImuError & error,const double & dt);

};




// 惯导机械编排算法解算结果
/*!
 * @brief INS单历元解算结果类
 */
class INSRes_SingleEpoch{
public:
    GPST t;                           /**< 观测时间            */
    double_t m_pPos[3] = {0};         /**< 位置 [lat,lon,h]   */
    double_t m_pSpeed[3] = {0};       /**< 速度 [Vn,Ve,Vd]    */
    double_t m_pQuaternion[4] = {0};  /**< 姿态四元数          */
    double_t m_pEMatrix[9] = {0};     /**< 姿态角矩阵          */
    double_t m_pEuler[3]{};           /**< 欧拉角组            */
    ImuError m_error;                 /**< IMU误差系数         */



    /*!
     * @brief 默认构造函数
     */
    INSRes_SingleEpoch();

    /*!
     * @brief 实例构造函数
     * 从另一个INS单历元解算结果实例构造一个相同的示例
     * @param another   input   INSRes_SingleEpoch      一个INS单历元解算结果实例
     */
    INSRes_SingleEpoch(const INSRes_SingleEpoch & another);

    /*!
     * @brief 默认析构函数
     */
    ~INSRes_SingleEpoch();

    /*!
     * 将INS状态转为一个vector向量
     * @return [GPST-week,GPST-second,pos-lat,pos-lon,pos-h,vel-n,vel-e,vel-d,euler-roll,euler-pitch,euler-yaw];
     */
    std::vector<double> toVector();

    /*!
     * 根据松组合系统状态反馈IMU状态
     * dx =  [delta_pos delta_v delta_attitude delta_Bg delta_Ba delta_Sg delta_Sa]
     * @param dx        input       Matrix          松组合系统状态向量
     */
    void stateFeedback(const Matrix & dx);


    /*!
     * 改变单位
     * b l roll pitch yaw单位由 deg 转为 rad
     */
    void changeUnitD2R();

    /*!
     * 改变单位
     * b l roll pitch yaw单位由 rad 转为 deg
     */
    void changeUnitR2D();
};


/*!
 * @brief INS解算的配置器
 */
class InsConfigure{
    ::std::string m_strImuDataFileDir;        /**< Imu数据文件路径 */
    ::std::string m_strOutputResFileDir;      /**< 输出INS结果文件路径 */
    double_t m_pStartPos[3] = {0};            /**< 起始位置信息 */
    double_t m_pStartV[3] = {0};              /**< 起始速度信息 */
    double_t m_pStartEuler[3] = {0};          /**< 起始欧拉角组（这个信息不一定可以在配置中给出，因为一般初始姿态需要对准后才能获得） */
    GPST *m_gpstBeginTime;                    /**< 开始处理的时间(nullptr表示默认第一行数据开始) */
    GPST *m_gpstEndTime;                      /**< 结束处理的时间(nullptr表示默认最后一行数据结束) */

public:
    InsConfigure();

    // set、get函数
    void setImuFileDir(const ::std::string dir) { m_strImuDataFileDir = dir;}
    ::std::string getImuFileDir() const {return m_strImuDataFileDir;}

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
/*!
 * @brief 惯性导航-机械编排算法解算类
 */
class PureIns{
    INSRes_SingleEpoch m_StartInfo;                  /**< 初始位置信息 */
    double_t m_dSampleFrequency{};                     /**< 采样频率 */
    ::std::deque<IMUData_SingleEpoch> m_ObsData;     /**< 观测数据 */

public:

    /*!
     * @brief 默认构造函数
     */
    PureIns();

    /*!
     * 设定惯导初始信息 - 初始位置、初始速度
     * @param pos      input    double[3]       [lat,lon,h]
     * @param speed    input    double[3]       [n,e,d] m/s
     */
    void setStartInfo(const double pos[], const double speed[]);

    /*!
     * 初始粗对准 -- 确定初始姿态
     * @param rawData   input       std::vector<IMUData_SingleEpoch>    IMU原始观测数据向量
     * @param euler     output      double[3]                           欧拉角组[roll,pitch,yaw] rad
     */
    void gyrAlignment(const ::std::vector<IMUData_SingleEpoch> & rawData,double *euler);

    /*!
     * 惯性导航机械编排算法单历元更新函数，静态函数 经过检验，绝对可信
     * @param obs2      input       IMUData_SingleEpoch     k-2历元IMU原始观测数据
     * @param obs1      input       IMUData_SingleEpoch     k-1历元IMU原始观测数据
     * @param obs       input       IMUData_SingleEpoch     k历元IMU原始观测数据
     * @param res2      input       INSRes_SingleEpoch      k-2历元INS解算结果(位置、速度，姿态)
     * @param res1      input       INSRes_SingleEpoch      k-1历元INS解算结果(位置、速度，姿态)
     * @param res       output      INSRes_SingleEpoch      K历元INS解算结果(位置、速度，姿态)
     */
    static void updateSinEpoch(const IMUData_SingleEpoch & obs2,const IMUData_SingleEpoch & obs1,
                        const IMUData_SingleEpoch & obs, const INSRes_SingleEpoch & res2,
                        const INSRes_SingleEpoch & res1,INSRes_SingleEpoch & res);


    // 手动配置自己的惯导机械编排算法
    // (asc文本、全部读取后处理)
    virtual void insSolver_asc_all(InsConfigure & configure);


    // 截取指定IMU文件下的一段数据

};






// *********************************************读取IMU数据函数***********************************************************


// ************************************************ASCIITYPE ***************************************************************
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






#endif //COMBINEDNAVIGATION_INSTYPE_H
