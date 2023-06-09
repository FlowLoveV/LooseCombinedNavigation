//
// Created by 0-0 mashuo on 2023/6/5.
//

#ifndef COMBINEDNAVIGATION_GINSTYPE_H
#define COMBINEDNAVIGATION_GINSTYPE_H

// 本文件中定义结构体中IMU误差、噪声及其标准差的单位如下，方便程序中计算
/**< 陀螺仪零偏               rad/s            */
/**< 加速度计零偏               m/s            */
/**< 比例因子                 无单位           */
/**< 角度随机游走            rad/sqrt(s)       */
/**< 加速度随机游走            m/sqrt(s)       */
/**< 相关时间                   s             */

#include "GNSSType.h"
#include "yaml-cpp/include/yaml-cpp/yaml.h"


struct ImuError{
    double gBias[3];       /**< 陀螺仪零偏                   */
    double aBias[3];       /**< 加速度计零偏                 */
    double gScale[3];      /**< 陀螺仪比例因子误差            */
    double aScale[3];      /**< 加速度计比例因子误差          */
};

struct ImuErrorNoise{
    double accVrw[3];      /**< 三轴速度随机游走常量         */
    double gyrArw[3];      /**< 三轴角度随机游走常量         */
    double gBiasStd[3];    /**< 三轴角速度零偏白噪声标准差    */
    double aBiasStd[3];    /**< 三轴加速度零偏白噪声标准差    */
    double gScaleStd[3];   /**< 三轴角速度比例因子白噪声标准差 */
    double aScaleStd[3];   /**< 三轴加速度比例因子白噪声标准差 */
    double corrTime[4];    /**< gB,aB,gS,aS随机游走相关时间  */
};

struct NavState{
    double pos[3];         /**< 位置 lat lon h                   */
    double vel[3];         /**< 速度 n e d m/s                   */
    double euler[3];       /**< 姿态角-欧拉角组 roll pitch yaw rad */
    ImuError imuError;     /**< IMU 误差                         */

};

struct GinsOptions{
    NavState initState;             /**< 初始状态                         */
    NavState initStateStd;          /**< 初始状态标准差                    */

    ImuErrorNoise imuNoise;         /**< imu误差噪声参数                   */

    double AntennaLeverArm[3]{};    /**< 安装参数：天线杆臂                 */

    GinsOptions();

    /*!
     * 根据配置信息直接得到GNSS/INS解算配置参数
     * @param config        input       YAML::Node      在使用YAML:LoadFile()函数后将config作为参数传递即可
     */
    explicit GinsOptions(const YAML::Node & config);

    /*!
     * 本函数用于向各种流中输出GNSS/INS组合导航参数配置，一般参数包括std::cout,std::fstream
     * @param stream    input       std::ostream        流
     */
    void print(std::ostream & stream = std::cout);

    /*!
     * 调整单位
     */
    void adjustUnit();

};


#endif //COMBINEDNAVIGATION_GINSTYPE_H
