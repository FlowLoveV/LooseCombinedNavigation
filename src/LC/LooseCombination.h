//
// Created by 0-0 mashuo on 2023/6/4.
//

#ifndef COMBINEDNAVIGATION_LOOSECOMBINATION_H
#define COMBINEDNAVIGATION_LOOSECOMBINATION_H

#include "INSType.h"
#include "GNSSType.h"
#include "GINSType.h"
#include "Filter/cKalman.h"

/**< 组合导航命名空间 */
namespace ns_GINS{
    class LooseCombination {
    public:
        /*!
         * 使用一个GINSOptions配置器来初始化一个组合导航实例
         * @param options       input       GinsOptions         组合导航解算配置
         */
        explicit LooseCombination(const YAML::Node & config);

        void addImuData(const IMUData_SingleEpoch & imudata,bool ifconpensate = false);

        void addGnssResData(const GnssRes & gnssres);

        void newProcess();

        NavState getState();

    protected:

        void preForPredict();

        virtual void preForUpdate();

        void initialize();

    private:

        int isToUpdate();


    private:
        /*------------------------------- 成员变量------------------------------------*/
        GinsOptions options_;
        const double TIME_ALIGN_ERROR = 0.001;        /**< imu数据和GNSS数据对齐误差 */
        // IMU数据(记录三个历元的IMU观测数据)
        std::vector<IMUData_SingleEpoch> imuData_;
        // IMU状态(记录三个历元位置、速度、姿态)
        std::vector<INSRes_SingleEpoch> insState_;
        // IMU误差
        ImuError imuError_;
        // GNSS结果
        GnssRes gnssRes_;
        // 滤波器
        ns_filter::cKalman filter_;
        // 观测模型参数
        enum OBSERVATION_MODEL {POS = 1, POS_VEL = 2, POS_VEL_ODO = 3};
        int obs_model = 0;
        // 检验参数
        const int RANK      = 21;
        const int NOISERANK = 18;
        // 状态ID和噪声ID
        // state ID and noise ID
        enum StateID { P_ID = 1, V_ID = 4, PHI_ID = 7, BG_ID = 10, BA_ID = 13, SG_ID = 16, SA_ID = 19 };
        enum NoiseID { VRW_ID = 1, ARW_ID = 4, BGSTD_ID = 7, BASTD_ID = 10, SGSTD_ID = 13, SASTD_ID = 16 };
    };
}



#endif //COMBINEDNAVIGATION_LOOSECOMBINATION_H
