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
        explicit LooseCombination(const GinsOptions & options);

        void addImuData(const IMUData_SingleEpoch & imudata,bool ifconpensate = false);

        void addGnssResData(const GnssRes & gnssres);

        void newProcess();

        NavState getState();

    protected:

    private:


        /*------------------------------- 成员变量------------------------------------*/
        GinsOptions options_;
        GPST timeStamp_;
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

        // 检验参数
        const int RANK      = 21;
        const int NOISERANK = 18;
        // 状态ID和噪声ID
        // state ID and noise ID
        enum StateID { P_ID = 0, V_ID = 3, PHI_ID = 6, BG_ID = 9, BA_ID = 12, SG_ID = 15, SA_ID = 18 };
        enum NoiseID { VRW_ID = 0, ARW_ID = 3, BGSTD_ID = 6, BASTD_ID = 9, SGSTD_ID = 12, SASTD_ID = 15 };
    };
}



#endif //COMBINEDNAVIGATION_LOOSECOMBINATION_H
