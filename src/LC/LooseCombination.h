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

        /*!
         * 增加imu观测值
         * @param imudata           input       IMUData_SingleEpoch     imuData
         * @param ifconpensate      input       bool                    是否对imu数据进行补偿 default = false
         */
        void addImuData(const IMUData_SingleEpoch & imudata,bool ifconpensate = false);

        /*!
         * 增加GNSS观测值
         * @param gnssres           input        GnssRes        gnss定位结果
         */
        void addGnssResData(const GnssRes & gnssres);

        /*!
         * 开始新一历元的处理过程
         */
        void newProcess();

        /*!
         * 获得当前IMU状态
         * @return      NavState    IMU状态
         */
        NavState getNavState();

        /*!
         * 得到当前定位的时间
         * @return      GPST        当前时间
         */
        GPST getTime();

        /*!
         * 获得当前IMU状态方差
         * @return      Matrix      IMU状态协方差阵
         */
        Matrix getStateVariance();

    protected:

        /*!
         * 根据上一历元IMU状态和本历元IMU观测值，构建状态转移矩阵Phi和噪声驱动阵G
         * @param res       input       INSRes_SingleEpoch      待更新历元的上一历元IMU状态
         * @param obs       input       IMUData_SingleEpoch     待更新历元IMU观测值
         * @param dt        input       double                  历元间隔
         */
        void preForPredict(const INSRes_SingleEpoch & res,const IMUData_SingleEpoch & obs,const double &dt);


        /*!
         * 根据更新历元IMU的状态构建构建Z、H、R矩阵
         * @param res       input       INSRes_SingleEpoch      待更新历元IMU状态（该状态由机械编排算法推导得来）
         * @param obs       input       IMUData_SingleEpoch     待更新历元IMU观测值
         * @param dt        input       double                  历元间隔
         */
        virtual void preForUpdate(const INSRes_SingleEpoch & res,const IMUData_SingleEpoch & obs,const double &dt);

        // 初始化状态向量x，和初始方差P0以及过程噪声阵Q
        void initialize();

        // ins更新单历元
        virtual void mesh();

    private:

        /*!
         * 判断下一步是否进行更新或者其他操作
         * @param updateTime        input       GPST        GNSS定位时间
         * @return  -1 : 更新上一历元状态
         *          0  : 更新本历元状态
         *          1  : 在上一历元和本历元间插值更新状态
         *          2  : 不更新状态，只进行机械编排递推
         */
        int ifUpdate(GPST updateTime);


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
