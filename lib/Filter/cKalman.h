//
// Created by 0-0 mashuo on 2023/5/30.
//

#ifndef COMBINEDNAVIGATION_CKALMAN_H
#define COMBINEDNAVIGATION_CKALMAN_H


#include "Matrix.h"
#include <iostream>


/*!
 * ns_filter命名空间 - 保存有关滤波算法的类以及函数
 */
namespace ns_filter{
    /*!
     * @brief 基础卡尔曼滤波类
     */
    class cKalman{
    public:
        /*!
         * @brief 构造函数
         */
        cKalman();

        void setMState0(const Matrix &mState0);

        void setMStateVariance0(const Matrix &mStateVariance0);

        void setMStateTrans(const Matrix &mStateTrans);

        void setMSystemNoise(const Matrix &mSystemNoise);

        void setMSystemNoiseDrive(const Matrix &mSystemNoiseDrive);

        void setMMeasurement(const Matrix &mMeasurement);

        void setMObservation(const Matrix &mObservation);

        void setMMeasureNoise(const Matrix &mMeasureNoise);

        /*!
         * 改变系统状态
         * @param state             input       Matrix          系统状态向量
         * @param state_variance    input       Matrix          系统状态向量方差
         */
        void changeState(const Matrix & state,const Matrix & state_variance);


        // 一步预测准备
        /*!
         * 进行一步预测
         * @param phi                       input           Matrix          状态转移矩阵
         * @param system_noise_variance     input           Matrix          系统噪声方差矩阵
         * @param system_noise_drive        input           Matrix          系统噪声驱动矩阵
         */
        void Predict(const Matrix & phi,const Matrix & system_noise_variance,const Matrix & system_noise_drive);


        // 测量更新准备
        /*!
         * 进行量测更新
         * @param obs                       input           Matrix          观测值
         * @param H                         input           Matrix          量测矩阵
         * @param R                         input           Matrix          量测噪声
         */
        void Update(const Matrix & obs,const Matrix & H,const Matrix & R);



        /*!
         * 进行一次预测和量测更新
         * @return bool 滤波更新是否成功
         */
        virtual bool upDate();


        /*!
         * 转移状态向量，这个函数用于在进行一次滤波更新后，将实例中的成员m_State_k赋值给m_State_k_1，便于
         * 下一次滤波更新的开始。
         */
        void TransStateMatrix();



        /*!
         * 转移状态向量的方差阵，这个函数用于在进行一次滤波更新后，将实例中的成员m_StateVariance_k赋值给
         * m_StateVariance_k_1，便于下一次滤波更新的开始
         */
        void TransStateVariance();

        const Matrix &getMStateK() const;

        const Matrix &getMStateVarianceK() const;


    protected:
        Matrix m_State0;                      /**< 初始时刻状态向量 */
        Matrix m_StateVariance0;              /**< 初始时刻状态向量方差 */
    private:
        Matrix m_State_k_1;                   /**< 上一历元状态向量 */
        Matrix m_State_estimated;             /**< 状态向量的预测值 */
        Matrix m_State_k;                     /**< 状态向量更新值 */
        Matrix m_StateTrans;                  /**< 状态转移矩阵 */
        Matrix m_StateVariance_k_1;           /**< 上一历元状态向量方差阵 */
        Matrix m_StateVariance_estimated;     /**< 状态向量方差阵的预测值 */
        Matrix m_StateVariance_k;             /**< 状态向量方差阵的更新值 */
        Matrix m_SystemNoise;                 /**< 系统噪声方差阵 */
        Matrix m_SystemNoiseDrive;            /**< 系统噪声驱动阵 */
        Matrix m_Measurement;                 /**< 量测矩阵 */
        Matrix m_Observation;                 /**< 观测值向量 */
        Matrix m_MeasureNoise;                /**< 观测噪声矩阵 */
        Matrix m_Gain;                        /**< 增益矩阵 */
    };
}


#endif //COMBINEDNAVIGATION_CKALMAN_H
