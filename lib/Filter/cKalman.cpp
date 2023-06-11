//
// Created by 0-0 mashuo on 2023/5/30.
//


#include "cKalman.h"

void ns_filter::cKalman::setMState0(const Matrix &mState0) {
    m_State0 = mState0;
    m_State_k_1 = mState0;
}

void ns_filter::cKalman::setMStateVariance0(const Matrix &mStateVariance0) {
    m_StateVariance0 = mStateVariance0;
    m_StateVariance_k_1 = mStateVariance0;
}

void ns_filter::cKalman::setMStateTrans(const Matrix &mStateTrans) {
    m_StateTrans = mStateTrans;
}

void ns_filter::cKalman::setMSystemNoise(const Matrix &mSystemNoise) {
    m_SystemNoiseVariance = mSystemNoise;
}

void ns_filter::cKalman::setMMeasurement(const Matrix &mMeasurement) {
    m_Measurement = mMeasurement;
}

void ns_filter::cKalman::setMObservation(const Matrix &mObservation) {
    m_Observation = mObservation;
}

void ns_filter::cKalman::setMMeasureNoise(const Matrix &mMeasureNoise) {
    m_MeasureNoise = mMeasureNoise;
}

void ns_filter::cKalman::TransStateMatrix() {
    m_State_k_1 = m_State_k;
}

void ns_filter::cKalman::TransStateVariance() {
    m_StateVariance_k_1 = m_StateVariance_k;
}

bool ns_filter::cKalman::upDate() {
    try {
        // 一步预测
        m_State_estimated = m_StateTrans * m_State_k_1;
        m_StateVariance_estimated = m_StateTrans * m_StateVariance_k_1 * m_StateTrans.T() +
                                     m_SystemNoiseVariance;
        // 量测更新
        m_Gain = m_StateVariance_estimated * m_Measurement.T() * (m_Measurement *
                                                                  m_StateVariance_estimated * m_Measurement.T() +
                                                                  m_MeasureNoise).inv();
        m_State_k = m_State_estimated + m_Gain * (m_Observation - m_Measurement * m_State_estimated);
        int row = m_State_k.row;
        Matrix I = eye(row);
        m_StateVariance_k = (I - m_Gain * m_Measurement) * m_StateVariance_estimated *
                            (I - m_Gain * m_Measurement).T() + m_Gain * m_MeasureNoise * m_Gain.T();

    }
    catch (const std::exception &e){
        std::cout << "error in filter\n";
        ::std::cout<<"Exception caught:"<<e.what();
        return false;
    }
    // 转移
    TransStateMatrix();
    TransStateVariance();
    return true;
}

void ns_filter::cKalman::changeState(const Matrix &state, const Matrix &state_variance) {
    m_State_k_1 = state;
    m_StateVariance_k_1 = state_variance;
}

void ns_filter::cKalman::Predict(const Matrix &phi, const Matrix &system_noise_variance) {
    // 赋值状态转移矩阵，系统噪声方差矩阵和系统噪声驱动矩阵
    m_StateTrans = phi;
    m_SystemNoiseVariance = system_noise_variance;
    // 开始进行一步预测
    m_State_estimated = phi * m_State_k_1;
    m_StateVariance_estimated = m_StateTrans * m_StateVariance_k_1 * m_StateTrans.T() +
                                 m_SystemNoiseVariance;
}

void ns_filter::cKalman::Update(const Matrix &obs, const Matrix &H, const Matrix &R) {
    // 赋值观测值向量、量测矩阵和量测噪声矩阵
    m_Observation = obs;
    m_Measurement = H;
    m_MeasureNoise = R;
    // 进行测量更新
    m_Gain = m_StateVariance_estimated * m_Measurement.T() * (m_Measurement *
                                                              m_StateVariance_estimated * m_Measurement.T() +
                                                              m_MeasureNoise).inv();
    m_State_k = m_State_estimated + m_Gain * (m_Observation - m_Measurement * m_State_estimated);
    int row = m_State_k.row;
    Matrix I = eye(row);
    m_StateVariance_k = (I - m_Gain * m_Measurement) * m_StateVariance_estimated *
                        (I - m_Gain * m_Measurement).T() + m_Gain * m_MeasureNoise * m_Gain.T();
    // 对系统状态进行转移，便于下一次滤波开始
    TransStateMatrix();
    TransStateVariance();
}

const Matrix &ns_filter::cKalman::getMStateK() const {
    return m_State_k.row==0 ? m_State_k_1 : m_State_k;
}

const Matrix &ns_filter::cKalman::getMStateVarianceK() const {
    return m_StateVariance_k.row==0 ? m_StateVariance_k_1 : m_StateVariance_k;
}

ns_filter::cKalman::cKalman() = default;
