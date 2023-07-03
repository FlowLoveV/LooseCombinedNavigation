//
// Created by 0-0 mashuo on 2023/6/16.
//

#ifndef COMBINEDNAVIGATION_ERROREVALUTION_H
#define COMBINEDNAVIGATION_ERROREVALUTION_H

#include "iostream"
#include "vector"
#include "TerminalMessage.h"


#include <iostream>
#include <cmath>
#include <vector>

class ErrorEvaluation {
public:
    // 构造函数，接受普通数组和数组长度
    ErrorEvaluation(const double* data, int size) {
        values.assign(data, data + size);
    }

    // 构造函数，接受标准模板库数组
    ErrorEvaluation(const std::vector<double>& data) : values(data) {}

    // 计算均值
    double calculateMean() const {
        double sum = 0.0;
        for (const auto& value : values) {
            sum += value;
        }
        return sum / values.size();
    }

    // 计算方差
    double calculateVariance() const {
        double mean = calculateMean();
        double variance = 0.0;
        for (const auto& value : values) {
            variance += (value - mean) * (value - mean);
        }
        return variance / values.size();
    }

    // 计算均方误差
    double calculateMeanSquaredError(const double* referenceData, int size) const {
        double mse = 0.0;
        for (int i = 0; i < size; ++i) {
            double error = values[i] - referenceData[i];
            mse += error * error;
        }
        return mse / size;
    }

    // 计算平均偏差
    double calculateMeanDeviation(const double* referenceData, int size) const {
        double deviation = 0.0;
        for (int i = 0; i < size; ++i) {
            deviation += std::abs(values[i] - referenceData[i]);
        }
        return deviation / size;
    }

private:
    std::vector<double> values;
};


#endif //COMBINEDNAVIGATION_ERROREVALUTION_H
