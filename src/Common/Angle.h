//
// Created by 0-0 mashuo on 2023/6/2.
//

#ifndef COMBINEDNAVIGATION_ANGLE_H
#define COMBINEDNAVIGATION_ANGLE_H

#define MY_PROJECT_PI 3.141592653589793
#define RAD2DEG 57.295779513082323
#define DEG2RAD 0.017453292519943

/*!
 * 弧度转角度函数
 * @param rad   input       double      弧度[rad]
 * @return                  double      角度[百分度]
 */
static double rad2deg(const double & rad){ return RAD2DEG * rad;}

/*!
 * 弧度转角度函数
 * @param rad   input       float      弧度[rad]
 * @return                  float      角度[百分度]
 */
static float rad2deg(const float & rad){ return RAD2DEG * rad;}


/*!
 * 角度转弧度函数
 * @param deg   input       double      角度[百分度]
 * @return                  double      弧度[rad]
 */
static double deg2rad(const double & deg){ return DEG2RAD * deg;}


/*!
 * 角度转弧度函数
 * @param deg   input       float      角度[百分度]
 * @return                  float      弧度[rad]
 */
static float deg2rad(const float & deg){ return DEG2RAD * deg;}


/*!
 * 对可迭代容器或者普通数组实现弧度转角度
 * @tparam T    double[],std::vector<double>,std::array<>等
 * @param rad   input/output    T       输入的弧度数组或者容器，输出角度
 */
template <class T>
void rad2deg(T & rad){
    for (auto begin = std::begin(rad); begin != std::end(rad); ++begin) {
        *begin *= RAD2DEG;
    }
}

/*!
 * 对可迭代容器或者普通数组实现角度转弧度
 * @tparam T    double[],std::vector<double>,std::array<>等
 * @param deg   input/output    T       输入的角度数组或者容器，输出弧度
 */
template <class T>
void deg2rad(T & deg){
    for (auto begin = std::begin(deg); begin != std::end(deg); ++begin) {
        *begin *= DEG2RAD;
    }
}



#endif //COMBINEDNAVIGATION_ANGLE_H
