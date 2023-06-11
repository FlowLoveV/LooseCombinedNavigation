

#include <QApplication>
#include "mainwindow.h"
#include <sciplot/sciplot.hpp>

#include "INSType.h"
#include "Rotation.h"
#include "cfileSaver.h"
#include "iostream"


int main(int argc, char *argv[]) {


    // 验证惯导程序是否正确
    FILE *imufile = std::fopen("/Users/0-0mashuo/Desktop/Clion/CombinedNavigation/Dataset/IMU.bin", "rb");
    FILE *resfile = std::fopen("/Users/0-0mashuo/Desktop/Clion/CombinedNavigation/Dataset/PureINS.bin", "rb");
    if (!imufile || !resfile) {
        std::cerr << "无法打开文件!";
    }

    // lambda构造imu数据，ins结果类
    IMUData_SingleEpoch imudata;
    INSRes_SingleEpoch insres;
    auto createIMU = [](IMUData_SingleEpoch &imu, const double *p) {
        imu.t = GPST(0, p[0]);
        memcpy(imu.m_pGyr, p + 1, 3 * sizeof(double));
        memcpy(imu.m_pAcc, p + 4, 3 * sizeof(double));
    };

    auto createRes = [](INSRes_SingleEpoch &res, const double *p) {
        res.t = GPST(0, p[0]);
        memcpy(res.m_pPos, p + 1, 3 * sizeof(double));
        memcpy(res.m_pSpeed, p + 4, 3 * sizeof(double));
        memcpy(res.m_pEuler, p + 7, 3 * sizeof(double));
    };

    std::vector<double> compared(11);
    auto compareRes = [&compared](INSRes_SingleEpoch &my, double *ref) {
        if (ref[9] < 0) {
            ref[9] += 360;
        }
        my.changeUnitR2D();
        compared[0] = my.t.second;
        compared[1] = ref[0];
        compared[2] = my.m_pPos[0] - ref[1];
        compared[3] = my.m_pPos[1] - ref[2];
        compared[4] = my.m_pPos[2] - ref[3];
        compared[5] = my.m_pSpeed[0] - ref[4];
        compared[6] = my.m_pSpeed[1] - ref[5];
        compared[7] = my.m_pSpeed[2] - ref[6];
        compared[8] = my.m_pEuler[0] - ref[7];
        compared[9] = my.m_pEuler[1] - ref[8];
        compared[10] = my.m_pEuler[2] - ref[9];
        my.changeUnitD2R();
    };


    double data[7], res[10];
    // imu数据同步
    do {
        fread(data, 8, 7, imufile);
    } while (data[0] < 91620.0);
    // res数据同步
    do {
        fread(res, 8, 10, resfile);
    } while (res[0] < 91620.0);

    std::vector<IMUData_SingleEpoch> vdata(3);
    std::vector<INSRes_SingleEpoch> vres(3);
    createIMU(imudata, data);
    double init[10] = {91620.0, 23.1373950708, 113.3713651222, 2.175,
                       0, 0, 0, 0.0107951084511778, -2.14251290749072, -75.7498049314083};
    createRes(insres, init);

    insres.changeUnitD2R();
    Rotation::Euler2EMatrix(insres.m_pEuler, insres.m_pEMatrix);
    Rotation::Euler2Quaternion(insres.m_pEuler, insres.m_pQuaternion);
    vdata.assign(3, imudata);
    vres.assign(3, insres);

    cfileSaver saver("/Users/0-0mashuo/Desktop/Clion/CombinedNavigation/Dataset/compare.txt");
    while (true) {
        if (std::feof(imufile) || std::feof(resfile)) {
            break;
        }
        fread(data, 8, 7, imufile);
        vdata[0] = vdata[1], vdata[1] = vdata[2], createIMU(vdata[2], data);
        vres[0] = vres[1], vres[1] = vres[2], vres[2].t = vdata[2].t;
        PureIns::updateSinEpoch(vdata[0], vdata[1], vdata[2],
                                vres[0], vres[1], vres[2]);
        compareRes(vres[2], res);
        saver.write(compared);
        fread(res, 8, 10, resfile);
    }

    return 0;
    // qt 界面 最后编写
    /*
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return QApplication::exec();*/
}
