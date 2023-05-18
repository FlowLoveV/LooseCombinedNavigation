#include <QApplication>

#include "Headers/mainwindow.h"
#include "INSData.h"
#include "iostream"
#include <iostream>
#include <fstream>
#include <string>
#include <Matrix.h>


int main(int argc, char *argv[]) {


    InsConfigure testConfigure;
    ::std::string filename = "/Users/0-0mashuo/Desktop/Clion/CombinedNavigation/ObsData/粗对准.ASC";
    testConfigure.setImuFileDir(filename);
    double_t pos[3] = {30.531651244,0,28.2134};
    testConfigure.setStartPos(pos);
    PureIns testSolver;
    testSolver.standardINSSolver(testConfigure);
    return 0;

    /*
    QApplication a(argc, argv);
    MainWindow w;

    w.show();
    return QApplication::exec();*/
    // test


}
