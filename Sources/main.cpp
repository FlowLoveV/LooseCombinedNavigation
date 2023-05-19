#include <QApplication>

#include "Headers/mainwindow.h"
#include "INSData.h"
#include "iostream"
#include <fstream>
#include <Matrix.h>

int main(int argc, char *argv[]) {
    const char * filename = "/Users/0-0mashuo/Desktop/Clion/CombinedNavigation/ObsData/IMU.bin";
    FILE *binfile = fopen(filename,"rb");
    if(binfile==NULL) ::std::cerr<<"can't open file"<<filename;
    double_t data[7];
    while(!feof(binfile)){
        fread(&data, 8, 7,binfile);
        printf("%.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",data[0],data[1],data[2],data[3],data[4],data[5],data[6]);
    }

    return 0;
    /*
    QApplication a(argc, argv);
    MainWindow w;

    w.show();
    return QApplication::exec();*/
    // test
}
