#include <QApplication>

#include "mainwindow.h"
#include "INSData.h"
#include "iostream"
#include <fstream>
#include <Matrix.h>
#include <sciplot/sciplot.hpp>
using namespace sciplot;

int main(int argc, char *argv[]) {
    // sciplot使用示例
    Vec x = linspace(0.0, 5.0, 200);
    Plot2D plot0;
    plot0.drawCurve(x, std::sin(x)).label("sin(x)");
    Plot2D plot1;
    plot1.drawCurve(x, std::cos(x)).label("cos(x)");

    // Use the previous plots as sub-figures in a larger figure. Note that
    // plot0 and plot1 will be deep-copied into fig
    Figure fig = {{plot0, plot1}};
    Canvas canvas = {{fig}};
    Canvas canvas1 = {{fig}};
    canvas.size(750, 750);
    canvas.title("dasdas");
    canvas.show();
    canvas1.show();

    /*const char * filename = "/Users/0-0mashuo/Desktop/Clion/CombinedNavigation/ObsData/IMU.bin";
    FILE *binfile = fopen(filename,"rb");
    if(binfile==NULL) ::std::cerr<<"can't open file"<<filename;
    double_t data[7];
    while(!feof(binfile)){
        fread(&data, 8, 7,binfile);
        printf("%.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",data[0],data[1],data[2],data[3],data[4],data[5],data[6]);
    }*/



    // qt 界面 最后编写
    /*
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return QApplication::exec();*/
}
