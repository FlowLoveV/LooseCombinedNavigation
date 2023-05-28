#include <QApplication>

#include "Headers/mainwindow.h"
#include "INSData.h"
#include "iostream"
#include <fstream>
#include <Matrix.h>
#include <sciplot/sciplot.hpp>
using namespace sciplot;

int main(int argc, char *argv[]) {
    // test
    // Create values for your x-axis
    Vec x = linspace(0.0, 5.0, 100);

    // Create a Plot object
    Plot2D plot1;
    // Set color palette for first Plot
    plot1.palette("paired");
    // Draw a sine graph putting x on the x-axis and sin(x) on the y-axis
    plot1.drawCurve(x, std::sin(x)).label("sin(x)").lineWidth(4);
    // Draw a cosine graph putting x on the x-axis and cos(x) on the y-axis
    plot1.drawCurve(x, std::cos(x)).label("cos(x)").lineWidth(2);

    // Create a second Plot object
    Plot2D plot2;
    // Draw a tangent graph putting x on the x-axis and tan(x) on the y-axis
    plot2.drawCurve(x, std::tan(x)).label("tan(x)").lineWidth(4);

    // Put both plots in a "figure" horizontally next to each other
    Figure figure = {{plot1, plot2}};

    // Create a canvas / drawing area to hold figure and plots
    Canvas canvas = {{figure}};
    // Set color palette for all Plots that do not have a palette set (plot2) / the default palette
    canvas.defaultPalette("set1");

    // Show the canvas in a pop-up window
    canvas.show();

    // Save the plot to a SVG file
    canvas.save("example-readme.svg");

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
