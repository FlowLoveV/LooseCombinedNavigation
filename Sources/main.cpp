

#include <QApplication>
#include "mainwindow.h"
#include <sciplot/sciplot.hpp>
#include "ncurses.h"

using namespace sciplot;




int main(int argc, char *argv[]) {
    using namespace std;


    // 结束ncurses

    return 0;


    /*const char * filename = "/Users/0-0mashuo/Desktop/Clion/CombinedNavigation/Dataset/IMU.bin";
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
