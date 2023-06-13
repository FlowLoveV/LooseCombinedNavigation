//
// Created by 0-0 mashuo on 2023/6/12.
//


#include "cfileReader.h"
#include "sciplot/sciplot.hpp"
#include "iostream"
#include "yaml-cpp/include/yaml-cpp/yaml.h"
#include "TerminalMessage.h"

// 配置信息
YAML::Node config;

// 读取配置函数
void readConfig(const std::string & filename);

int main(int argc,char *argv[]){
    cfileReader reader("/Users/0-0mashuo/Desktop/Clion/CombinedNavigation/Dataset/遮挡LCNavRes.txt");
    std::cout << reader.measureLineWidth() << std::endl;

    return -1;
    /*using namespace sciplot;
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

    return -1;*/
}

void readConfig(const std::string & filename){
    try{
        config = YAML::LoadFile(filename);
    }
    catch (const std::exception & e){

    }
}


