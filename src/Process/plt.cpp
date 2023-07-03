//
// Created by 0-0 mashuo on 2023/6/12.
//


#include "cfileReader.h"
#include "sciplot/sciplot.hpp"
#include "iostream"
#include "yaml-cpp/include/yaml-cpp/yaml.h"
#include "TerminalMessage.h"

// 模式1作图函数
void mode1_plt(const std::string & filename);
// 模式2作图函数
void mode2_lpt(const std::string & filename1,const std::string & filename2);
// 模式3作图函数
void mode3_plt(const std::string & filename);

int main(int argc,char *argv[]){
    // 参数数目检验
    if(argc!=2){
        TerminalMessage::displayErrorMessage("参数数目错误!请输入两个参数");
    }
    // 配置信息
    YAML::Node config;
    // 文件行数
    double Rows[2];
    // 文件列数
    double Cols[2];
    try{
        config = YAML::LoadFile(argv[1]);
    }
    catch (const std::exception & e){
        TerminalMessage::displayErrorMessage("配置文件读取失败!请检查配置文件路径或者格式");
    }
    cfileReader reader1,reader2;
    int mode;
    // 根据模式读取文件
    try{
        mode = config["mode"].as<int>();
    }catch (const std::exception & e){
        TerminalMessage::displayErrorMessage("配置模式设置有误!请重新设置");
    }
    switch (mode) {
        case 1:
            break;
        case 2:
            break;
        case 3:
            break;
        default:
            TerminalMessage::displayErrorMessage("作图模式只能选择1、2或者3!");
    }


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



