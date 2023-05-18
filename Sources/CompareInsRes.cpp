//
// Created by 0-0 mashuo on 2023/5/18.
//

// 此文件用于对比INS结算结果

#include "iostream"
#include "ostream"
#include "fstream"
#include "filesystem"
#include "TimeSys.h"

int main(int argc,char* argv[]){
    // 参数管理
    if(argc!=4) {
        system("echo \"\\033[1;31mWrong number of arguments！\" ");
    }
    ::std::fstream fin1(argv[1]),fin2(argv[2]);
    ::std::string  error = "echo \"\\033[1;31merror in DiffInsRes() ";
    // 容错处理
    if(!fin1.is_open()){
        ::std::string filedir(argv[1]);
        ::std::string cmd = error + " can't open file " + filedir + "\"";
        const char * command = cmd.c_str();
        system(command);
        return -1;
    }
    if(!fin2.is_open()){
        ::std::string filedir(argv[2]);
        ::std::string cmd = error + + " can't open file " + filedir + "\"";
        const char * command = cmd.c_str();
        system(command);
        return -1;
    }
    namespace fs = std::filesystem;
    ::std::string filepath;
    if(::std::filesystem::is_directory(argv[3])){
        // 如果只给出了路径，则需要系统给出命名，这里采用获得时间的命名方法
        ::std::tm  tm = getCurrentTime();
        char buffer[80];
        // 格式化时间
        strftime(buffer, 80, "%m%d-%H-%M-%S", &tm);
        // ::std::string time = ::std::put_time(&tm,"%Y-%m-%d-%H-%M-%S");
        filepath = "/DiffInsRes" + ::std::string(buffer);
    }
    else if(::std::filesystem::is_regular_file(argv[3])){
        filepath = argv[3];
    }
    else{
        ::std::string cmd = error + argv[3] + "is neither a directory nor a regular file \"";
        return -1;
    }
    ::std::ofstream out(filepath);
    // 开始进行结果数据的差分

    ::std::string str  = "echo \"\\033[1;32mDone!\nThe Diff data has been written to " + filepath + "\"";
    const char * done_cmd = str.c_str();
    system(done_cmd);
    return 0;
}

