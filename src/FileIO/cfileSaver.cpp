//
// Created by 0-0 mashuo on 2023/6/1.
//

#include "cfileSaver.h"
#include "iostream"

cfileSaver::cfileSaver() = default;

cfileSaver::cfileSaver(const std::string &filename, const int &type, const int &format) {
    open(filename,type);
    m_sfileName = filename;
    m_ifileFormat = format;
    if(!is_open()) std::cerr << "can't open file : " << filename << '\n';
}



bool cfileSaver::loadData(const std::vector<double> & vec) {
    if(vec.empty()) return false;
    else {
        cleardata();
        m_vdata = vec;
        return true;
    }
}

void cfileSaver::cleardata() {
    if(!m_vdata.empty()) m_vdata.clear();
}

bool cfileSaver::open(const std::string &filename, const int &type) {
    auto mode = type == 0 ? (std::ios_base::out) : (std::ios_base::out | std::ios_base::binary);
    m_fileFp.open(filename,mode);
    m_ifileType = type;
    return is_open();
}

bool cfileSaver::write(const std::vector<double> vec) {
    loadData(vec);
    // 将数据输出
    if(is_open()){
        for (const auto &item: m_vdata) {
            m_fileFp << item << " ";
        }
        m_fileFp << '\n';
        return true;
    }else return false;
}
