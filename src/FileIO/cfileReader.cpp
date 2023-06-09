//
// Created by 0-0 mashuo on 2023/6/1.
//

#include <iostream>
#include "cfileReader.h"
#include "QString"
#include "QRegularExpression"
#include "QDebug"


cfileReader::cfileReader(const std::string &filename, const int type, const int &format) {
    open(filename,type);
    m_sfileName = filename;
    m_ifileFormat = format;
    if(!is_open()) std::cerr << "can't open file : " << filename << '\n';
}

cfileReader::cfileReader() = default;

void cfileReader::deleteData() {
    if(!m_vdata.empty()) m_vdata.clear();
}


std::vector<double> &cfileReader::readline(const int & col) {
    // 首先删除已存数据
    deleteData();
    // 到达文件结尾，则停止读取，返回空数组
    if(is_eof()){
        return m_vdata;
    }
    // 预留col空间
    m_vdata.reserve(col);
    if(m_ifileType == cFileBase::ASCIITYPE){ // ASCII文本格式
        std::string line;
        std::getline(m_fileFp, line);
        if(line.empty()) {
            deleteData();
            return m_vdata;
        }
        try{
            // 这里使用QT中的string分割方法来分割字符串
            QString str = QString::fromStdString(line);
            QRegularExpression RegExp("[, ]+");  // 这里使用正则表达式来定义分隔符
            QStringList parts = str.split(RegExp);
            for (const auto &item: parts) {
                if(!item.isEmpty() && !item.contains(" "))
                    m_vdata.push_back(std::stod(item.toStdString()));
            }

        }
        catch(const std::exception & e){
            deleteData();
            std::cerr << "读取" << m_sfileName << "时发生错误!\n" << "请检验文件格式是否正确。";
            return m_vdata;
        }
    }else{
        // 二进制文本格式
        m_fileFp.read((char*)m_vdata.data(), sizeof(double) * col);
    }
    return m_vdata;
}

bool cfileReader::open(const std::string &filename, const int &filetype) {
    auto mode = filetype == cFileBase::ASCIITYPE ? std::ios_base::in : (std::ios_base::in | std::ios_base::binary);
    m_fileFp.open(filename, mode);
    m_ifileType = filetype;
    return is_open();
}

