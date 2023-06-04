//
// Created by 0-0 mashuo on 2023/6/1.
//

#include "cfileBase.h"


cFileBase::cFileBase() = default;


cFileBase::~cFileBase() {
    if(is_open()) close();
}

void cFileBase::close() {
    m_fileFp.close();
}

bool cFileBase::is_open() {
    return m_fileFp.is_open();
}

bool cFileBase::is_eof() {
    return m_fileFp.eof();
}

std::fstream &cFileBase::getFstream() {
    return m_fileFp;
}


