//
// Created by 0-0 mashuo on 2023/6/1.
//

#include "cfileBase.h"


cFileBase::cFileBase() = default;


cFileBase::~cFileBase() {
    if(is_open()) close();
}

void cFileBase::close() {
    fileFp.close();
}

bool cFileBase::is_open() {
    return fileFp.is_open();
}

bool cFileBase::is_eof() {
    return fileFp.eof();
}

std::fstream &cFileBase::getFstream() {
    return fileFp;
}

bool cFileBase::open(const std::string &filename, const int &filetype) {
    auto mode = filetype == ASCII ? std::ios_base::in : (std::ios_base::in | std::ios_base::binary);
    fileFp.open(filename,mode);
    fileType = filetype;
    return is_open();
}
