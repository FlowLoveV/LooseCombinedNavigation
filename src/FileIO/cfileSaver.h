//
// Created by 0-0 mashuo on 2023/6/1.
//

#ifndef COMBINEDNAVIGATION_CFILESAVER_H
#define COMBINEDNAVIGATION_CFILESAVER_H

#include "cfileBase.h"

class cfileSaver : public cFileBase{
public:
    cfileSaver();

    cfileSaver(const std::string & filename,const int & type = ASCII,const std::string & format = "default");

    bool write();
};


#endif //COMBINEDNAVIGATION_CFILESAVER_H
