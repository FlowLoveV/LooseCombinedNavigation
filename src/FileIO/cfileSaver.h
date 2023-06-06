//
// Created by 0-0 mashuo on 2023/6/1.
//

#ifndef COMBINEDNAVIGATION_CFILESAVER_H
#define COMBINEDNAVIGATION_CFILESAVER_H

#include "vector"
#include "cfileBase.h"


class cfileSaver : public cFileBase{
public:
    cfileSaver();

    explicit cfileSaver(const std::string & filename, const int & type = 0, const int & format = 22);

    bool open(const std::string & filename,const int & type = 0);

    bool write(std::vector<double> vec);


protected:
    std::string m_sfileName;

private:
    std::vector<double> m_vdata;

    void cleardata();

    bool loadData(const std::vector<double> & vec);
};


#endif //COMBINEDNAVIGATION_CFILESAVER_H
