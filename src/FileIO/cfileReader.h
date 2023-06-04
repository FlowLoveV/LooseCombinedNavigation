//
// Created by 0-0 mashuo on 2023/6/1.
//

#ifndef COMBINEDNAVIGATION_CFILEREADER_H
#define COMBINEDNAVIGATION_CFILEREADER_H

#include <vector>
#include "cfileBase.h"


/**< 对读取文件格式的记录 */

/*!
 * NOVATEL OME7板卡输出数据，使用NOVATEL CONVERTOR 转换得到的 ASCII文本格式的IMU数据
 */
#define NOVATEL_OME7_IMU_ACSII "NOVATEL_OME7_IMU_ACSII"



class cfileReader : public cFileBase{
public:
    cfileReader();

    explicit cfileReader(const std::string &filename, const int type = cFileBase::ASCIITYPE, const std::string &format = "default");

    bool open(const std::string &filename,const int &filetype = cFileBase::ASCIITYPE);

    virtual std::vector<double>& readline(const int & col);

protected:
    std::string m_sfileName;

private:
    std::vector<double> m_vdata;

    void deleteData();
};


#endif //COMBINEDNAVIGATION_CFILEREADER_H
