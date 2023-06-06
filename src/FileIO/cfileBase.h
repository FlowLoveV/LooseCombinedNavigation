//
// Created by 0-0 mashuo on 2023/6/1.
//

#ifndef COMBINEDNAVIGATION_CFILEBASE_H
#define COMBINEDNAVIGATION_CFILEBASE_H

#include "fstream"

class cFileBase{
public:
    static const int ASCIITYPE = 0;         /**< 表示待读取文件格式为ASCII文本 */
    static const int BINARYTYPE = 1;        /**< 表示待读取文件格式为二进制 */

    cFileBase();

    ~cFileBase();

    void close();

    bool is_open();

    bool is_eof();

    std::fstream & getFstream();

protected:
    int m_ifileFormat;             /**< 记录文件的编码格式       */
    std::fstream m_fileFp;                 /**< 文件流                 */
    int m_ifileType = cFileBase::ASCIITYPE;               /**< 文件格式ASCII文本或者二进制 */
};

#endif //COMBINEDNAVIGATION_CFILEBASE_H
