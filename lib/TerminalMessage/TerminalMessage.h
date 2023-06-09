//
// Created by 0-0 mashuo on 2023/6/9.
//

#ifndef COMBINEDNAVIGATION_TERMINALMESSAGE_H
#define COMBINEDNAVIGATION_TERMINALMESSAGE_H

#include "iostream"

class TerminalMessage{
public:
    static void displayErrorMessage(const std::string& message) {
        // 设置红色文本颜色代码：31
        displayColoredMessage(message, "31");
    }

    static void displayWarningMessage(const std::string& message) {
        // 设置黄色文本颜色代码：33
        displayColoredMessage(message, "33");
    }

    static void displaySuccessMessage(const std::string& message) {
        // 设置绿色文本颜色代码：32
        displayColoredMessage(message, "32");
    }


private:
    static void displayColoredMessage(const std::string& message, const std::string& colorCode);
};


#endif //COMBINEDNAVIGATION_TERMINALMESSAGE_H

