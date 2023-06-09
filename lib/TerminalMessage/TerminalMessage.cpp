//
// Created by 0-0 mashuo on 2023/6/9.
//

#include "TerminalMessage.h"


void TerminalMessage::displayColoredMessage(const std::string &message, const std::string &colorCode) {
    std::cout << "\033[" << colorCode << "m" << message << "\033[0m" << std::endl;
}

