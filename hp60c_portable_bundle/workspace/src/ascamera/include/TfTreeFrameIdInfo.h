/**
 * @file      TfTreeFrameIdInfo.cpp
 * @brief     tf tree topic frameId info.
 *
 * Copyright (c) 2024 Angstrong Tech.Co.,Ltd
 *
 * @author    Angstrong SDK develop Team
 * @date      2024/01/08
 * @version   1.0

 */
#pragma once

#include <string>
class TfTreeFrameIdInfo
{
public:
    TfTreeFrameIdInfo(const std::string &nodeNameSpace, unsigned int index);
    ~TfTreeFrameIdInfo();

    std::string getColorFrameId();
    std::string getDepthFrameId();
    std::string getDefaultFrameId();
    std::string getCamLinkFrameId();

private:
    unsigned int m_idx;
    std::string m_nodeNameSpace;

};