
#include "TfTreeFrameIdInfo.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "Logger.h"

TfTreeFrameIdInfo::TfTreeFrameIdInfo(const std::string &nodeNameSpace, unsigned int index)
{
    m_idx = index;
    m_nodeNameSpace = nodeNameSpace;
    LOG(INFO) << "Namespace " <<  m_nodeNameSpace << std::endl;
}

TfTreeFrameIdInfo::~TfTreeFrameIdInfo()
{

}

std::string TfTreeFrameIdInfo::getColorFrameId()
{
    std::string str = m_nodeNameSpace + "_color_" + std::to_string(m_idx);

    if (str.compare(0, 1, "/") == 0) {
        str = str.substr(1);
    }

    return str;
}

std::string TfTreeFrameIdInfo::getDepthFrameId()
{
    std::string str = m_nodeNameSpace + "_depth_" + std::to_string(m_idx);

    if (str.compare(0, 1, "/") == 0) {
        str = str.substr(1);
    }

    return str;
}

std::string TfTreeFrameIdInfo::getDefaultFrameId()
{
    std::string str = m_nodeNameSpace + "_ascamera_" + std::to_string(m_idx);

    if (str.compare(0, 1, "/") == 0) {
        str = str.substr(1);
    }

    return str;
}

std::string TfTreeFrameIdInfo::getCamLinkFrameId()
{
    std::string str = m_nodeNameSpace + "_camera_link_" + std::to_string(m_idx);

    if (str.compare(0, 1, "/") == 0) {
        str = str.substr(1);
    }

    return str;
}

