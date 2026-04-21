/**
 * @file      Camera.cpp
 * @brief     angstrong Camera.
 *
 * Copyright (c) 2023 Angstrong Tech.Co.,Ltd
 *
 * @author    Angstrong SDK develop Team
 * @date      2023/05/29
 * @version   1.0

 */
#include <algorithm>
#include <unistd.h>
#include "Camera.h"

Camera::Camera(AS_CAM_PTR pCamera, const AS_SDK_CAM_MODEL_E &cam_type, const std::string &nodeNameSpace,
               unsigned int devId, rclcpp::Logger logger)
    : m_logger(logger)
{
    int ret = 0;
    m_handle = pCamera;
    m_cam_type = cam_type;
    m_check_fps = new CheckFps(pCamera);
    ret = AS_SDK_GetCameraAttrs(m_handle,  m_attr);
    if (ret != 0) {
        RCLCPP_WARN_STREAM(m_logger, "get camera attrs failed");
    }
    memset(&m_cam_parameter, 0, sizeof(AS_CAM_Parameter_s));

    m_frameIdInfo = new TfTreeFrameIdInfo(nodeNameSpace, devId);
}

Camera::~Camera()
{
    if (m_check_fps != nullptr) {
        delete m_check_fps;
        m_check_fps = nullptr;
    }
    if (m_is_thread) {
        m_is_thread = false;
    }
    if (m_backgroundThread.joinable()) {
        m_backgroundThread.join();
    }

    delete m_frameIdInfo;
}

int Camera::init()
{
    int ret = 0;
    char sn_buff[64] = {0};
    ret = AS_SDK_GetSerialNumber(m_handle, sn_buff, sizeof(sn_buff));
    if (ret != 0) {
        RCLCPP_ERROR_STREAM(m_logger, "get camera serial number failed");
        return -1;
    }
    m_serialno = std::string(sn_buff);

    char fwVersion[100] = {0};
    ret = AS_SDK_GetFwVersion(m_handle, fwVersion, sizeof(fwVersion));
    if (ret == 0) {
        RCLCPP_INFO_STREAM(m_logger, "#camera[" << m_handle << "] SN[" << m_serialno << "]'s firmware version:" << fwVersion);
    }
    m_is_thread = true;
    m_backgroundThread = std::thread(&Camera::backgroundThread, this);
    return ret;
}

double Camera::checkFps()
{
    std::string Info = "";
    switch (m_attr.type) {
    case AS_CAMERA_ATTR_LNX_USB:
        Info = (std::to_string(m_attr.attr.usbAttrs.bnum) + ":" + m_attr.attr.usbAttrs.port_numbers);
        break;
    case AS_CAMERA_ATTR_NET:
        Info = (std::to_string(m_attr.attr.netAttrs.port) + ":" + m_attr.attr.netAttrs.ip_addr);
        break;
    case AS_CAMERA_ATTR_WIN_USB:
        Info = std::string(m_attr.attr.winAttrs.symbol_link) + ":" + std::string(m_attr.attr.winAttrs.location_path);
        break;
    default:
        RCLCPP_ERROR_STREAM(m_logger,  "attr type error");
        break;
    }
    return m_check_fps->checkFps(m_serialno, Info);
}

int Camera::enableSaveImage(bool enable)
{
    m_save_img = enable;
    if (m_cam_type == AS_SDK_CAM_MODEL_KUNLUN_A) {
        m_save_merge_img = enable;
    }
    return 0;
}

int Camera::getSerialNo(std::string &sn)
{
    sn = m_serialno;
    return 0;
}

int Camera::getCameraAttrs(AS_CAM_ATTR_S &info)
{
    info = m_attr;
    return 0;
}

int Camera::getCamParameter(AS_CAM_Parameter_s &cam_parameter)
{
    if (m_isGetParameter) {
        memcpy(&cam_parameter, &m_cam_parameter, sizeof(AS_CAM_Parameter_s));
        return 0;
    } else {
        return -1;
    }
}

int Camera::backgroundThread()
{
    int ret = 0;
    while (m_is_thread) {
        // KONDYOR not support to get CamParameter, KUNLUN A don't need to get CamParameter
        if ((m_cam_type != AS_SDK_CAM_MODEL_KONDYOR_NET) && (m_cam_type != AS_SDK_CAM_MODEL_KONDYOR)) {
            ret = AS_SDK_GetCamParameter(m_handle, &m_cam_parameter);
            if (ret == 0) {
                RCLCPP_INFO_STREAM(m_logger, "SN [ " << m_serialno << " ]'s parameter:");
                RCLCPP_INFO_STREAM(m_logger, "irfx: " << m_cam_parameter.fxir);
                RCLCPP_INFO_STREAM(m_logger, "irfy: " << m_cam_parameter.fyir);
                RCLCPP_INFO_STREAM(m_logger, "ircx: " << m_cam_parameter.cxir);
                RCLCPP_INFO_STREAM(m_logger, "ircy: " << m_cam_parameter.cyir);
                RCLCPP_INFO_STREAM(m_logger, "rgbfx: " << m_cam_parameter.fxrgb);
                RCLCPP_INFO_STREAM(m_logger, "rgbfy: " << m_cam_parameter.fyrgb);
                RCLCPP_INFO_STREAM(m_logger, "rgbcx: " << m_cam_parameter.cxrgb);
                RCLCPP_INFO_STREAM(m_logger, "rgbcy: " << m_cam_parameter.cyrgb << std::endl);
                m_is_thread = false;
                m_isGetParameter = true;
                break;
            }
        } else {
            m_isGetParameter = true;
            m_is_thread = false;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    return 0;
}

void Camera::saveImage(const AS_SDK_Data_s *pstData)
{
    if (!m_save_img) {
        m_cnt = 0;
        return;
    }

    if (m_cam_type == AS_SDK_CAM_MODEL_KUNLUN_A) {
        if (m_cnt >= 1) {
            m_save_img = false;
        }
        m_cnt++;
    } else {
        m_save_img = false;
    }

    if (pstData->depthImg.size > 0) {
        std::string depthimgName(std::string(m_serialno + "_depth_") + std::to_string(
                                     pstData->depthImg.width) + "x" + std::to_string(pstData->depthImg.height)
                                 + "_" + std::to_string(m_depthindex++) + ".yuv");
        if (saveYUVImg(depthimgName.c_str(), pstData->depthImg.data, pstData->depthImg.size) != 0) {
            RCLCPP_ERROR_STREAM(m_logger, "save depth image failed!");
        } else {
            RCLCPP_INFO_STREAM(m_logger, "save depth image success!");
            RCLCPP_INFO_STREAM(m_logger, "location: " << getcwd(nullptr, 0) << "/" << depthimgName);
        }
    }

    if (pstData->rgbImg.size > 0) {
        std::string rgbName(std::string(m_serialno + "_rgb_") + std::to_string(pstData->rgbImg.width) + "x" +
                            std::to_string(pstData->rgbImg.height) + "_" + std::to_string(m_rgbindex++) + ".yuv");
        if (saveYUVImg(rgbName.c_str(), pstData->rgbImg.data, pstData->rgbImg.size) != 0) {
            RCLCPP_ERROR_STREAM(m_logger, "save rgb image failed!");
        } else {
            RCLCPP_INFO_STREAM(m_logger, "save rgb image success!");
            RCLCPP_INFO_STREAM(m_logger, "location: " << getcwd(nullptr, 0) << "/" << rgbName);
        }
    }

    if (pstData->pointCloud.size > 0) {
        std::string pointCloudName(std::string(m_serialno + "_PointCloud_"  + std::to_string(
                pstData->pointCloud.width) + "x" + std::to_string(pstData->pointCloud.height)
                                               + "_"  + std::to_string(m_pointCloudIndex++) + ".pcd"));
        if (savePointCloudWithPcdFormat(pointCloudName.c_str(), static_cast<float *>(pstData->pointCloud.data),
                                        pstData->pointCloud.size / sizeof(float)) != 0) {
            RCLCPP_ERROR_STREAM(m_logger, "save point cloud failed!");
        } else {
            RCLCPP_INFO_STREAM(m_logger, "save point cloud success!");
            RCLCPP_INFO_STREAM(m_logger, "location: " << getcwd(nullptr, 0) << "/" << pointCloudName);
        }
    }

    if (pstData->irImg.size > 0) {
        std::string irimgName(std::string(m_serialno + "_ir_" + std::to_string(pstData->irImg.width) + "x" +
                                          std::to_string(pstData->irImg.height) + "_" + std::to_string(m_irindex++) + ".yuv"));
        if (saveYUVImg(irimgName.c_str(), pstData->irImg.data, pstData->irImg.size) != 0) {
            RCLCPP_ERROR_STREAM(m_logger,  "save ir image failed!");
        } else {
            RCLCPP_INFO_STREAM(m_logger, "save ir image success!");
            RCLCPP_INFO_STREAM(m_logger, "location: " << getcwd(nullptr, 0) << "/" << irimgName);
        }
    }

    if (pstData->peakImg.size > 0) {
        std::string peakimgName(std::string(m_serialno + "_peak_") + std::to_string(
                                    pstData->peakImg.width) + "x" + std::to_string(pstData->peakImg.height)
                                + "_" + std::to_string(m_peakindex++) + ".yuv");
        if (saveYUVImg(peakimgName.c_str(), pstData->peakImg.data, pstData->peakImg.size) != 0) {
            RCLCPP_ERROR_STREAM(m_logger, "save peak image failed!");
        } else {
            RCLCPP_INFO_STREAM(m_logger, "save peak image success!");
            RCLCPP_INFO_STREAM(m_logger, "location: " << getcwd(nullptr, 0) << "/" << peakimgName);
        }
    }

    return;
}

void Camera::saveMergeImage(const AS_SDK_MERGE_s *pstData)
{
    if (!m_save_merge_img) {
        return;
    }
    m_save_merge_img = false;
    if (pstData->depthImg.size > 0) {
        std::string depthimgName(std::string(m_serialno + "_depth_merge_") + std::to_string(
                                     pstData->depthImg.width) + "x" + std::to_string(pstData->depthImg.height)
                                 + "_" + std::to_string(m_depthindex++) + ".yuv");
        if (saveYUVImg(depthimgName.c_str(), pstData->depthImg.data, pstData->depthImg.size) != 0) {
            RCLCPP_ERROR_STREAM(m_logger, "save depth image failed!");
        } else {
            RCLCPP_INFO_STREAM(m_logger, "save depth image success!");
            RCLCPP_INFO_STREAM(m_logger, "location: " << getcwd(nullptr, 0) << "/" << depthimgName);
        }
    }

    if (pstData->pointCloud.size > 0) {
        std::string pointCloudName(std::string(m_serialno + "_PointCloud_merge_"  + std::to_string(
                pstData->pointCloud.width) + "x" + std::to_string(pstData->pointCloud.height)
                                               + "_"  + std::to_string(m_pointCloudIndex++) + ".pcd"));
        if (savePointCloudWithPcdFormat(pointCloudName.c_str(), static_cast<float *>(pstData->pointCloud.data),
                                        pstData->pointCloud.size / sizeof(float)) != 0) {
            RCLCPP_ERROR_STREAM(m_logger, "save point cloud failed!");
        } else {
            RCLCPP_INFO_STREAM(m_logger, "save point cloud success!");
            RCLCPP_INFO_STREAM(m_logger, "location: " << getcwd(nullptr, 0) << "/" << pointCloudName);
        }
    }

    return;
}


std::string Camera::getColorFrameId()
{
    return m_frameIdInfo->getColorFrameId();
}

std::string Camera::getDepthFrameId()
{
    return m_frameIdInfo->getDepthFrameId();
}

std::string Camera::getDefaultFrameId()
{
    return m_frameIdInfo->getDefaultFrameId();
}

std::string Camera::getCamLinkFrameId()
{
    return m_frameIdInfo->getCamLinkFrameId();
}

