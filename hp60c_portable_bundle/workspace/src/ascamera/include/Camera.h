/**
 * @file      Camera.h
 * @brief     angstrong camera.
 *
 * Copyright (c) 2023 Angstrong Tech.Co.,Ltd
 *
 * @author    Angstrong SDK develop Team
 * @date      2023/05/29
 * @version   1.0

 */
#pragma once
#include <chrono>
#include <thread>
#include "Logger.h"
#include "as_camera_sdk_api.h"
#include "common.h"
#include "TfTreeFrameIdInfo.h"
#include "rclcpp/rclcpp.hpp"

class CheckFps
{
public:
    CheckFps(AS_CAM_PTR pCamera)
    {
        m_pCamera = pCamera;
    };
    ~CheckFps() {};
public:
    double checkFps(const std::string &sn, const std::string &Info)
    {
        double fps = 0.0;
        auto t_cur = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_cur - t_last).count();
        if (duration > m_duration) {
            fps = frameCount / (duration / 1000);
            frameCount = 0;
            t_last = t_cur;
            LOG(INFO) << "#camera[" << Info << "] SN[" << sn << "]'s FrameRate:" << fps << std::endl;
        }
        ++frameCount;
        return fps;
    }
private:
    AS_CAM_PTR m_pCamera;
    unsigned int frameCount = 0;
    std::chrono::steady_clock::time_point t_last;
    const unsigned int m_duration = 2000; /* ms */
};

class Camera
{
public:
    Camera(AS_CAM_PTR pCamera, const AS_SDK_CAM_MODEL_E &cam_type, const std::string &nodeNameSpace, unsigned int devId,
           rclcpp::Logger logger);
    ~Camera();
public:
    int init();
    double checkFps();
    int enableSaveImage(bool enable);
    int getSerialNo(std::string &sn);
    int getCamParameter(AS_CAM_Parameter_s &cam_parameter);
    int getCameraAttrs(AS_CAM_ATTR_S &attr);
    void saveImage(const AS_SDK_Data_s *pstData);
    void saveMergeImage(const AS_SDK_MERGE_s *pstData);
    std::string getColorFrameId();
    std::string getDepthFrameId();
    std::string getDefaultFrameId();
    std::string getCamLinkFrameId();

private:
    int backgroundThread();

private:
    AS_CAM_PTR m_handle = nullptr;
    std::string m_serialno;
    CheckFps *m_check_fps = nullptr;
    bool m_save_img = false;
    bool m_save_merge_img = false;
    AS_CAM_ATTR_S m_attr;
    AS_CAM_Parameter_s m_cam_parameter;
    AS_SDK_CAM_MODEL_E m_cam_type = AS_SDK_CAM_MODEL_UNKNOWN;
    int m_cnt = 0; /* for kunlun a to save odd even */
    int m_depthindex = 0;
    int m_rgbindex = 0;
    int m_pointCloudIndex = 0;
    int m_irindex = 0;
    int m_peakindex = 0;
    bool m_is_thread = false;
    std::thread m_backgroundThread;
    bool m_isGetParameter = false;
    TfTreeFrameIdInfo *m_frameIdInfo = nullptr;
    rclcpp::Logger m_logger;
};
