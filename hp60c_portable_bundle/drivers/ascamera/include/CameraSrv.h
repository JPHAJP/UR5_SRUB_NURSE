/**
 * @file      CameraSrv.h
 * @brief     angstrong camera service header.
 *
 * Copyright (c) 2023 Angstrong Tech.Co.,Ltd
 *
 * @author    Angstrong SDK develop Team
 * @date      2023/03/27
 * @version   1.0

 */
#pragma once

#include <list>
#include <functional>
#include <mutex>
#include "as_camera_sdk_api.h"

typedef struct CamSvrStreamParam {
    bool open;
    bool start;
    int image_flag;
} CamSvrStreamParam_s;

class ICameraStatus
{
public:
    virtual ~ICameraStatus() = default;
public:
    virtual int onCameraAttached(AS_CAM_PTR pCamera, CamSvrStreamParam_s &param, const AS_SDK_CAM_MODEL_E &cam_type) = 0;
    virtual int onCameraDetached(AS_CAM_PTR pCamera) = 0;
    virtual int onCameraOpen(AS_CAM_PTR pCamera) = 0;
    virtual int onCameraClose(AS_CAM_PTR pCamera) = 0;
    virtual int onCameraStart(AS_CAM_PTR pCamera) = 0;
    virtual int onCameraStop(AS_CAM_PTR pCamera) = 0;
    virtual void onCameraNewFrame(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData) = 0;
    virtual void onCameraNewMergeFrame(AS_CAM_PTR pCamera, const AS_SDK_MERGE_s *pstData) = 0;
};

class CameraSrv
{
public:
    CameraSrv(ICameraStatus *cameraStatus, const std::string &filepath);
    ~CameraSrv();

    CameraSrv(const CameraSrv &) = delete;
    CameraSrv &operator = (const CameraSrv &) = delete;

public:
    int start();
    void stop();
    std::timed_mutex &getLock()
    {
        return m_mutex;
    }

private:
    static void onAttached(AS_CAM_ATTR_S *attr, void *privateData);
    static void onDetached(AS_CAM_ATTR_S *attr, void *privateData);
    static void onNewFrame(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData, void *privateData);
    static void onNewMergeFrame(AS_CAM_PTR pCamera, const AS_SDK_MERGE_s *pstData, void *privateData);
    int getConfigFile(AS_CAM_PTR pCamera, std::string &configfile, AS_SDK_CAM_MODEL_E cam_type);
    int scanDir(const std::string &dir, std::vector<std::string> &file);

private:
    std::list<AS_CAM_PTR> m_devsList;
    ICameraStatus *m_camera_status;
    std::timed_mutex m_mutex;
    std::string m_configPath;
};

