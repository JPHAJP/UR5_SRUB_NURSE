/**
 * @file      CameraSrv.cpp
 * @brief     angstrong camera service.
 *
 * Copyright (c) 2023 Angstrong Tech.Co.,Ltd
 *
 * @author    Angstrong SDK develop Team
 * @date      2023/02/15
 * @version   1.0

 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <thread>
#include <sys/time.h>
#include <signal.h>
#include <malloc.h>
#include <string.h>
#include <iostream>
#include <string>
#include <dirent.h>

#include "Logger.h"
#include "as_camera_sdk_api.h"
#include "common.h"
#include "CameraPublisher.h"
#include "CameraSrv.h"

CameraSrv::CameraSrv(ICameraStatus *cameraStatus, const std::string &filepath) : m_camera_status(cameraStatus)
{
    int ret = 0;
    LOG(INFO) << "Angstrong camera server" << std::endl;
    ret = AS_SDK_Init();
    if (ret != 0) {
        LOG(ERROR) << "sdk init failed" << std::endl;
    }
    char sdkVersion[64] = {0};
    ret = AS_SDK_GetSwVersion(sdkVersion, sizeof(sdkVersion));
    if (ret != 0) {
        LOG(ERROR) << "get sdk version failed" << std::endl;
    }
    LOG(INFO) << "Angstrong camera sdk version:" << sdkVersion << std::endl;

    m_configPath = filepath;
}

CameraSrv::~CameraSrv()
{
    LOG(INFO) << "Angstrong camera server exit" << std::endl;
    int ret = AS_SDK_Deinit();
    if (ret != 0) {
        LOG(ERROR) << "sdk deinit failed" << std::endl;
    }
}

int CameraSrv::start()
{
    AS_LISTENER_CALLBACK_S listener_callback;
    listener_callback.onAttached = onAttached;
    listener_callback.onDetached = onAttached;
    listener_callback.privateData = this;

    AS_SDK_StartListener(listener_callback, AS_LISTENNER_TYPE_USB, true);
    AS_SDK_StartListener(listener_callback, AS_LISTENNER_TYPE_NET, true);
    return 0;
}

void CameraSrv::stop()
{
    int ret = 0;
    int dev_idx = 0;

    ret = AS_SDK_StopListener();
    if (ret == 0) {
        LOG(INFO) << "stop listener monitor" << std::endl;
    }

    LOG(INFO) << "stop and close the camera" << std::endl;
    for (auto it = m_devsList.begin(); it != m_devsList.end(); ) {
        AS_CAM_PTR dev = (AS_CAM_PTR)(*it);
        LOG(INFO) << "close camera idx " << dev_idx << std::endl;

        ret = AS_SDK_StopStream(dev);
        if (ret < 0) {
            LOG(ERROR) << "stop stream, ret: " << ret << std::endl;
        }
        ret = AS_SDK_CloseCamera(dev);
        if (ret < 0) {
            LOG(ERROR) << "close camera, ret: " << ret << std::endl;
        }
        ret = AS_SDK_DestoryCamHandle(dev);
        if (ret == 0) {
            LOG(INFO) << "destory camera success" << std::endl;
        }
        m_devsList.erase(it++);
        dev_idx++;
    }
}

void CameraSrv::onAttached(AS_CAM_ATTR_S *attr, void *privateData)
{
    int ret = 0;
    CameraSrv *server = static_cast<CameraSrv *>(privateData);
    CamSvrStreamParam_s stream_param;
    stream_param.open = true;
    stream_param.start = true;
    stream_param.image_flag = 0;

    LOG(INFO) << "attached" << std::endl;
    std::lock_guard<std::timed_mutex> lock(server->m_mutex);

    bool exist = false;
    for (const auto &camera : server->m_devsList) {
        AS_CAM_ATTR_S attr_t;
        memset(&attr_t, 0, sizeof(AS_CAM_ATTR_S));
        ret = AS_SDK_GetCameraAttrs(camera, attr_t);
        if ((ret == 0) && ((attr->type == AS_CAMERA_ATTR_LNX_USB))) {
            if ((attr->attr.usbAttrs.bnum == attr_t.attr.usbAttrs.bnum)
                && (strcmp(attr->attr.usbAttrs.port_numbers, attr_t.attr.usbAttrs.port_numbers) == 0)) {
                LOG(WARN) << "this camera exist" << std::endl;
                exist = true;
                break;
            }
        } else if ((ret == 0) && (attr->type == AS_CAMERA_ATTR_NET)) {
            if (strcmp(attr->attr.netAttrs.ip_addr, attr_t.attr.netAttrs.ip_addr) == 0) {
                LOG(WARN) << "this camera exist" << std::endl;
                exist = true;
                break;
            }
        } else if ((ret == 0) && (attr->type == AS_CAMERA_ATTR_WIN_USB)) {
            if (strcmp(attr->attr.winAttrs.symbol_link, attr_t.attr.winAttrs.symbol_link) == 0) {
                LOG(WARN) << "this camera exist" << std::endl;
                exist = true;
                break;
            }
        } else {
            LOG(ERROR) << "error camera attr" << std::endl;
            return;
        }
    }
    if (exist) {
        LOG(WARN) << "this device exist, ignore this event, attached end" << std::endl;
        return;
    }

    LOG(INFO) << "this is a new attach device, create and open it" << std::endl;
    AS_CAM_PTR newdev;
    ret = AS_SDK_CreateCamHandle(newdev, attr);
    if (ret == 0) {
        server->m_devsList.push_back(newdev);

        // get model type
        AS_SDK_CAM_MODEL_E cam_type;
        ret = AS_SDK_GetCameraModel(newdev, cam_type);
        LOG(INFO) << "get model type " << cam_type << std::endl;

        std::string file_path;
        if (server->getConfigFile(newdev, file_path, cam_type) != 0) {
            LOG(ERROR) << "cannot find config file" << std::endl;
            return;
        }

        server->m_camera_status->onCameraAttached(newdev, stream_param, cam_type);
        if (stream_param.open) {
            AS_CAM_Stream_Cb_s streamCallback;
            streamCallback.callback = server->onNewFrame;
            streamCallback.privateData = privateData;

            ret = AS_SDK_OpenCamera(newdev, file_path.c_str());
            if (ret < 0) {
                LOG(ERROR) << "open camera, ret: " << ret << std::endl;
                return;
            }
            server->m_camera_status->onCameraOpen(newdev);
            ret = AS_SDK_RegisterStreamCallback(newdev, &streamCallback);
            if (ret != 0) {
                LOG(ERROR) << "register stream callback failed" << std::endl;
            }

            if (cam_type == AS_SDK_CAM_MODEL_KUNLUN_A) {
                // Register merge stream callback
                AS_CAM_Merge_Cb_s mergeStreamCallback;
                mergeStreamCallback.callback = onNewMergeFrame;
                mergeStreamCallback.privateData = privateData;

                ret = AS_SDK_RegisterMergeFrameCallback(newdev, &mergeStreamCallback);
                if (ret != 0) {
                    LOG(ERROR) << "Register merge stream callback failed" << std::endl;
                    return;
                }
            }

            if (stream_param.start) {
                ret = AS_SDK_StartStream(newdev, stream_param.image_flag);
                if (ret < 0) {
                    LOG(ERROR) << "start stream, ret: " << ret << std::endl;
                    return;
                }
                server->m_camera_status->onCameraStart(newdev);
            }
        }
    }
    LOG(INFO) << "attached end" << std::endl;
}

void CameraSrv::onDetached(AS_CAM_ATTR_S *attr, void *privateData)
{
    int ret = 0;
    LOG(INFO) << "detached" << std::endl;
    CameraSrv *server = static_cast<CameraSrv *>(privateData);
    std::lock_guard<std::timed_mutex> lock(server->m_mutex);
    for (auto it = server->m_devsList.begin(); it != server->m_devsList.end(); it++) {
        AS_CAM_PTR dev = (AS_CAM_PTR)(*it);
        AS_CAM_ATTR_S t_attr;
        ret = AS_SDK_GetCameraAttrs(dev, t_attr);
        if (ret < 0) {
            LOG(ERROR) << "get camera attribute failed" << std::endl;
            return;
        }

        if (t_attr.type == AS_CAMERA_ATTR_LNX_USB) {
            if ((t_attr.attr.usbAttrs.bnum == attr->attr.usbAttrs.bnum)
                && (t_attr.attr.usbAttrs.dnum == attr->attr.usbAttrs.dnum)) {
                LOG(INFO) << "close and delete it from the list" << std::endl;
                server->m_camera_status->onCameraStop(dev);
                ret = AS_SDK_StopStream(dev, 0);
                if (ret != 0) {
                    LOG(INFO) << "stop stream failed" << std::endl;
                } else {
                    LOG(INFO) << "stop stream success" << std::endl;
                }
                server->m_camera_status->onCameraClose(dev);
                ret = AS_SDK_CloseCamera(dev);
                if (ret != 0) {
                    LOG(INFO) << "close camera failed" << std::endl;
                } else {
                    LOG(INFO) << "close camera success" << std::endl;
                }
                server->m_camera_status->onCameraDetached(dev);
                ret = AS_SDK_DestoryCamHandle(dev);
                if (ret == 0) {
                    LOG(INFO) << "destory camera success" << std::endl;
                }
                server->m_devsList.erase(it);
                break;
            }
        } else if (t_attr.type == AS_CAMERA_ATTR_NET) {
            if (strcmp(t_attr.attr.netAttrs.ip_addr, attr->attr.netAttrs.ip_addr) == 0) {
                LOG(INFO) << "close and delete it from the list" << std::endl;
                server->m_camera_status->onCameraStop(dev);
                ret = AS_SDK_StopStream(dev, 0);
                if (ret != 0) {
                    LOG(INFO) << "stop stream failed" << std::endl;
                } else {
                    LOG(INFO) << "stop stream success" << std::endl;
                }
                server->m_camera_status->onCameraClose(dev);
                ret = AS_SDK_CloseCamera(dev);
                if (ret != 0) {
                    LOG(INFO) << "close camera failed" << std::endl;
                } else {
                    LOG(INFO) << "close camera success" << std::endl;
                }
                server->m_camera_status->onCameraDetached(dev);
                ret = AS_SDK_DestoryCamHandle(dev);
                if (ret == 0) {
                    LOG(INFO) << "destory camera success" << std::endl;
                }
                server->m_devsList.erase(it);
                break;
            }
        } else {
            LOG(ERROR) << "camera attr type error" << std::endl;
            return;
        }

    }
    LOG(INFO) << "detached end" << std::endl;
}

void CameraSrv::onNewFrame(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData, void *privateData)
{
    CameraSrv *server = static_cast<CameraSrv *>(privateData);
    server->m_camera_status->onCameraNewFrame(pCamera, pstData);
}

void CameraSrv::onNewMergeFrame(AS_CAM_PTR pCamera, const AS_SDK_MERGE_s *pstData, void *privateData)
{
    CameraSrv *server = static_cast<CameraSrv *>(privateData);
    server->m_camera_status->onCameraNewMergeFrame(pCamera, pstData);
}

int CameraSrv::getConfigFile(AS_CAM_PTR pCamera, std::string &configfile, AS_SDK_CAM_MODEL_E cam_type)
{
    int ret = 0;
    std::string name_key;
    switch (cam_type) {
    case AS_SDK_CAM_MODEL_KONDYOR:
    case AS_SDK_CAM_MODEL_KONDYOR_NET:
        name_key = "kondyor_";
        break;
    case AS_SDK_CAM_MODEL_NUWA_XB40:
    case AS_SDK_CAM_MODEL_NUWA_X100:
    case AS_SDK_CAM_MODEL_NUWA_HP60:
    case AS_SDK_CAM_MODEL_NUWA_HP60V:
        name_key = "nuwa_";
        break;
    case AS_SDK_CAM_MODEL_KUNLUN_A:
    case AS_SDK_CAM_MODEL_KUNLUN_C:
        name_key = "kunlun_";
        break;
    case AS_SDK_CAM_MODEL_HP60C:
        name_key = "hp60c_";
        break;
    case AS_SDK_CAM_MODEL_HP60CN:
        name_key = "hp60cn_";
        break;
    case AS_SDK_CAM_MODEL_VEGA:
        name_key = "vega_";
        break;
    case AS_SDK_CAM_MODEL_CHANGJIANG_B:
        name_key = "changjiangB_";
        break;
    case AS_SDK_CAM_MODEL_TANGGULA:
        name_key = "tanggula_";
        break;
    case AS_SDK_CAM_MODEL_TANGGULA_A:
        name_key = "tanggulaA_";
        break;
    case AS_SDK_CAM_MODEL_TAISHAN:
        name_key = "taishan_";
        break;
    case AS_SDK_CAM_MODEL_TANGGULA_B:
        name_key = "tanggulaB_";
        break;
    default:
        LOG(ERROR) << "cam type error" << std::endl;
        return -1;
        break;
    }

    // get json
    std::vector<std::string> files;
    scanDir(m_configPath, files);
    ret = -1;
    for (auto it = files.begin(); it != files.end(); it++) {
        std::string filename = (*it).substr((*it).find_last_of("/"));
        if (filename.find(name_key) < filename.size()) {
            LOG(INFO) << "get file: " << *it << std::endl;
            configfile = (*it);
            ret = 0;
            break;
        }
    }

    if (ret != 0) {
        LOG(ERROR) << "cannot find config file" << std::endl;
        return -1;
    }

    return 0;
}

int CameraSrv::scanDir(const std::string &dir, std::vector<std::string> &file)
{
    int ret = 0;
    DIR *directory;
    struct dirent *ent;
    if (!(directory = opendir(dir.c_str()))) {
        std::cout << "can't not open dir:" << dir << std::endl;
        return -1;
    }
    while ((ent = readdir(directory)) != nullptr) {
        if (strncmp(ent->d_name, ".", 1) == 0) {
            continue;
        }
        if (ent->d_type == DT_REG) {
            std::string filepath(dir + "/" + ent->d_name);
            file.push_back(filepath);
        }
        if (ent->d_type == DT_DIR) {
            std::string childpath;
            childpath.append(dir);
            childpath.append("/");
            childpath.append(ent->d_name);
            scanDir(childpath, file);
        }
    }
    delete ent;
    closedir(directory);
    return ret;
}

