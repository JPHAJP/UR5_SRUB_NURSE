/**
 * @file      as_camera_sdk_api.h
 * @brief     angstrong camera sdk api
 *
 * Copyright (c) 2023 Angstrong Tech.Co.,Ltd
 *
 * @author    Angstrong SDK develop Team
 * @date      2023/03/24
 * @version   1.1

 */
#pragma once

#include <list>
#include <vector>
#include <string>
#include "as_camera_sdk_def.h"

/**
 * @brief     SDK init
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2022/09/02
 */
int AS_SDK_Init();

/**
 * @brief     SDK Deinit
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2022/09/02
 */
int AS_SDK_Deinit();

/**
 * @brief     get the angstrong camera list which connected to the host
 * @param[out]devList : angstrong cmaera handle list
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2022/07/26
 */
int AS_SDK_GetCameraList(std::list<AS_CAM_PTR> &devList);

/**
 * @brief     free the angstrong camera list
 * @param[in]devList : angstrong cmaera handle list
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2022/07/26
 */
int AS_SDK_FreeCameraList(std::list<AS_CAM_PTR> &devList);

/**
 * @brief     open angstrong camera by handle
 * @param[in]pCamera : the camera handle which got in function AS_SDK_GetCameraList
 * @param[in]pParaFilePath : configuration file path
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2023/02/06
 */
int AS_SDK_OpenCamera(AS_CAM_PTR pCamera, const char *pParaFilePath);

/**
 * @brief     close angstrong camera by handle
 * @param[in]pCamera : the camera handle which got in function AS_SDK_GetCameraList
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2022/07/26ok
 */
int AS_SDK_CloseCamera(AS_CAM_PTR pCamera);

/**
 * @brief     register stream callback function, should open the camera before
 *            call this function
 * @param[in]pCamera : the camera handle which got in function AS_SDK_GetCameraList
 * @param[in]pstCallback : the callback function and private data
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2022/07/26
 */
int AS_SDK_RegisterStreamCallback(AS_CAM_PTR pCamera, AS_CAM_Stream_Cb_s *pstCallback);

/**
 * @brief     register merge frame callback function, should open the camera before
 *            call this function
 * @param[in]pCamera : the camera handle which got in function AS_SDK_GetCameraList
 * @param[in]pstCallback : the callback function and private data
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2022/11/30
 */
int AS_SDK_RegisterMergeFrameCallback(AS_CAM_PTR pCamera, AS_CAM_Merge_Cb_s *pstCallback);

/**
 * @brief     start streming by handle
 * @param[in]pCamera : the camera handle which got in function AS_SDK_GetCameraList
 * @param[in]type : the stream type
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2022/07/26
 */
int AS_SDK_StartStream(AS_CAM_PTR pCamera, int type = 0);

/**
 * @brief     stop streming by handle
 * @param[in]pCamera : the camera handle which got in function AS_SDK_GetCameraList
 * @param[in]type : the stream type
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2022/07/26
 */
int AS_SDK_StopStream(AS_CAM_PTR pCamera, int type = 0);

/**
 * @brief     get firmware version
 * @param[out]pszVersion : firmware version
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2022/08/29
 */
int AS_SDK_GetFwVersion(AS_CAM_PTR pCamera, char *pszVersion, int size);

/**
 * @brief     get the sdk version
 * @param[out]pszVersion : the sdk version
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2022/11/17
 */
int AS_SDK_GetSwVersion(char *pszVersion, unsigned int size);

/**
 * @brief     get stream type
 * @param[out]type : stream type
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2022/10/17
 */
int AS_SDK_GetStreamType(AS_CAM_PTR pCamera, int *type);

/**
 * @brief     get the camera serial number
 * @param[in]pCamera : the camera handle which got in function AS_SDK_GetCameraList
 * @param[out]sn : the camera serial number
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2022/07/26
 */
int AS_SDK_GetSerialNumber(AS_CAM_PTR pCamera, char *pszSerialNo, int size);

/**
 * @brief     upgrade the camera firmware
 * @param[in]pCamera : the camera handle which got in function AS_SDK_GetCameraList
 * @param[in]pszFilePath : the upgrade bin file path
 * @param[in]pstCallback : upgrade status callback info
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2022/07/26
 */
int AS_SDK_UpgradeDev(AS_CAM_PTR pCamera, const char *pszFilePath, AS_CAM_Upgrade_Dev_s *pstCallback);

/**
 * @brief     get the camera parameter
 * @param[in]pCamera : the camera handle which got in function AS_SDK_GetCameraList
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2022/07/26
 */
int AS_SDK_GetCamParameter(AS_CAM_PTR pCamera, AS_CAM_Parameter_s *pstParam);

/**
 * @brief     get the dtof camera mode
 * @param[in]pCamera : the camera handle which got in function AS_SDK_GetCameraList
 * @param[out]mode : the dtof camera mode
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2022/12/8
 */
int AS_SDK_GetTofMode(AS_CAM_PTR pCamera, AS_TOFMODE_E &mode);

/**
 * @brief     set the stream parameter
 * @param[in]pCamera : the camera handle which got in function AS_SDK_GetCameraList
 * @param[in]type : the stream type
 * @param[in]pstStreamParam : the steram paramter info which include width,height and frame rate
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2022/07/26
 */
int AS_SDK_SetStreamParam(AS_CAM_PTR pCamera, AS_MEDIA_TYPE_E type, const AS_STREAM_Param_s *pstStreamParam);

/**
 * @brief     get raw stream param
 * @param[out]pstStreamParam : width,height and fps of raw stream
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2022/10/17
 */
int AS_SDK_GetStreamParam(AS_CAM_PTR pCamera, AS_STREAM_Param_s *pstStreamParam);

/**
 * @brief     set dev net attrs. if need not set any attr, set ip_addr/mac_addr 0, or set is_dhcp -1
 * @param[in]pCamera : camera handle
 * @param[in]AS_NET_DEV_Attrs_s : dev net attrs
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2023/03/24
 */
int AS_SDK_SetDevNetAttrs(AS_CAM_PTR pCamera, AS_NET_DEV_Attrs_s attrs);

/**
 * @brief     get dev net attrs
 * @param[in]pCamera : camera handle
 * @param[in]AS_NET_DEV_Attrs_s : dev net attrs
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2023/03/24
 */
int AS_SDK_GetDevNetAttrs(AS_CAM_PTR pCamera, AS_NET_DEV_Attrs_s &attrs);

/**
 * @brief     get the camera model
 * @param[in]pCamera : the camera handle which got in function AS_SDK_GetCameraList
 * @param[out]model : the camera model
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2023/04/24
 */
int AS_SDK_GetCameraModel(AS_CAM_PTR pCamera, AS_SDK_CAM_MODEL_E &model);

/**
 * @brief     start camera listener
 * @param[in] callback : callback for listenner
 * @param[in] type : listener type
 * @param[in] enumerate : enumerate when start listenner
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2023/03/13
 */
int AS_SDK_StartListener(const AS_LISTENER_CALLBACK_S &callback, AS_LISTENER_TYPE_E type, bool enumerate = true);

/**
 * @brief     stop camera listenner
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2023/03/13
 */
int AS_SDK_StopListener(void);

/**
 * @brief     create camera handle by attribute
 * @param[out] pCamera : the new camera handle
 * @param[in] attr : camera attribute
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2023/03/13
 */
int AS_SDK_CreateCamHandle(AS_CAM_PTR &pCamera, AS_CAM_ATTR_S *attr);

/**
 * @brief     destrory camera handle
 * @param[in] pCamera : the camera handle
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2023/03/13
 */
int AS_SDK_DestoryCamHandle(AS_CAM_PTR pCamera);

/**
 * @brief     get camera attributes
 * @param[in] pCamera : the camera handle
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2023/03/13
 */
int AS_SDK_GetCameraAttrs(AS_CAM_PTR pCamera, AS_CAM_ATTR_S &attr);

/**
 * @brief     set time stamp type
 * @param[in]pCamera : the camera handle
 * @param[in]type : the time stamp type
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2022/09/19
 */
int AS_SDK_SetTimeStampType(AS_CAM_PTR pCamera, AS_TIME_STAMP_TYPE_E type);

/**
 * @brief     log the camera configuration
 * @param[in]pCamera : the camera handle
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2022/10/18
 */
int AS_SDK_LogCameraCfg(AS_CAM_PTR pCamera);

/**
 * @brief     get the image size capability set
 * @param[in]pCamera : the camera handle
 * @param[in]type : the stream type
 * @param[out]capability : the image size capability set
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2022/12/07
 */
int AS_SDK_GetCapability(AS_CAM_PTR pCamera, AS_MEDIA_TYPE_E type, std::vector<AS_STREAM_Param_s> &capability);

/**
 * @brief     get camera config info
 * @param[in]pCamera : the camera handle
 * @param[in]AS_CONFIG_INFO_S :  config info
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2023/08/02
 */
int AS_SDK_GetConfigInfo(AS_CAM_PTR pCamera, AS_CONFIG_INFO_S &config_info);

/**
 * @brief     reboot camera
 * @param[in]pCamera : the camera handle
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2023/08/18
 */
int AS_SDK_CAMERA_Reboot(AS_CAM_PTR pCamera);

/**
 * @brief     get private max size
 * @param[in]pCamera : the camera handle
 * @param[out]size : data size (byte)
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2024/05/14
 */
int AS_SDK_GetPrivateMaxSize(AS_CAM_PTR pCamera, int &size);

/**
 * @brief     write private data to the camera
 * @param[in]pCamera : the camera handle
 * @param[in]data : the data pointer
 * @param[in]size : data size (byte)
 * @param[in]offset : the offset of the data pointer
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2024/05/14
 */
int AS_SDK_WritePrivateData(AS_CAM_PTR pCamera, void *data, int size, int offset);

/**
 * @brief     read private data to the camera
 * @param[in]pCamera : the camera handle
 * @param[in]data : the data pointer
 * @param[in]size : data size (byte)
 * @param[in]offset : the offset of the data pointer
 * @return    0 success,non-zero error code.
 * @exception None
 * @author    Angstrong SDK develop Team
 * @date      2024/05/14
 */
int AS_SDK_ReadPrivateData(AS_CAM_PTR pCamera, void *data, int size, int offset);