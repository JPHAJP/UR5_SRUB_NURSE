/**
 * @file      as_camera_sdk_def.h
 * @brief     angstrong camera sdk data structure
 *
 * Copyright (c) 2023 Angstrong Tech.Co.,Ltd
 *
 * @author    Angstrong SDK develop Team
 * @date      2023/03/24
 * @version   1.1

 */
#pragma once
#include <vector>
#include <string.h>
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#endif

#ifdef _WIN32
#define WinBuffSize 2048
#else
#define WinBuffSize 64
#endif

#define DEFAULT_IMG_FLG     0x00000000
#define DEPTH_IMG_FLG       0x00000001
#define RGB_IMG_FLG         0x00000002
#define IR_IMG_FLG          0x00000004
#define POINTCLOUD_IMG_FLG  0x00000008
#define YUYV_IMG_FLG        0x00000010
#define PEAK_IMG_FLG        0x00000020
#define SUB_DEPTH_IMG_FLG   0x00000040
#define MJPEG_IMG_FLG       0x00000080

typedef void *AS_CAM_PTR;

typedef enum FRAME_TYPE_E {
    AS_FRAME_TYPE_DEPTH = 0,
    AS_FRAME_TYPE_RGB,
    AS_FRAME_TYPE_IR,
    AS_FRMAE_TYPE_POINTCLOUD,
    AS_FRAME_TYPE_YUYV,
    AS_FRAME_TYPE_PEAK,
    AS_FRAME_TYPE_MJPEG,
    AS_FRAME_TYPE_BUTT
} AS_FRAME_Type_e;

typedef struct CAM_FRAME {
    AS_FRAME_Type_e type;
    unsigned int width;
    unsigned int height;
    void *data;
    unsigned int size;
    unsigned int bufferSize;
    unsigned int frameId;
    unsigned long long ts;
} AS_Frame_s;

typedef struct AS_POINTCLOUD {
    double angle;
    double distance;
    double quality;
} AS_POINTCLOUD_s;

typedef struct CAM_DATA {
    AS_Frame_s depthImg;
    AS_Frame_s rgbImg;
    AS_Frame_s irImg;
    AS_Frame_s pointCloud;
    AS_Frame_s yuyvImg;
    AS_Frame_s peakImg;
    AS_Frame_s mjpegImg;
    std::vector<AS_POINTCLOUD_s> pointCloud2;
} AS_SDK_Data_s;

typedef struct CAM_MERGE {
    AS_Frame_s depthImg;
    AS_Frame_s pointCloud;
} AS_SDK_MERGE_s;

typedef void(*AS_CAM_StreamCallback)(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData,
                                     void *privateData);

typedef struct STREAM_CALLBACK_S {
    AS_CAM_StreamCallback callback;
    void *privateData;
} AS_CAM_Stream_Cb_s;

typedef void(*AS_CAM_MergeFrameCallback)(AS_CAM_PTR pCamera, const AS_SDK_MERGE_s *pstData,
        void *privateData);

typedef struct MERGE_CALLBACK_S {
    AS_CAM_MergeFrameCallback callback;
    void *privateData;
} AS_CAM_Merge_Cb_s;

typedef struct CAM_Parameter {
    float fxir;
    float fyir;
    float cxir;
    float cyir;
    float fxrgb;
    float fyrgb;
    float cxrgb;
    float cyrgb;

    float R00;
    float R01;
    float R02;
    float R10;
    float R11;
    float R12;
    float R20;
    float R21;
    float R22;
    float T1;
    float T2;
    float T3;

    float K1ir;
    float K2ir;
    float K3ir;
    float P1ir;
    float P2ir;
    float K1rgb;
    float K2rgb;
    float K3rgb;
    float P1rgb;
    float P2rgb;
} AS_CAM_Parameter_s;

typedef struct STREAM_PARAM_S {
    int width;
    int height;
    int fps;
} AS_STREAM_Param_s;

typedef void(*AS_CAM_UpGradeCallback)(AS_CAM_PTR pCamera, void *privateData, float fProcess);
typedef struct CAM_UPGRADE_DEV_S {
    AS_CAM_UpGradeCallback callback;
    void *privateData;
} AS_CAM_Upgrade_Dev_s;

typedef enum TOFMode {
    odd_mode = 0,
    even_mode = 1,
    merge_mode = 2,
    mode_butt
} AS_TOFMODE_E;

typedef struct NET_DEV_ATTRS {
    char ip_addr[64];
    char mac_addr[64];
    int is_dhcp;
} AS_NET_DEV_Attrs_s;

typedef enum CAMERA_MODEL_E {
    AS_SDK_CAM_MODEL_UNKNOWN,
    AS_SDK_CAM_MODEL_NUWA_XB40,
    AS_SDK_CAM_MODEL_NUWA_X100,
    AS_SDK_CAM_MODEL_NUWA_HP60,
    AS_SDK_CAM_MODEL_NUWA_HP60V,
    AS_SDK_CAM_MODEL_KUNLUN_A,
    AS_SDK_CAM_MODEL_KUNLUN_C,
    AS_SDK_CAM_MODEL_KONDYOR,
    AS_SDK_CAM_MODEL_KONDYOR_NET,
    AS_SDK_CAM_MODEL_HP60C,
    AS_SDK_CAM_MODEL_HP60CN,
    AS_SDK_CAM_MODEL_VEGA,
    AS_SDK_CAM_MODEL_CHANGA,
    AS_SDK_CAM_MODEL_CHANGJIANG_B,
    AS_SDK_CAM_MODEL_TANGGULA,
    AS_SDK_CAM_MODEL_TANGGULA_A,
    AS_SDK_CAM_MODEL_TAISHAN,
    AS_SDK_CAM_MODEL_O2,
    AS_SDK_CAM_MODEL_TANGGULA_B,
    AS_SDK_CAM_MODEL_BUTT
} AS_SDK_CAM_MODEL_E;

typedef enum MEDIA_TYPE_E {
    AS_MEDIA_TYPE_DEPTH,
    AS_MEDIA_TYPE_RGB,
    AS_MEDIA_TYPE_IR,
    AS_MEDIA_TYPE_PEAK,
    AS_MEDIA_TYPE_BUTT
} AS_MEDIA_TYPE_E;

typedef enum AS_LISTENER_TYPE {
    AS_LISTENNER_TYPE_USB = 0,
    AS_LISTENNER_TYPE_NET,
    AS_LISTENNER_TYPE_BUTT
} AS_LISTENER_TYPE_E;

typedef enum AS_CAM_ATTR_TYPE {
    AS_CAMERA_ATTR_LNX_USB,
    AS_CAMERA_ATTR_WIN_USB,
    AS_CAMERA_ATTR_NET,
    AS_CAMERA_ATTR_BUTT
} AS_CAM_ATTR_TYPE_E;

typedef struct AS_USB_DEV_ATTR {
    int vid;
    int pid;
    char serial[64];
    int bnum;
    int dnum;
    int port_number;
    char port_numbers[64];
} AS_USB_DEV_ATTR_S;

typedef struct AS_NETWORK_DEV_ATTR {
    char ip_addr[64];
    int port;
    AS_SDK_CAM_MODEL_E model;
} AS_NETWORK_DEV_ATTR_S;

typedef struct AS_USB_WIN_DEV_ATTR {
    int vid;
    int pid;
    int deviceID;
    char symbol_link[WinBuffSize];
    char location_path[WinBuffSize];
    char serialPort[WinBuffSize];
} AS_USB_WIN_DEV_ATTR_S ;

typedef struct AS_CAM_ATTR {
    AS_CAM_ATTR_TYPE_E type;
    union {
        AS_USB_DEV_ATTR_S usbAttrs;
        AS_NETWORK_DEV_ATTR_S netAttrs;
        AS_USB_WIN_DEV_ATTR_S winAttrs;
    } attr;
} AS_CAM_ATTR_S;

typedef void (*AS_Listener_Callback)(AS_CAM_ATTR_S *attr, void *privateData);

typedef struct AS_LISTENER_CALLBACK {
    AS_Listener_Callback onAttached;
    AS_Listener_Callback onDetached;
    void *privateData;
} AS_LISTENER_CALLBACK_S;

typedef enum TIME_STAMP_TYPE {
    AS_TIME_STAMP_TYPE_STEADY_CLOCK,
    AS_TIME_STAMP_TYPE_SYSTEM_CLOCK,
    AS_TIME_STAMP_TYPE_BUTT
} AS_TIME_STAMP_TYPE_E;

typedef struct AS_CONFIG_INFO {
    bool is_Registration;
    bool is_pclOrganized;
    bool depth_decimal;
} AS_CONFIG_INFO_S;