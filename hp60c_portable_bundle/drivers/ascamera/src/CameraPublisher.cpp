/**
 * @file      CameraPublisher.cpp
 * @brief     angstrong camera publisher.
 *
 * Copyright (c) 2023 Angstrong Tech.Co.,Ltd
 *
 * @author    Angstrong SDK develop Team
 * @date      2023/03/27
 * @version   1.0

 */

#include "CameraPublisher.h"

#define LOG_PARA(value, str)\
    do { \
        if (value != -1) { \
            RCLCPP_INFO_STREAM(m_logger, "get " << str << " " << value); \
        } \
    } while(0)


CameraPublisher::CameraPublisher()
    : Node("camera_publisher")
    , m_logger(this->get_logger())
{
    initLaunchParams();
    m_nodeNameSpace = get_namespace();
}

CameraPublisher::~CameraPublisher()
{
    stop();
}

int CameraPublisher::start()
{
    int ret = 0;

    std::string config_path;
    this->declare_parameter<std::string>("confiPath", "");
    this->get_parameter<std::string>("confiPath", config_path);
    if (config_path.size() == 0) {
        RCLCPP_ERROR_STREAM(m_logger, "config file path error");
        return -1;
    }

    if (server == nullptr) {
        server = new CameraSrv(this, config_path);
        ret = server->start();
        if (ret != 0) {
            RCLCPP_ERROR_STREAM(m_logger, "start server failed");
        }
        m_monitor_flg = true;
        m_monitor_thd = std::thread(&CameraPublisher::streamController, this);
    }

    return 0;
}

void CameraPublisher::stop()
{
    if (server != nullptr) {
        server->stop();
        m_monitor_flg = false;
        if (m_monitor_thd.joinable()) {
            m_monitor_thd.join();
        }
        delete server;
        server = nullptr;
    }

    /* free the map */
    m_camera_map.erase(m_camera_map.begin(), m_camera_map.end());
}

void CameraPublisher::saveImage()
{
    for (auto it = m_camera_map.begin(); it != m_camera_map.end(); it++) {
        it->second->enableSaveImage(true);
    }
}

void CameraPublisher::logFps(bool enable)
{
    m_logfps = enable;
}

bool CameraPublisher::getLogFps()
{
    return m_logfps;
}

int CameraPublisher::onCameraAttached(AS_CAM_PTR pCamera, CamSvrStreamParam_s &param,
                                      const AS_SDK_CAM_MODEL_E &cam_type)
{
    int ret = 0;
    AS_CAM_ATTR_S attr_t;
    bool reconnected = false;
    unsigned int dev_idx = imgPubList.size();
    param.open = true;
    memset(&attr_t, 0, sizeof(AS_CAM_ATTR_S));
    ret = AS_SDK_GetCameraAttrs(pCamera, attr_t);
    if (ret != 0) {
        RCLCPP_ERROR_STREAM(m_logger, "get device path info failed");
    }

    /* open the specified camera */
    switch (attr_t.type) {
    case AS_CAMERA_ATTR_LNX_USB:
        if ((m_launch_param.usb_bus_num != -1) && (strcmp(m_launch_param.usb_port_nums, "null") != 0)) {
            if ((attr_t.attr.usbAttrs.bnum != m_launch_param.usb_bus_num) ||
                (std::string(attr_t.attr.usbAttrs.port_numbers) != m_launch_param.usb_port_nums)) {
                param.open = false;
                RCLCPP_WARN_STREAM(m_logger, "found a dev but cannot match the specified camera, launch bus num " <<
                                   m_launch_param.usb_bus_num <<
                                   ", port num " << m_launch_param.usb_port_nums << ", but found bus num " << attr_t.attr.usbAttrs.bnum << ", port num " <<
                                   attr_t.attr.usbAttrs.port_numbers);
                return -1;
            } else {
                RCLCPP_INFO_STREAM(m_logger, "open the specified camera : bus num:" << attr_t.attr.usbAttrs.bnum
                                   << "  port numbers:" << std::string(attr_t.attr.usbAttrs.port_numbers));
            }
        }
        break;
    case AS_CAMERA_ATTR_NET:
        break;
    default:
        break;
    }
    logCameraPathInfo(attr_t);

    /* create publisher */
    unsigned int imgPubIdx = 0;
    switch (attr_t.type) {
    case AS_CAMERA_ATTR_LNX_USB:
        for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
            if ((it->attr_s.attr.usbAttrs.bnum == attr_t.attr.usbAttrs.bnum)
                && (strcmp(it->attr_s.attr.usbAttrs.port_numbers, attr_t.attr.usbAttrs.port_numbers) == 0)) {
                RCLCPP_INFO_STREAM(m_logger, "reconnected, update the camera info");
                param.start = true;
                it->pCamera = pCamera;
                memcpy(&it->attr_s, &attr_t, sizeof(AS_CAM_ATTR_S));
                it->stream_flg = 0;
                reconnected = true;
                dev_idx = imgPubIdx;
                break;
            }
            imgPubIdx++;
        }
        break;
    case AS_CAMERA_ATTR_NET:
        for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
            if ((it->attr_s.attr.netAttrs.port == attr_t.attr.netAttrs.port)
                && (strcmp(it->attr_s.attr.netAttrs.ip_addr, attr_t.attr.netAttrs.ip_addr) == 0)) {
                RCLCPP_INFO_STREAM(m_logger, "reconnected, update the camera info");
                param.start = true;
                it->pCamera = pCamera;
                memcpy(&it->attr_s, &attr_t, sizeof(AS_CAM_ATTR_S));
                it->stream_flg = 0;
                reconnected = true;
                dev_idx = imgPubIdx;
                break;
            }
            imgPubIdx++;
        }
        break;
    default:
        break;
    }

    /* create publisher */
    if (!reconnected) {
        RCLCPP_INFO_STREAM(m_logger, "create a new publisher info set");
        PUBLISHER_INFO_S stPublisherInfo;
        stPublisherInfo.index = dev_idx;
        switch (cam_type) {
        case AS_SDK_CAM_MODEL_NUWA_XB40:
        case AS_SDK_CAM_MODEL_NUWA_X100:
        case AS_SDK_CAM_MODEL_NUWA_HP60:
        case AS_SDK_CAM_MODEL_NUWA_HP60V:
            stPublisherInfo.depth_camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("~/depth" + std::to_string(
                    dev_idx) + "/camera_info", 5);
            stPublisherInfo.depth_raw_pub = this->create_publisher<sensor_msgs::msg::Image>("~/depth" + std::to_string(
                                                dev_idx) + "/image_raw", 100);
            stPublisherInfo.ir_camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("~/ir" + std::to_string(
                    dev_idx) + "/camera_info", 5);
            stPublisherInfo.ir_img_pub = this->create_publisher<sensor_msgs::msg::Image>("~/ir" + std::to_string(
                                             dev_idx) + "/image", 100);
            stPublisherInfo.depth_points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/depth" + std::to_string(
                                                   dev_idx) + "/points", 100);
            break;
        case AS_SDK_CAM_MODEL_CHANGJIANG_B:
            stPublisherInfo.depth_camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("~/depth" + std::to_string(
                    dev_idx) + "/camera_info", 5);
            stPublisherInfo.depth_raw_pub = this->create_publisher<sensor_msgs::msg::Image>("~/depth" + std::to_string(
                                                dev_idx) + "/connectDepth",
                                            100);
            stPublisherInfo.depth_points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/depth" + std::to_string(
                                                   dev_idx) + "/points", 100);
            break;
        case AS_SDK_CAM_MODEL_TANGGULA_A:
            stPublisherInfo.depth_camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("~/depth" + std::to_string(
                    dev_idx) + "/camera_info", 5);
            stPublisherInfo.depth_raw_pub = this->create_publisher<sensor_msgs::msg::Image>("~/depth" + std::to_string(
                                                dev_idx) + "/image_raw", 100);
            stPublisherInfo.depth_points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/depth" + std::to_string(
                                                   dev_idx) + "/points", 100);
            break;
        case AS_SDK_CAM_MODEL_HP60C:
        case AS_SDK_CAM_MODEL_HP60CN:
        case AS_SDK_CAM_MODEL_TANGGULA:
        case AS_SDK_CAM_MODEL_TAISHAN:
        case AS_SDK_CAM_MODEL_TANGGULA_B:
            stPublisherInfo.depth_camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("~/depth" + std::to_string(
                    dev_idx) + "/camera_info", 5);
            stPublisherInfo.depth_raw_pub = this->create_publisher<sensor_msgs::msg::Image>("~/depth" + std::to_string(
                                                dev_idx) + "/image_raw", 100);
            stPublisherInfo.rgb_camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("~/rgb" + std::to_string(
                    dev_idx) + "/camera_info", 5);
            stPublisherInfo.rgb_img_raw_pub = this->create_publisher<sensor_msgs::msg::Image>("~/rgb" + std::to_string(
                                                  dev_idx) + "/image", 100);
            stPublisherInfo.depth_points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/depth" + std::to_string(
                                                   dev_idx) + "/points", 100);
            break;
        case AS_SDK_CAM_MODEL_VEGA:
            stPublisherInfo.depth_camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("~/depth" + std::to_string(
                    dev_idx) + "/camera_info", 5);
            stPublisherInfo.depth_raw_pub = this->create_publisher<sensor_msgs::msg::Image>("~/depth" + std::to_string(
                                                dev_idx) + "/image_raw", 100);
            stPublisherInfo.rgb_camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("~/rgb" + std::to_string(
                    dev_idx) + "/camera_info", 5);
            stPublisherInfo.rgb_img_raw_pub = this->create_publisher<sensor_msgs::msg::Image>("~/rgb" + std::to_string(
                                                  dev_idx) + "/image", 100);
            stPublisherInfo.ir_camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("~/ir" + std::to_string(
                    dev_idx) + "/camera_info", 5);
            stPublisherInfo.ir_img_pub = this->create_publisher<sensor_msgs::msg::Image>("~/ir" + std::to_string(
                                             dev_idx) + "/image", 100);
            stPublisherInfo.depth_points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/depth" + std::to_string(
                                                   dev_idx) + "/points", 100);
            break;
        case AS_SDK_CAM_MODEL_KUNLUN_A:
            stPublisherInfo.depth_camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("~/depth" + std::to_string(
                    dev_idx) + "/camera_info", 5);
            stPublisherInfo.depth_odd_raw_pub = this->create_publisher<sensor_msgs::msg::Image>("~/depth" + std::to_string(
                                                    dev_idx) + "/odd_image_raw", 100);
            stPublisherInfo.depth_even_raw_pub = this->create_publisher<sensor_msgs::msg::Image>("~/depth" + std::to_string(
                    dev_idx) + "/even_image_raw", 100);
            stPublisherInfo.depth_merge_raw_pub = this->create_publisher<sensor_msgs::msg::Image>("~/depth" + std::to_string(
                    dev_idx) + "/merge_image_raw", 100);
            stPublisherInfo.peak_camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("~/peak" + std::to_string(
                    dev_idx) + "/camera_info", 5);
            stPublisherInfo.peak_odd_img_pub = this->create_publisher<sensor_msgs::msg::Image>("~/peak" + std::to_string(
                                                   dev_idx) + "/odd_image", 100);
            stPublisherInfo.peak_even_img_pub = this->create_publisher<sensor_msgs::msg::Image>("~/peak" + std::to_string(
                                                    dev_idx) + "/even_image", 100);
            stPublisherInfo.depth_points_odd_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/depth" + std::to_string(
                    dev_idx) + "/odd_points", 100);
            stPublisherInfo.depth_points_even_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/depth" +
                                                    std::to_string(dev_idx) + "/even_points", 100);
            stPublisherInfo.depth_points_merge_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/depth" +
                    std::to_string(dev_idx) + "/merge_points", 100);
            break;
        case AS_SDK_CAM_MODEL_KUNLUN_C:
            stPublisherInfo.depth_camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("~/depth" + std::to_string(
                    dev_idx) + "/camera_info", 5);
            stPublisherInfo.depth_raw_pub = this->create_publisher<sensor_msgs::msg::Image>("~/depth" + std::to_string(
                                                dev_idx) + "/image_raw", 100);
            stPublisherInfo.rgb_camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("~/rgb" + std::to_string(
                    dev_idx) + "/camera_info", 5);
            stPublisherInfo.rgb_img_raw_pub = this->create_publisher<sensor_msgs::msg::Image>("~/rgb" + std::to_string(
                                                  dev_idx) + "/image", 100);
            stPublisherInfo.peak_camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("~/peak" + std::to_string(
                    dev_idx) + "/camera_info", 5);
            stPublisherInfo.peak_img_pub = this->create_publisher<sensor_msgs::msg::Image>("~/peak" + std::to_string(
                                               dev_idx) + "/image", 100);
            stPublisherInfo.depth_points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/depth" + std::to_string(
                                                   dev_idx) + "/points", 100);
            break;
        case AS_SDK_CAM_MODEL_KONDYOR:
            stPublisherInfo.depth_points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/depth" + std::to_string(
                                                   dev_idx) + "/points", 100);
            break;
        case AS_SDK_CAM_MODEL_KONDYOR_NET:
            stPublisherInfo.depth_points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/depth" + std::to_string(
                                                   dev_idx) + "/points", 100);
            break;
        default:
            break;
        }

        stPublisherInfo.pCamera = pCamera;
        memcpy(&stPublisherInfo.attr_s, &attr_t, sizeof(AS_CAM_ATTR_S));
        if (param.image_flag == 0) {
            stPublisherInfo.stream_flg = 0x0fffffff;
        } else {
            stPublisherInfo.stream_flg = param.image_flag;
        }
        stPublisherInfo.camStatus = CAMERA_CLOSED_STATUS;
        imgPubList.push_back(stPublisherInfo);
    }

    m_camera_map.insert(std::make_pair(pCamera, std::make_shared<Camera>(pCamera, cam_type, m_nodeNameSpace, dev_idx,
                                       this->get_logger())));
    m_cam_type_map.insert(std::make_pair(pCamera, cam_type));

    /* nuwa camera whether match usb device.
       If it is a virtual machine, do not match it. */
    if ((cam_type == AS_SDK_CAM_MODEL_NUWA_XB40) ||
        (cam_type == AS_SDK_CAM_MODEL_NUWA_X100) ||
        (cam_type == AS_SDK_CAM_MODEL_NUWA_HP60) ||
        (cam_type == AS_SDK_CAM_MODEL_NUWA_HP60V)) {
        extern int AS_Nuwa_SetUsbDevMatch(bool is_match);
        AS_Nuwa_SetUsbDevMatch(!virtualMachine());
        // AS_Nuwa_SetUsbDevMatch(false);
    }

    return 0;
}

int CameraPublisher::onCameraDetached(AS_CAM_PTR pCamera)
{
    int ret = 0;

    RCLCPP_INFO_STREAM(m_logger, "camera detached");
    auto camIt = m_camera_map.find(pCamera);
    if (camIt != m_camera_map.end()) {
        m_camera_map.erase(pCamera);
    }

    if (m_cam_type_map.find(pCamera) != m_cam_type_map.end()) {
        m_cam_type_map.erase(pCamera);
    }
    return ret;
}

void CameraPublisher::onCameraNewFrame(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData)
{
    int ret = -1;
    std::string serialno = "";

    AS_CAM_Parameter_s stIrRgbParameter;

    auto camIt = m_camera_map.find(pCamera);
    if (camIt != m_camera_map.end()) {
        if (m_logfps) {
            camIt->second->checkFps();
        }
        camIt->second->getSerialNo(serialno);
        camIt->second->saveImage(pstData);
        ret = camIt->second->getCamParameter(stIrRgbParameter);
    }

    builtin_interfaces::msg::Time time = this->get_clock()->now();

    if (ret == 0) {
        depthInfoPublisher(pCamera, pstData, stIrRgbParameter, time);
        pointcloudPublisher(pCamera, pstData, stIrRgbParameter);
        rgbInfoPublisher(pCamera, pstData, stIrRgbParameter, time);
        irInfoPublisher(pCamera, pstData, stIrRgbParameter, time);
        peakInfoPublisher(pCamera, pstData, stIrRgbParameter, time);
        tfPublisher(pCamera, stIrRgbParameter);
    }
}

void CameraPublisher::onCameraNewMergeFrame(AS_CAM_PTR pCamera, const AS_SDK_MERGE_s *pstData)
{
    int ret = -1;
    std::string serialno = "";

    AS_CAM_Parameter_s stIrRgbParameter;

    RCLCPP_INFO_STREAM_ONCE(m_logger, "publish merge frame info");

    auto camIt = m_camera_map.find(pCamera);
    if (camIt != m_camera_map.end()) {
        if (m_logfps) {
            camIt->second->checkFps();
        }
        camIt->second->getSerialNo(serialno);
        camIt->second->saveMergeImage(pstData);
        ret = camIt->second->getCamParameter(stIrRgbParameter);
    }

    builtin_interfaces::msg::Time time = this->get_clock()->now();

    if (ret == 0) {
        depthMergeInfoPublisher(pCamera, pstData, stIrRgbParameter, time);
        pointcloudMergePublisher(pCamera, pstData, stIrRgbParameter);
    }
}

int CameraPublisher::onCameraOpen(AS_CAM_PTR pCamera)
{
    int ret = 0;
    RCLCPP_INFO_STREAM(m_logger, "camera opened");

    for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
        if (it->pCamera == pCamera) {
            memset(&it->config_info, 0, sizeof(AS_CONFIG_INFO_S));
            AS_SDK_CAM_MODEL_E cam_type = AS_SDK_CAM_MODEL_UNKNOWN;
            ret = AS_SDK_GetCameraModel(pCamera, cam_type);
            if (ret < 0) {
                RCLCPP_ERROR_STREAM(m_logger, "get camera model fail");
                return ret;
            }
            if ((cam_type == AS_SDK_CAM_MODEL_HP60C) || (cam_type == AS_SDK_CAM_MODEL_HP60CN)
                || (cam_type == AS_SDK_CAM_MODEL_VEGA) || (cam_type == AS_SDK_CAM_MODEL_TAISHAN)
                || (cam_type == AS_SDK_CAM_MODEL_TANGGULA_B)) {
                ret = AS_SDK_GetConfigInfo(pCamera, it->config_info);
                RCLCPP_INFO_STREAM(m_logger, "get config info, ret " << ret << ", is_Registration " << it->config_info.is_Registration);
            }
            it->camStatus = CAMERA_OPENED_STATUS;
        }
    }

    auto camIt = m_camera_map.find(pCamera);
    if (camIt != m_camera_map.end()) {
        camIt->second->init();
    }

    ret = setResolution(pCamera, m_launch_param);
    if (ret < 0) {
        RCLCPP_ERROR_STREAM(m_logger, "set resolution fail.");
        return ret;
    }

    return ret;
}

int CameraPublisher::onCameraClose(AS_CAM_PTR pCamera)
{
    int ret = 0;

    for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
        if (it->pCamera == pCamera) {
            it->camStatus = CAMERA_CLOSED_STATUS;
            break;
        }
    }

    return ret;
}

int CameraPublisher::onCameraStart(AS_CAM_PTR pCamera)
{
    int ret = 0;

    for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
        if (it->pCamera == pCamera) {
            it->camStatus = CAMERA_STREAM_STATUS;
            break;
        }
    }

    return ret;
}

int CameraPublisher::onCameraStop(AS_CAM_PTR pCamera)
{
    int ret = 0;

    for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
        if (it->pCamera == pCamera) {
            it->camStatus = CAMERA_OPENED_STATUS;
            break;
        }
    }

    return ret;
}

int CameraPublisher::genRosDepthCamInfo(const AS_Frame_s *pstFrame,
                                        sensor_msgs::msg::CameraInfo *pstCamInfo,
                                        AS_CAM_Parameter_s &stParameter, bool registration)
{
    std::array<double, 9> R = {1, 0, 0,
                               0, 1, 0,
                               0, 0, 1
                              };

    pstCamInfo->width = pstFrame->width;
    pstCamInfo->height = pstFrame->height;

    pstCamInfo->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    pstCamInfo->d.resize(5);
    pstCamInfo->d[0] = 0;
    pstCamInfo->d[1] = 0;
    pstCamInfo->d[2] = 0;
    pstCamInfo->d[3] = 0;
    pstCamInfo->d[4] = 0;

    pstCamInfo->k.fill(0.0);
    if (registration) {
        pstCamInfo->k[0] = stParameter.fxrgb;
        pstCamInfo->k[2] = stParameter.cxrgb;
        pstCamInfo->k[4] = stParameter.fyrgb;
        pstCamInfo->k[5] = stParameter.cyrgb;
    } else {
        pstCamInfo->k[0] = stParameter.fxir;
        pstCamInfo->k[2] = stParameter.cxir;
        pstCamInfo->k[4] = stParameter.fyir;
        pstCamInfo->k[5] = stParameter.cyir;
    }
    pstCamInfo->k[8] = 1.0;

    pstCamInfo->r = R;

    pstCamInfo->p.fill(0.0);
    pstCamInfo->p[0] = pstCamInfo->k[0];
    pstCamInfo->p[2] = pstCamInfo->k[2];
    pstCamInfo->p[3] = 0.0;
    pstCamInfo->p[5] = pstCamInfo->k[4];
    pstCamInfo->p[6] = pstCamInfo->k[5];
    pstCamInfo->p[7] = 0;
    pstCamInfo->p[10] = 1.0;
    pstCamInfo->p[11] = 0;

    pstCamInfo->binning_x = 1;
    pstCamInfo->binning_y = 1;

    pstCamInfo->roi.width = pstFrame->width;
    pstCamInfo->roi.height = pstFrame->height;
    pstCamInfo->roi.x_offset = 0;
    pstCamInfo->roi.y_offset = 0;
    pstCamInfo->roi.do_rectify = false;

    return 0;
}

int CameraPublisher::genRosRgbCamInfo(const AS_Frame_s *pstFrame,
                                      sensor_msgs::msg::CameraInfo *pstCamInfo, AS_CAM_Parameter_s &stParameter)
{
    std::array<double, 9> R = {1, 0, 0,
                               0, 1, 0,
                               0, 0, 1
                              };

    pstCamInfo->width = pstFrame->width;
    pstCamInfo->height = pstFrame->height;

    pstCamInfo->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    pstCamInfo->d.resize(5);
    pstCamInfo->d[0] = 0;
    pstCamInfo->d[1] = 0;
    pstCamInfo->d[2] = 0;
    pstCamInfo->d[3] = 0;
    pstCamInfo->d[4] = 0;

    pstCamInfo->k.fill(0.0);
    pstCamInfo->k[0] = stParameter.fxrgb;
    pstCamInfo->k[2] = stParameter.cxrgb;
    pstCamInfo->k[4] = stParameter.fyrgb;
    pstCamInfo->k[5] = stParameter.cyrgb;
    pstCamInfo->k[8] = 1.0;

    pstCamInfo->r = R;

    pstCamInfo->p.fill(0.0);
    pstCamInfo->p[0] = pstCamInfo->k[0];
    pstCamInfo->p[2] = pstCamInfo->k[2];
    pstCamInfo->p[3] = 0.0;
    pstCamInfo->p[5] = pstCamInfo->k[4];
    pstCamInfo->p[6] = pstCamInfo->k[5];
    pstCamInfo->p[7] = 0;
    pstCamInfo->p[10] = 1.0;
    pstCamInfo->p[11] = 0;

    pstCamInfo->binning_x = 1;
    pstCamInfo->binning_y = 1;

    pstCamInfo->roi.width = pstFrame->width;
    pstCamInfo->roi.height = pstFrame->height;
    pstCamInfo->roi.x_offset = 0;
    pstCamInfo->roi.y_offset = 0;
    pstCamInfo->roi.do_rectify = false;

    return 0;
}

int CameraPublisher::genRosIrCamInfo(const AS_Frame_s *pstFrame,
                                     sensor_msgs::msg::CameraInfo *pstCamInfo,
                                     AS_CAM_Parameter_s &stParameter, bool registration)
{
    std::array<double, 9> R = {1, 0, 0,
                               0, 1, 0,
                               0, 0, 1
                              };

    pstCamInfo->width = pstFrame->width;
    pstCamInfo->height = pstFrame->height;

    pstCamInfo->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    pstCamInfo->d.resize(5);
    pstCamInfo->d[0] = 0;
    pstCamInfo->d[1] = 0;
    pstCamInfo->d[2] = 0;
    pstCamInfo->d[3] = 0;
    pstCamInfo->d[4] = 0;

    pstCamInfo->k.fill(0.0);
    if (registration) {
        pstCamInfo->k[0] = stParameter.fxrgb;
        pstCamInfo->k[2] = stParameter.cxrgb;
        pstCamInfo->k[4] = stParameter.fyrgb;
        pstCamInfo->k[5] = stParameter.cyrgb;
    } else {
        pstCamInfo->k[0] = stParameter.fxir;
        pstCamInfo->k[2] = stParameter.cxir;
        pstCamInfo->k[4] = stParameter.fyir;
        pstCamInfo->k[5] = stParameter.cyir;
    }

    pstCamInfo->k[8] = 1.0;

    pstCamInfo->r = R;

    pstCamInfo->p.fill(0.0);
    pstCamInfo->p[0] = pstCamInfo->k[0];
    pstCamInfo->p[2] = pstCamInfo->k[2];
    pstCamInfo->p[3] = 0.0;
    pstCamInfo->p[5] = pstCamInfo->k[4];
    pstCamInfo->p[6] = pstCamInfo->k[5];
    pstCamInfo->p[7] = 0;
    pstCamInfo->p[10] = 1.0;
    pstCamInfo->p[11] = 0;

    pstCamInfo->binning_x = 1;
    pstCamInfo->binning_y = 1;

    pstCamInfo->roi.width = pstFrame->width;
    pstCamInfo->roi.height = pstFrame->height;
    pstCamInfo->roi.x_offset = 0;
    pstCamInfo->roi.y_offset = 0;
    pstCamInfo->roi.do_rectify = false;

    return 0;
}

int CameraPublisher::genRosPeakCamInfo(const AS_Frame_s *pstFrame,
                                       sensor_msgs::msg::CameraInfo *pstCamInfo, AS_CAM_Parameter_s &stParameter)
{
    std::array<double, 9> R = {1, 0, 0,
                               0, 1, 0,
                               0, 0, 1
                              };

    pstCamInfo->width = pstFrame->width;
    pstCamInfo->height = pstFrame->height;

    pstCamInfo->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    pstCamInfo->d.resize(5);
    pstCamInfo->d[0] = 0;
    pstCamInfo->d[1] = 0;
    pstCamInfo->d[2] = 0;
    pstCamInfo->d[3] = 0;
    pstCamInfo->d[4] = 0;

    pstCamInfo->k.fill(0.0);
    pstCamInfo->k[0] = stParameter.fxir;
    pstCamInfo->k[2] = stParameter.cxir;
    pstCamInfo->k[4] = stParameter.fyir;
    pstCamInfo->k[5] = stParameter.cyir;
    pstCamInfo->k[8] = 1.0;

    pstCamInfo->r = R;

    pstCamInfo->p.fill(0.0);
    pstCamInfo->p[0] = pstCamInfo->k[0];
    pstCamInfo->p[2] = pstCamInfo->k[2];
    pstCamInfo->p[3] = 0.0;
    pstCamInfo->p[5] = pstCamInfo->k[4];
    pstCamInfo->p[6] = pstCamInfo->k[5];
    pstCamInfo->p[7] = 0;
    pstCamInfo->p[10] = 1.0;
    pstCamInfo->p[11] = 0;

    pstCamInfo->binning_x = 1;
    pstCamInfo->binning_y = 1;

    pstCamInfo->roi.width = pstFrame->width;
    pstCamInfo->roi.height = pstFrame->height;
    pstCamInfo->roi.x_offset = 0;
    pstCamInfo->roi.y_offset = 0;
    pstCamInfo->roi.do_rectify = false;

    return 0;
}

int CameraPublisher::genRosDepthImage(const AS_Frame_s *pstFrame, sensor_msgs::msg::Image *pstImage)
{
    pstImage->height = pstFrame->height;
    pstImage->width = pstFrame->width;

    if (pstFrame->size == pstFrame->width * pstFrame->height * 2) {
        pstImage->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        pstImage->step = pstFrame->width * 2;
    } else {
        pstImage->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        pstImage->step = pstFrame->width * 4;
    }
    pstImage->is_bigendian = false;

    std::vector<unsigned char> vcTmp((unsigned char *)pstFrame->data,
                                     (unsigned char *)pstFrame->data + pstFrame->size);
    pstImage->data = vcTmp;

    return 0;
}

int CameraPublisher::genRosRgbImage(const AS_Frame_s *pstFrame, sensor_msgs::msg::Image *pstImage)
{
    pstImage->height = pstFrame->height;
    pstImage->width = pstFrame->width;

    pstImage->encoding = sensor_msgs::image_encodings::BGR8;
    pstImage->is_bigendian = false;
    pstImage->step = pstFrame->width * 3;

    std::vector<unsigned char> vcTmp((unsigned char *)pstFrame->data,
                                     (unsigned char *)pstFrame->data + pstFrame->size);
    pstImage->data = vcTmp;

    return 0;
}

int CameraPublisher::genRosIrImage(const AS_Frame_s *pstFrame, sensor_msgs::msg::Image *pstImage)
{
    pstImage->height = pstFrame->height;
    pstImage->width = pstFrame->width;

    pstImage->encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    pstImage->is_bigendian = false;
    pstImage->step = pstFrame->width;

    std::vector<unsigned char> vcTmp((unsigned char *)pstFrame->data,
                                     (unsigned char *)pstFrame->data + pstFrame->size);
    pstImage->data = vcTmp;

    return 0;
}

int CameraPublisher::genRosPeakImage(const AS_Frame_s *pstFrame, sensor_msgs::msg::Image *pstImage)
{
    pstImage->height = pstFrame->height;
    pstImage->width = pstFrame->width;

    pstImage->encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    pstImage->is_bigendian = false;
    pstImage->step = pstFrame->width;

    std::vector<unsigned char> vcTmp((unsigned char *)pstFrame->data,
                                     (unsigned char *)pstFrame->data + pstFrame->size);
    pstImage->data = vcTmp;

    return 0;
}

void CameraPublisher::depthInfoPublisher(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData,
        AS_CAM_Parameter_s &stParameter, builtin_interfaces::msg::Time time)
{
    if (pstData->depthImg.size == 0) {
        return;
    }
    RCLCPP_INFO_STREAM_ONCE(m_logger, "publish depth info");
    sensor_msgs::msg::CameraInfo stDepthCamInfo;
    sensor_msgs::msg::Image stDepthImage;

    stDepthCamInfo.header.stamp = time;
    stDepthImage.header.stamp = time;

    AS_SDK_CAM_MODEL_E cam_type = AS_SDK_CAM_MODEL_UNKNOWN;
    auto it_par = m_cam_type_map.find(pCamera);
    if ( it_par != m_cam_type_map.end()) {
        cam_type = it_par->second;
    }

    for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
        if (it->pCamera == pCamera) {
            if (cam_type != AS_SDK_CAM_MODEL_KONDYOR) {
                genRosDepthCamInfo(&pstData->depthImg, &stDepthCamInfo, stParameter, it->config_info.is_Registration);
            }

            genRosDepthImage(&pstData->depthImg, &stDepthImage);
            if (rclcpp::ok()) {
                auto camIt = m_camera_map.find(pCamera);
                if (camIt == m_camera_map.end()) {
                    RCLCPP_ERROR_STREAM(m_logger, "error camera ptr");
                    return;
                }
                if (m_launch_param.pub_tfTree) {
                    if (it->config_info.is_Registration) {
                        stDepthImage.header.frame_id = camIt->second->getColorFrameId();
                        stDepthCamInfo.header.frame_id = camIt->second->getColorFrameId();
                    } else {
                        stDepthImage.header.frame_id = camIt->second->getDepthFrameId();
                        stDepthCamInfo.header.frame_id = camIt->second->getDepthFrameId();
                    }
                } else {
                    stDepthImage.header.frame_id = camIt->second->getDefaultFrameId();
                    stDepthCamInfo.header.frame_id = camIt->second->getDefaultFrameId();
                }

                if (cam_type != AS_SDK_CAM_MODEL_KONDYOR) {
                    it->depth_camera_info_pub->publish(stDepthCamInfo);
                }
                if (cam_type == AS_SDK_CAM_MODEL_KUNLUN_A) {
                    if (pstData->depthImg.height == 96) {
                        it->depth_odd_raw_pub->publish(stDepthImage);
                    } else if (pstData->depthImg.height == 8) {
                        it->depth_even_raw_pub->publish(stDepthImage);
                    }
                } else {
                    it->depth_raw_pub->publish(stDepthImage);
                }
            }
            break;
        }
    }

    return;
}

void CameraPublisher::pointcloudPublisher(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pData,
        AS_CAM_Parameter_s &stParameter)
{
    if (pData->pointCloud.size == 0) {
        return;
    }
    RCLCPP_INFO_STREAM_ONCE(m_logger, "publish pointcloud");
    auto pointCloudMsg = sensor_msgs::msg::PointCloud2();
    auto pointCloudMsgRgb = sensor_msgs::msg::PointCloud2();
    // sensor_msgs::PointCloud2 pointCloudMsgRgb;
    auto camIt = m_camera_map.find(pCamera);
    if (camIt == m_camera_map.end()) {
        RCLCPP_ERROR_STREAM(m_logger, "error camera ptr");
        return;
    }

    /* publish point cloud */
    if (m_launch_param.color_pcl == false) {
        pcl::PointCloud<pcl::PointXYZ> stPointCloud;
        for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
            if (it->pCamera == pCamera) {
                if (it->config_info.is_pclOrganized) {
                    stPointCloud.width = pData->pointCloud.width;
                    stPointCloud.height = pData->pointCloud.height;
                } else {
                    stPointCloud.width = pData->pointCloud.size / sizeof(float) / 3;
                    stPointCloud.height = 1;
                }
            }
        }
        stPointCloud.points.resize(stPointCloud.width * stPointCloud.height);

        for (unsigned int i = 0 ; i < stPointCloud.points.size(); i++) {
            int index = i * 3;
            stPointCloud.points[i].x = *((float *)pData->pointCloud.data + index) / 1000;
            stPointCloud.points[i].y = *((float *)pData->pointCloud.data + index + 1) / 1000;
            stPointCloud.points[i].z = *((float *)pData->pointCloud.data + index + 2) / 1000;
        }

        pcl::toROSMsg(stPointCloud, pointCloudMsg);
        pointCloudMsg.header.stamp = this->get_clock()->now();

        AS_SDK_CAM_MODEL_E cam_type = AS_SDK_CAM_MODEL_UNKNOWN;
        auto it_par = m_cam_type_map.find(pCamera);
        if ( it_par != m_cam_type_map.end()) {
            cam_type = it_par->second;
        }

        if (rclcpp::ok()) {
            for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
                if (it->pCamera == pCamera) {
                    if (m_launch_param.pub_tfTree) {
                        if (it->config_info.is_Registration) {
                            pointCloudMsg.header.frame_id = camIt->second->getColorFrameId();
                        } else {
                            pointCloudMsg.header.frame_id = camIt->second->getDepthFrameId();
                        }
                    } else {
                        pointCloudMsg.header.frame_id =  camIt->second->getDefaultFrameId();
                    }

                    if (cam_type == AS_SDK_CAM_MODEL_KUNLUN_A) {
                        if (pData->pointCloud.height == 96) {
                            it->depth_points_odd_pub->publish(pointCloudMsg);
                        } else if (pData->pointCloud.height == 8) {
                            it->depth_points_even_pub->publish(pointCloudMsg);
                        }
                    } else {
                        it->depth_points_pub->publish(pointCloudMsg);
                    }
                }
            }
        }
    } else { /* publish point cloud with rgb */
        pcl::PointCloud<pcl::PointXYZRGB> stPointCloudRgb;
        for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
            if (it->pCamera == pCamera) {
                if (it->config_info.is_pclOrganized) {
                    stPointCloudRgb.width = pData->pointCloud.width;
                    stPointCloudRgb.height = pData->pointCloud.height;
                } else {
                    stPointCloudRgb.width = pData->pointCloud.size / sizeof(float) / 3;
                    stPointCloudRgb.height = 1;
                }
            }
        }
        bool decimal = (pData->depthImg.size == pData->depthImg.width * pData->depthImg.height * 2) ? false : true;
        for (unsigned int y = 0; y < pData->depthImg.height; y++) {
            for (unsigned int x = 0; x < pData->depthImg.width; x++) {
                double depth = 0.0;
                if (decimal) {
                    depth = ((float *)pData->depthImg.data)[y * pData->depthImg.width + x];
                } else {
                    depth = ((uint16_t *)pData->depthImg.data)[y * pData->depthImg.width + x];
                }
                if (depth < 0.0001)
                    continue;
                pcl::PointXYZRGB p;

                p.z = depth / 1000;
                p.x = (x - stParameter.cxrgb) * p.z / stParameter.fxrgb;
                p.y = (y - stParameter.cyrgb) * p.z / stParameter.fyrgb;

                p.b = ((unsigned char *)pData->rgbImg.data)[ (y * pData->depthImg.width + x) * 3 ];
                p.g = ((unsigned char *)pData->rgbImg.data)[ (y * pData->depthImg.width + x) * 3 + 1];
                p.r = ((unsigned char *)pData->rgbImg.data)[ (y * pData->depthImg.width + x ) * 3 + 2];

                stPointCloudRgb.points.push_back(p);
            }
        }
        pcl::toROSMsg(stPointCloudRgb, pointCloudMsgRgb);
        pointCloudMsgRgb.header.stamp = this->get_clock()->now();

        if (rclcpp::ok()) {
            for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
                if (it->pCamera == pCamera) {
                    if (m_launch_param.pub_tfTree) {
                        if (it->config_info.is_Registration) {
                            pointCloudMsgRgb.header.frame_id = camIt->second->getColorFrameId();
                        } else {
                            pointCloudMsgRgb.header.frame_id = camIt->second->getDepthFrameId();
                        }
                    } else {
                        pointCloudMsgRgb.header.frame_id = camIt->second->getDefaultFrameId();
                    }
                    it->depth_points_pub->publish(pointCloudMsgRgb);
                }
            }
        }
    }

}

void CameraPublisher::rgbInfoPublisher(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData,
                                       AS_CAM_Parameter_s &stParameter, builtin_interfaces::msg::Time time)
{
    if (pstData->rgbImg.size == 0) {
        return;
    }
    RCLCPP_INFO_STREAM_ONCE(m_logger, "publish color(rgb) info");
    sensor_msgs::msg::CameraInfo stRgbCamInfo;
    sensor_msgs::msg::Image rgbImage;

    stRgbCamInfo.header.stamp = time;
    rgbImage.header.stamp = time;

    genRosRgbCamInfo(&pstData->rgbImg, &stRgbCamInfo, stParameter);
    genRosRgbImage(&pstData->rgbImg, &rgbImage);

    auto camIt = m_camera_map.find(pCamera);
    if (camIt == m_camera_map.end()) {
        RCLCPP_ERROR_STREAM(m_logger, "error camera ptr");
        return;
    }
    if (rclcpp::ok()) {
        for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
            if (it->pCamera == pCamera) {
                if (m_launch_param.pub_tfTree) {
                    stRgbCamInfo.header.frame_id = camIt->second->getColorFrameId();
                    rgbImage.header.frame_id = camIt->second->getColorFrameId();
                } else {
                    stRgbCamInfo.header.frame_id = camIt->second->getDefaultFrameId();
                    rgbImage.header.frame_id = camIt->second->getDefaultFrameId();
                }
                it->rgb_camera_info_pub->publish(stRgbCamInfo);
                it->rgb_img_raw_pub->publish(rgbImage);
            }
        }
    }
}

void CameraPublisher::irInfoPublisher(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData,
                                      AS_CAM_Parameter_s &stParameter, builtin_interfaces::msg::Time time)
{
    if (pstData->irImg.size == 0) {
        return;
    }
    RCLCPP_INFO_STREAM_ONCE(m_logger, "publish infrared info");
    sensor_msgs::msg::CameraInfo stIrCamInfo;
    sensor_msgs::msg::Image irImage;
    genRosIrCamInfo(&pstData->irImg, &stIrCamInfo, stParameter);
    genRosIrImage(&pstData->irImg, &irImage);
    auto camIt = m_camera_map.find(pCamera);
    if (camIt == m_camera_map.end()) {
        RCLCPP_ERROR_STREAM(m_logger, "error camera ptr");
        return;
    }
    if (rclcpp::ok()) {
        for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
            if (it->pCamera == pCamera) {
                if (m_launch_param.pub_tfTree) {
                    stIrCamInfo.header.frame_id = camIt->second->getDepthFrameId();
                    irImage.header.frame_id = camIt->second->getDepthFrameId();
                } else {
                    stIrCamInfo.header.frame_id = camIt->second->getDefaultFrameId();
                    irImage.header.frame_id = camIt->second->getDefaultFrameId();
                }
                it->ir_camera_info_pub->publish(stIrCamInfo);
                it->ir_img_pub->publish(irImage);
            }
        }
    }
}

void CameraPublisher::peakInfoPublisher(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData,
                                        AS_CAM_Parameter_s &stParameter, builtin_interfaces::msg::Time time)
{
    if (pstData->peakImg.size == 0) {
        return;
    }
    RCLCPP_INFO_STREAM_ONCE(m_logger, "publish peak info");
    sensor_msgs::msg::CameraInfo stPeakCamInfo;
    sensor_msgs::msg::Image peakImage;
    genRosPeakCamInfo(&pstData->peakImg, &stPeakCamInfo, stParameter);
    genRosPeakImage(&pstData->peakImg, &peakImage);

    AS_SDK_CAM_MODEL_E cam_type = AS_SDK_CAM_MODEL_UNKNOWN;
    auto it_par = m_cam_type_map.find(pCamera);
    if ( it_par != m_cam_type_map.end()) {
        cam_type = it_par->second;
    }

    auto camIt = m_camera_map.find(pCamera);
    if (camIt == m_camera_map.end()) {
        RCLCPP_ERROR_STREAM(m_logger, "error camera ptr");
        return;
    }
    if (rclcpp::ok()) {
        for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
            if (it->pCamera == pCamera) {
                if (m_launch_param.pub_tfTree) {
                    stPeakCamInfo.header.frame_id = camIt->second->getDepthFrameId();
                    peakImage.header.frame_id = camIt->second->getDepthFrameId();
                } else {
                    stPeakCamInfo.header.frame_id = camIt->second->getDefaultFrameId();
                    peakImage.header.frame_id = camIt->second->getDefaultFrameId();
                }
                it->peak_camera_info_pub->publish(stPeakCamInfo);
                if (cam_type == AS_SDK_CAM_MODEL_KUNLUN_A) {
                    if (pstData->peakImg.height == 96) {
                        it->peak_odd_img_pub->publish(peakImage);
                    } else if (pstData->peakImg.height == 8) {
                        it->peak_even_img_pub->publish(peakImage);
                    }
                } else {
                    it->peak_img_pub->publish(peakImage);
                }
            }
        }
    }
}

void CameraPublisher::depthMergeInfoPublisher(AS_CAM_PTR pCamera, const AS_SDK_MERGE_s *pstData,
        AS_CAM_Parameter_s &stParameter, builtin_interfaces::msg::Time time)
{
    AS_SDK_CAM_MODEL_E cam_type = AS_SDK_CAM_MODEL_UNKNOWN;
    auto it_par = m_cam_type_map.find(pCamera);
    if ( it_par != m_cam_type_map.end()) {
        cam_type = it_par->second;
    }
    if (cam_type != AS_SDK_CAM_MODEL_KUNLUN_A) {
        return;
    }

    if (pstData->depthImg.size == 0) {
        return;
    }
    sensor_msgs::msg::Image stDepthImage;
    stDepthImage.header.stamp = time;
    genRosDepthImage(&pstData->depthImg, &stDepthImage);

    auto camIt = m_camera_map.find(pCamera);
    if (camIt == m_camera_map.end()) {
        RCLCPP_ERROR_STREAM(m_logger, "error camera ptr");
        return;
    }
    if (rclcpp::ok()) {
        for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
            if (it->pCamera == pCamera) {
                if (m_launch_param.pub_tfTree) {
                    stDepthImage.header.frame_id = camIt->second->getDepthFrameId();
                } else {
                    stDepthImage.header.frame_id = camIt->second->getDefaultFrameId();
                }
                it->depth_merge_raw_pub->publish(stDepthImage);
            }
        }
    }
}

void CameraPublisher::pointcloudMergePublisher(AS_CAM_PTR pCamera, const AS_SDK_MERGE_s *pData,
        AS_CAM_Parameter_s &stParameter)
{
    if (pData->pointCloud.size == 0) {
        return;
    }
    auto pointCloudMsg = sensor_msgs::msg::PointCloud2();
    auto pointCloudMsgRgb = sensor_msgs::msg::PointCloud2();
    // sensor_msgs::PointCloud2 pointCloudMsgRgb;

    /* publish point cloud */
    pcl::PointCloud<pcl::PointXYZ> stPointCloud;
    stPointCloud.width = pData->pointCloud.size / sizeof(float) / 3;
    stPointCloud.height = 1;
    stPointCloud.points.resize(stPointCloud.width * stPointCloud.height);

    for (size_t i = 0 ; i < stPointCloud.points.size(); i++) {
        int index = i * 3;
        stPointCloud.points[i].x = *((float *)pData->pointCloud.data + index) / 1000;
        stPointCloud.points[i].y = *((float *)pData->pointCloud.data + index + 1) / 1000;
        stPointCloud.points[i].z = *((float *)pData->pointCloud.data + index + 2) / 1000;
    }

    pcl::toROSMsg(stPointCloud, pointCloudMsg);
    pointCloudMsg.header.stamp = this->get_clock()->now();

    auto camIt = m_camera_map.find(pCamera);
    if (camIt == m_camera_map.end()) {
        RCLCPP_ERROR_STREAM(m_logger, "error camera ptr");
        return;
    }
    if (rclcpp::ok()) {
        for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
            if (it->pCamera == pCamera) {
                if (m_launch_param.pub_tfTree) {
                    pointCloudMsg.header.frame_id = camIt->second->getDepthFrameId();
                } else {
                    pointCloudMsg.header.frame_id = camIt->second->getDefaultFrameId();
                }
                it->depth_points_merge_pub->publish(pointCloudMsg);
            }
        }
    }
}

void CameraPublisher::tfPublisher(AS_CAM_PTR pCamera, AS_CAM_Parameter_s &stParameter)
{
    if (!m_launch_param.pub_tfTree) {
        return;
    }

    RCLCPP_INFO_STREAM_ONCE(m_logger, "publish tf info");
    static tf2_ros::TransformBroadcaster br(*this);
    auto camIt = m_camera_map.find(pCamera);
    if (camIt == m_camera_map.end()) {
        RCLCPP_ERROR_STREAM(m_logger, "error camera ptr");
        return;
    }

    for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
        if (it->pCamera == pCamera) {
            auto it_par = m_cam_type_map.find(pCamera);
            if (it_par != m_cam_type_map.end()) {
                if ((it_par->second == AS_SDK_CAM_MODEL_HP60C) || (it_par->second == AS_SDK_CAM_MODEL_HP60CN) ||
                    (it_par->second == AS_SDK_CAM_MODEL_VEGA) || (it_par->second == AS_SDK_CAM_MODEL_KUNLUN_C) ||
                    (it_par->second == AS_SDK_CAM_MODEL_KONDYOR) || (it_par->second == AS_SDK_CAM_MODEL_KONDYOR_NET) ||
                    (it_par->second == AS_SDK_CAM_MODEL_CHANGA) || (it_par->second == AS_SDK_CAM_MODEL_CHANGJIANG_B) ||
                    (it_par->second == AS_SDK_CAM_MODEL_TANGGULA) || (it_par->second == AS_SDK_CAM_MODEL_TAISHAN) ||
                    (it_par->second == AS_SDK_CAM_MODEL_TANGGULA_B)) {
                    geometry_msgs::msg::TransformStamped rgbTransform;
                    rgbTransform.header.stamp = this->get_clock()->now();
                    rgbTransform.header.frame_id = camIt->second->getCamLinkFrameId();
                    rgbTransform.child_frame_id = camIt->second->getColorFrameId();
                    rgbTransform.transform.translation.x = stParameter.T1 / 1000;
                    rgbTransform.transform.translation.y = stParameter.T2 / 1000;
                    rgbTransform.transform.translation.z = stParameter.T3 / 1000;
                    rgbTransform.transform.rotation.x = 0;
                    rgbTransform.transform.rotation.y = 0;
                    rgbTransform.transform.rotation.z = 0;

                    tf2::Quaternion qtn;

                    double sy = std::sqrt(stParameter.R00 * stParameter.R00 + stParameter.R10 * stParameter.R10);
                    double angleX;
                    double angleY;
                    double angleZ;
                    bool singular = sy < 1e-6;
                    if (!singular) {
                        angleX = std::atan2(stParameter.R21, stParameter.R22);
                        angleY = std::atan2(-stParameter.R20, sy);
                        angleZ = std::atan2(stParameter.R10, stParameter.R00);
                    } else {
                        angleX = std::atan2(-stParameter.R12, stParameter.R11);
                        angleY = std::atan2(-stParameter.R20, sy);
                        angleZ = 0;
                    }
                    // std::cout << "angleX:" << angleX << std::endl;
                    // std::cout << "angleY:" << angleY << std::endl;
                    // std::cout << "angleZ:" << angleZ << std::endl;

                    qtn.setRPY(angleX, angleY, angleZ);
                    rgbTransform.transform.rotation.x = qtn.getX();
                    rgbTransform.transform.rotation.y = qtn.getY();
                    rgbTransform.transform.rotation.z = qtn.getZ();
                    rgbTransform.transform.rotation.w = qtn.getW();

                    br.sendTransform(rgbTransform);
                }
            }

            geometry_msgs::msg::TransformStamped depthTransform;
            depthTransform.header.stamp = this->get_clock()->now();
            depthTransform.header.frame_id = camIt->second->getCamLinkFrameId();
            if (it->config_info.is_Registration) {
                depthTransform.child_frame_id = camIt->second->getColorFrameId();
            } else {
                depthTransform.child_frame_id = camIt->second->getDepthFrameId();
            }
            depthTransform.transform.translation.x = 0.0;
            depthTransform.transform.translation.y = 0.0;
            depthTransform.transform.translation.z = 0.0;
            depthTransform.transform.rotation.x = 0.0;
            depthTransform.transform.rotation.y = 0.0;
            depthTransform.transform.rotation.z = 0.0;
            depthTransform.transform.rotation.w = 1.0;

            br.sendTransform(depthTransform);
        }
    }
}

int CameraPublisher::setResolution(AS_CAM_PTR pCamera, LAUNCH_CONFI_PARAM_S para)
{
    int ret = 0;
    if ((para.set_depth_width != -1) && (para.set_depth_height != -1) && (para.set_fps != -1)) {
        AS_STREAM_Param_s depthInfo;
        depthInfo.width = para.set_depth_width;
        depthInfo.height = para.set_depth_height;
        depthInfo.fps = para.set_fps;

        ret = AS_SDK_SetStreamParam(pCamera, AS_MEDIA_TYPE_DEPTH, &depthInfo);
        if (ret < 0) {
            RCLCPP_ERROR_STREAM(m_logger, "set depth param fail");
            return ret;
        }
        RCLCPP_INFO_STREAM(m_logger, "set depth resolution: " << depthInfo.width << " x " << depthInfo.height << " @ " <<
                           depthInfo.fps << "fps");
    }
    if ((para.set_rgb_width != -1) && (para.set_rgb_height != -1) && (para.set_fps != -1)) {
        AS_STREAM_Param_s rgbInfo;
        rgbInfo.width = para.set_rgb_width;
        rgbInfo.height = para.set_rgb_height;
        rgbInfo.fps = para.set_fps;

        ret = AS_SDK_SetStreamParam(pCamera, AS_MEDIA_TYPE_RGB, &rgbInfo);
        if (ret < 0) {
            RCLCPP_ERROR_STREAM(m_logger, "set rgb param fail");
            return ret;
        }
        RCLCPP_INFO_STREAM(m_logger, "set rgb resolution: " << rgbInfo.width << " x " << rgbInfo.height << " @ " << rgbInfo.fps
                           << "fps");
    }
    if ((para.set_ir_width != -1) && (para.set_ir_height != -1) && (para.set_fps != -1)) {
        AS_STREAM_Param_s irInfo;
        irInfo.width = para.set_ir_width;
        irInfo.height = para.set_ir_height;
        irInfo.fps = para.set_fps;

        ret = AS_SDK_SetStreamParam(pCamera, AS_MEDIA_TYPE_IR, &irInfo);
        if (ret < 0) {
            RCLCPP_ERROR_STREAM(m_logger, "set ir param fail");
            return ret;
        }
        RCLCPP_INFO_STREAM(m_logger, "set ir resolution: " << irInfo.width << " x " << irInfo.height << " @ " << irInfo.fps <<
                           "fps");
    }
    if ((para.set_peak_width != -1) && (para.set_peak_height != -1) && (para.set_fps != -1)) {
        AS_STREAM_Param_s peakInfo;
        peakInfo.width = para.set_peak_width;
        peakInfo.height = para.set_peak_height;
        peakInfo.fps = para.set_fps;

        ret = AS_SDK_SetStreamParam(pCamera, AS_MEDIA_TYPE_PEAK, &peakInfo);
        if (ret < 0) {
            RCLCPP_ERROR_STREAM(m_logger, "set peak param fail");
            return ret;
        }
        RCLCPP_INFO_STREAM(m_logger, "set peak resolution: " << peakInfo.width << " x " << peakInfo.height << " @ " <<
                           peakInfo.fps << "fps");
    }
    return ret;
}

void CameraPublisher::saveImage(const std::string &serialno, const AS_SDK_Data_s *pstData)
{
    static int depthindex = 0;
    static int pointCloudIndex = 0;
    static int rgbindex = 0;
    static int irindex = 0;
    static int peakindex = 0;

    if (pstData->depthImg.size > 0) {
        std::string depthImgName(std::string("depth_" + std::to_string(pstData->depthImg.width) + "x" +
                                             std::to_string(pstData->depthImg.height)
                                             + "_" + std::to_string(depthindex++) + ".yuv"));
        if (saveYUVImg(depthImgName.c_str(), pstData->depthImg.data, pstData->depthImg.size) != 0) {
            RCLCPP_ERROR_STREAM(m_logger, "save img failed!");
        } else {
            RCLCPP_INFO_STREAM(m_logger, serialno << ", save depth image success!");
            RCLCPP_INFO_STREAM(m_logger, "location: " << getcwd(nullptr, 0) << "/" << depthImgName);
        }
    }

    if (pstData->pointCloud.size > 0) {
        std::string pointCloudName(std::string("PointCloud_" + std::to_string(pointCloudIndex++) + ".txt"));
        if (savePointCloud(pointCloudName.c_str(), (float *)pstData->pointCloud.data,
                           pstData->pointCloud.size / sizeof(float)) != 0) {
            RCLCPP_ERROR_STREAM(m_logger, "save point cloud failed!");
        } else {
            RCLCPP_INFO_STREAM(m_logger, "save point cloud success!");
            RCLCPP_INFO_STREAM(m_logger, "location: " << getcwd(nullptr, 0) << "/" << pointCloudName);
        }
    }

    if (pstData->rgbImg.size > 0) {
        std::string rgbName(std::string("rgb_" + std::to_string(pstData->rgbImg.width) + "x" +
                                        std::to_string(pstData->rgbImg.height)
                                        + "_" + std::to_string(rgbindex++) + ".yuv"));
        if (saveYUVImg(rgbName.c_str(), pstData->rgbImg.data, pstData->rgbImg.size) != 0) {
            RCLCPP_ERROR_STREAM(m_logger, "save rgb image failed!");
        } else {
            RCLCPP_INFO_STREAM(m_logger, "save rgb image success!");
            RCLCPP_INFO_STREAM(m_logger, "location: " << getcwd(nullptr, 0) << "/" << rgbName);
        }
    }

    if (pstData->irImg.size > 0) {
        std::string irName(std::string("ir_" + std::to_string(pstData->irImg.width) + "x" +
                                       std::to_string(pstData->irImg.height)
                                       + "_" + std::to_string(irindex++) + ".yuv"));
        if (saveYUVImg(irName.c_str(), pstData->irImg.data, pstData->irImg.size) != 0) {
            RCLCPP_ERROR_STREAM(m_logger, "save ir image failed!");
        } else {
            RCLCPP_INFO_STREAM(m_logger, "save ir image success!");
            RCLCPP_INFO_STREAM(m_logger, "location: " << getcwd(nullptr, 0) << "/" << irName);
        }
    }

    if (pstData->peakImg.size > 0) {
        std::string peakName(std::string("peak_" + std::to_string(pstData->peakImg.width) + "x" +
                                         std::to_string(pstData->peakImg.height)
                                         + "_" + std::to_string(peakindex++) + ".yuv"));
        if (saveYUVImg(peakName.c_str(), pstData->peakImg.data, pstData->peakImg.size) != 0) {
            RCLCPP_ERROR_STREAM(m_logger, "save peak image failed!");
        } else {
            RCLCPP_INFO_STREAM(m_logger, "save peak image success!");
            RCLCPP_INFO_STREAM(m_logger, "location: " << getcwd(nullptr, 0) << "/" << peakName);
        }
    }

}

void CameraPublisher::logCfgParameter()
{
    for (auto it = m_camera_map.begin(); it != m_camera_map.end(); it++) {
        AS_SDK_LogCameraCfg(it->first);
    }
}

template<typename T>
void getSubTopicStreamType(const typename rclcpp::Publisher<T>::SharedPtr &publisher, unsigned int &streamType,
                           unsigned int idx)
{
    if (publisher) {
        if (publisher->get_subscription_count()) {
            std::string topic_name = publisher->get_topic_name();
            if ((topic_name.find("depth" + std::to_string(idx)) != std::string::npos) &&
                topic_name.find("points") != std::string::npos) {
                streamType = POINTCLOUD_IMG_FLG | DEPTH_IMG_FLG;
            } else if ((topic_name.find("connectDepth" + std::to_string(idx)) != std::string::npos)) {
                streamType = DEPTH_IMG_FLG | SUB_DEPTH_IMG_FLG;
            } else if ((topic_name.find("connectDepth" + std::to_string(idx)) != std::string::npos) &&
                       topic_name.find("points") != std::string::npos) {
                streamType = POINTCLOUD_IMG_FLG | DEPTH_IMG_FLG | SUB_DEPTH_IMG_FLG;
            } else if (topic_name.find("depth" + std::to_string(idx)) != std::string::npos) {
                streamType = DEPTH_IMG_FLG;
            } else if (topic_name.find("ir" + std::to_string(idx)) != std::string::npos) {
                streamType = IR_IMG_FLG;
            } else if ((topic_name.find("rgb" + std::to_string(idx)) != std::string::npos)
                       || (topic_name.find("mono8" + std::to_string(idx)) != std::string::npos)) {
                streamType = RGB_IMG_FLG;
            } else if (topic_name.find("peak" + std::to_string(idx)) != std::string::npos) {
                streamType = PEAK_IMG_FLG;
            } else if (topic_name.find("yuyv" + std::to_string(idx)) != std::string::npos) {
                streamType = YUYV_IMG_FLG;
            } else {
                streamType = 0;
            }
        } else {
            streamType = 0;
        }
    } else {
        streamType = 0;
    }

    return;
}

void CameraPublisher::getSubListStreamType(const PUBLISHER_INFO_S &publisherInfo, unsigned int &type)
{
    unsigned int streamType = 0;
    type = 0;

    // depth
    getSubTopicStreamType<sensor_msgs::msg::CameraInfo>(publisherInfo.depth_camera_info_pub, streamType,
            publisherInfo.index);
    type |= streamType;
    getSubTopicStreamType<sensor_msgs::msg::Image>(publisherInfo.depth_raw_pub, streamType, publisherInfo.index);
    type |= streamType;
    getSubTopicStreamType<sensor_msgs::msg::Image>(publisherInfo.depth_odd_raw_pub, streamType, publisherInfo.index);
    type |= streamType;
    getSubTopicStreamType<sensor_msgs::msg::Image>(publisherInfo.depth_even_raw_pub, streamType, publisherInfo.index);
    type |= streamType;
    getSubTopicStreamType<sensor_msgs::msg::Image>(publisherInfo.depth_merge_raw_pub, streamType, publisherInfo.index);
    type |= streamType;

    // ir
    getSubTopicStreamType<sensor_msgs::msg::CameraInfo>(publisherInfo.ir_camera_info_pub, streamType, publisherInfo.index);
    type |= streamType;
    getSubTopicStreamType<sensor_msgs::msg::Image>(publisherInfo.ir_img_pub, streamType, publisherInfo.index);
    type |= streamType;

    // rgb
    getSubTopicStreamType<sensor_msgs::msg::CameraInfo>(publisherInfo.rgb_camera_info_pub, streamType, publisherInfo.index);
    type |= streamType;
    getSubTopicStreamType<sensor_msgs::msg::Image>(publisherInfo.rgb_img_raw_pub, streamType, publisherInfo.index);
    type |= streamType;

    // peak
    getSubTopicStreamType<sensor_msgs::msg::CameraInfo>(publisherInfo.peak_camera_info_pub, streamType,
            publisherInfo.index);
    type |= streamType;
    getSubTopicStreamType<sensor_msgs::msg::Image>(publisherInfo.peak_img_pub, streamType, publisherInfo.index);
    type |= streamType;
    getSubTopicStreamType<sensor_msgs::msg::Image>(publisherInfo.peak_odd_img_pub, streamType, publisherInfo.index);
    type |= streamType;
    getSubTopicStreamType<sensor_msgs::msg::Image>(publisherInfo.peak_even_img_pub, streamType, publisherInfo.index);
    type |= streamType;

    // point
    getSubTopicStreamType<sensor_msgs::msg::PointCloud2>(publisherInfo.depth_points_pub, streamType, publisherInfo.index);
    type |= streamType;
    getSubTopicStreamType<sensor_msgs::msg::PointCloud2>(publisherInfo.depth_points_odd_pub, streamType,
            publisherInfo.index);
    type |= streamType;
    getSubTopicStreamType<sensor_msgs::msg::PointCloud2>(publisherInfo.depth_points_even_pub, streamType,
            publisherInfo.index);
    type |= streamType;
    getSubTopicStreamType<sensor_msgs::msg::PointCloud2>(publisherInfo.depth_points_merge_pub, streamType,
            publisherInfo.index);
    type |= streamType;

    return;
}

int CameraPublisher::checkStreamContrll(unsigned int allType, unsigned int checkType, AS_CAM_PTR pCamera,
                                        int &streamFlag, AS_APP_CAMERA_STATUS_E status)
{
    int ret = 0;

    if ((allType & checkType) == checkType) {
        if (!((streamFlag & checkType) == checkType)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            if (status != CAMERA_CLOSED_STATUS) {
                if ((m_cam_type_map[pCamera] == AS_SDK_CAM_MODEL_E::AS_SDK_CAM_MODEL_CHANGJIANG_B) ||
                    (m_cam_type_map[pCamera] == AS_SDK_CAM_MODEL_E::AS_SDK_CAM_MODEL_TAISHAN)) {
                    AS_SDK_StopStream(pCamera);
                }
                ret = AS_SDK_StartStream(pCamera, streamFlag | checkType);
                // RCLCPP_INFO_STREAM(m_logger, "AS_SDK_StartStream ret " << ret << ", streamType 0x" << std::hex << checkType << std::dec <<  std::endl;
                if (ret == 0) {
                    onCameraStart(pCamera);
                    streamFlag |= checkType;
                }
            } else {
                streamFlag |= checkType;
            }
        }
    } else {
        if (allType == 0) {
            if (status == CAMERA_STREAM_STATUS) {
                ret = AS_SDK_StopStream(pCamera);
                if (ret == 0) {
                    streamFlag = 0;
                    onCameraStop(pCamera);
                }
            } else {
                streamFlag = 0;
            }
        } else if ((streamFlag & checkType) == checkType) {
            if (status == CAMERA_STREAM_STATUS) {
                ret = AS_SDK_StopStream(pCamera, checkType);
                // RCLCPP_INFO_STREAM(m_logger, "AS_SDK_StopStream ret " << ret << ", streamType 0x" << std::hex << checkType << std::dec);
                if (ret == 0) {
                    streamFlag ^= checkType;
                    if (streamFlag == 0) {
                        onCameraStop(pCamera);
                    }
                }
            } else {
                streamFlag ^= checkType;
            }
        }
    }

    return ret;
}

int CameraPublisher::streamController()
{
    int ret = 0;
    while (rclcpp::ok()) {
        server->getLock().lock();
        for (auto it = imgPubList.begin(); it != imgPubList.end(); it++) {
            unsigned int type = 0;
            getSubListStreamType(*it, type);

            // subscrib depth stream
            ret = checkStreamContrll(type, DEPTH_IMG_FLG, it->pCamera, it->stream_flg, it->camStatus);
            ret = checkStreamContrll(type, RGB_IMG_FLG, it->pCamera, it->stream_flg, it->camStatus);
            ret = checkStreamContrll(type, IR_IMG_FLG, it->pCamera, it->stream_flg, it->camStatus);
            ret = checkStreamContrll(type, POINTCLOUD_IMG_FLG, it->pCamera, it->stream_flg, it->camStatus);
            ret = checkStreamContrll(type, YUYV_IMG_FLG, it->pCamera, it->stream_flg, it->camStatus);
            ret = checkStreamContrll(type, PEAK_IMG_FLG, it->pCamera, it->stream_flg, it->camStatus);
            ret = checkStreamContrll(type, SUB_DEPTH_IMG_FLG, it->pCamera, it->stream_flg, it->camStatus);
        }

        server->getLock().unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return ret;
}

bool CameraPublisher::virtualMachine()
{
    int cnt = 0;
    char szCnt[8];
    FILE *fp = nullptr;

    char cmd[128];
    snprintf(cmd, sizeof(cmd) - 1, R"(lscpu | grep "Hypervisor vendor" | wc -l)");
    fp = popen(cmd, "r");
    if (fgets(szCnt, sizeof(szCnt), fp) != nullptr) {
        if (strlen(szCnt) != 0) {
            cnt = std::stoi(szCnt);
        }
    }
    pclose(fp);
    fp = nullptr;
    if (cnt == 0) {
        return false;
    } else {
        return true;
    }
}

void CameraPublisher::logCameraPathInfo(AS_CAM_ATTR_S &attr_t)
{
    switch (attr_t.type) {
    case AS_CAMERA_ATTR_LNX_USB:
        RCLCPP_INFO_STREAM(m_logger, "usb camera");
        RCLCPP_INFO_STREAM(m_logger, "bnum:" << attr_t.attr.usbAttrs.bnum);
        RCLCPP_INFO_STREAM(m_logger, "dnum:" << attr_t.attr.usbAttrs.dnum);
        RCLCPP_INFO_STREAM(m_logger, "port_numbers:" << attr_t.attr.usbAttrs.port_numbers);
        break;
    case AS_CAMERA_ATTR_NET:
        RCLCPP_INFO_STREAM(m_logger, "net camera");
        RCLCPP_INFO_STREAM(m_logger, "ip:" << attr_t.attr.netAttrs.ip_addr);
        RCLCPP_INFO_STREAM(m_logger, "port:" << attr_t.attr.netAttrs.port);
        break;
    default:
        break;
    }
}

int CameraPublisher::initLaunchParams()
{
    memset(&m_launch_param, 0, sizeof(LAUNCH_CONFI_PARAM_S));
    this->declare_parameter<int>("depth_width", -1);
    this->get_parameter_or<int>("depth_width", m_launch_param.set_depth_width, -1);
    this->declare_parameter<int>("depth_height", -1);
    this->get_parameter_or<int>("depth_height", m_launch_param.set_depth_height, -1);
    this->declare_parameter<int>("rgb_width", -1);
    this->get_parameter_or<int>("rgb_width", m_launch_param.set_rgb_width, -1);
    this->declare_parameter<int>("rgb_height", -1);
    this->get_parameter_or<int>("rgb_height", m_launch_param.set_rgb_height, -1);
    this->declare_parameter<int>("ir_width", -1);
    this->get_parameter_or<int>("ir_width", m_launch_param.set_ir_width, -1);
    this->declare_parameter<int>("ir_height", -1);
    this->get_parameter_or<int>("ir_height", m_launch_param.set_ir_height, -1);
    this->declare_parameter<int>("peak_width", -1);
    this->get_parameter_or<int>("peak_width", m_launch_param.set_peak_width, -1);
    this->declare_parameter<int>("peak_height", -1);
    this->get_parameter_or<int>("peak_height", m_launch_param.set_peak_height, -1);
    this->declare_parameter<int>("fps", -1);
    this->get_parameter_or<int>("fps", m_launch_param.set_fps, -1);

    this->declare_parameter<int>("usb_bus_no", -1);
    this->get_parameter_or<int>("usb_bus_no", m_launch_param.usb_bus_num, -1);

    std::string usb_port_nums;
    this->declare_parameter<std::string>("usb_path", "null");
    this->get_parameter_or<std::string>("usb_path", usb_port_nums, "null");
    strncpy(m_launch_param.usb_port_nums, usb_port_nums.c_str(),
            std::min(sizeof(m_launch_param.usb_port_nums), usb_port_nums.length()));

    this->declare_parameter<bool>("color_pcl", false);
    this->get_parameter_or<bool>("color_pcl", m_launch_param.color_pcl, false);
    this->declare_parameter<bool>("pub_tfTree", true);
    this->get_parameter_or<bool>("pub_tfTree", m_launch_param.pub_tfTree, true);
    printLaunchParams(m_launch_param);

    return 0;
}

int CameraPublisher::printLaunchParams(LAUNCH_CONFI_PARAM_S para)
{
    LOG_PARA(para.set_depth_width, "depth_width");
    LOG_PARA(para.set_depth_height, "depth_height");

    LOG_PARA(para.set_rgb_width, "rgb_width");
    LOG_PARA(para.set_rgb_height, "rgb_height");

    LOG_PARA(para.set_ir_width, "ir_width");
    LOG_PARA(para.set_ir_height, "ir_height");

    LOG_PARA(para.set_peak_width, "peak_width");
    LOG_PARA(para.set_peak_height, "peak_height");

    LOG_PARA(para.set_fps, "set_fps");
    LOG_PARA(para.usb_bus_num, "usb_bus_num");

    if (strncmp(para.usb_port_nums, "null", strlen("null")) != 0) {
        RCLCPP_INFO_STREAM(m_logger, "get usb_port_nums " << para.usb_port_nums);
    }

    if (para.color_pcl != false) {
        RCLCPP_INFO_STREAM(m_logger, "get color_pcl " << para.color_pcl);
    }

    if (para.pub_tfTree != false) {
        RCLCPP_INFO_STREAM(m_logger, "get pub_tfTree " << para.pub_tfTree);
    }

    return 0;
}


