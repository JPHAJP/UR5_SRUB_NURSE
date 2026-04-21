/**
 * @file      CameraPublisher.h
 * @brief     angstrong ros2 camera publisher node.
 *
 * Copyright (c) 2023 Angstrong Tech.Co.,Ltd
 *
 * @author    Angstrong SDK develop Team
 * @date      2023/03/27
 * @version   1.0

 */
#pragma once

#include <thread>
#include <map>
#include <unordered_map>
#include <chrono>
#include <memory>
#include <string>
#include "CameraSrv.h"
#include "Logger.h"
#include "as_camera_sdk_api.h"
#include "common.h"
#include "ascamera_node.h"
#include "Camera.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/distortion_models.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

typedef enum CAMERA_STATUS_E {
    CAMERA_CLOSED_STATUS = 0,
    CAMERA_OPENED_STATUS = 1,
    CAMERA_STREAM_STATUS = 2,
    CAMERA_STREAM_BUUT,
} AS_APP_CAMERA_STATUS_E;

typedef struct PUBLISHER_INFO {
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_camera_info_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_raw_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_points_pub;

    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr ir_camera_info_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ir_img_pub;

    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_camera_info_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_img_raw_pub;

    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr peak_camera_info_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr peak_img_pub;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_odd_raw_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_even_raw_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_merge_raw_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_points_odd_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_points_even_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_points_merge_pub;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr peak_odd_img_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr peak_even_img_pub;

    AS_CAM_PTR pCamera;
    AS_APP_CAMERA_STATUS_E camStatus;
    int stream_flg;
    AS_CAM_ATTR_S attr_s;
    AS_CONFIG_INFO_S config_info;
    unsigned int index;
} PUBLISHER_INFO_S;

class CameraPublisher : public ICameraStatus, public rclcpp::Node
{
public:
    CameraPublisher();
    virtual ~CameraPublisher();
    CameraPublisher(const CameraPublisher &) = delete;
    CameraPublisher &operator = (const CameraPublisher &) = delete;

public:
    int start();
    void stop();
    void saveImage();
    void logFps(bool enable);
    bool getLogFps();
    void logCfgParameter();

private:
    virtual int onCameraAttached(AS_CAM_PTR pCamera, CamSvrStreamParam_s &param,
                                 const AS_SDK_CAM_MODEL_E &cam_type) override;
    virtual int onCameraDetached(AS_CAM_PTR pCamera) override;
    virtual int onCameraOpen(AS_CAM_PTR pCamera) override;
    virtual int onCameraClose(AS_CAM_PTR pCamera) override;
    virtual int onCameraStart(AS_CAM_PTR pCamera) override;
    virtual int onCameraStop(AS_CAM_PTR pCamera) override;
    virtual void onCameraNewFrame(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData) override;
    virtual void onCameraNewMergeFrame(AS_CAM_PTR pCamera, const AS_SDK_MERGE_s *pstData) override;

private:
    int genRosDepthCamInfo(const AS_Frame_s *pstFrame, sensor_msgs::msg::CameraInfo *pstCamInfo,
                           AS_CAM_Parameter_s &stParameter, bool registration = false);
    int genRosRgbCamInfo(const AS_Frame_s *pstFrame, sensor_msgs::msg::CameraInfo *pstCamInfo,
                         AS_CAM_Parameter_s &stParameter);
    int genRosIrCamInfo(const AS_Frame_s *pstFrame, sensor_msgs::msg::CameraInfo *pstCamInfo,
                        AS_CAM_Parameter_s &stParameter, bool registration = false);
    int genRosPeakCamInfo(const AS_Frame_s *pstFrame, sensor_msgs::msg::CameraInfo *pstCamInfo,
                          AS_CAM_Parameter_s &stParameter);

    int genRosDepthImage(const AS_Frame_s *pstFrame, sensor_msgs::msg::Image *pstImage);
    int genRosRgbImage(const AS_Frame_s *pstFrame, sensor_msgs::msg::Image *pstImage);
    int genRosIrImage(const AS_Frame_s *pstFrame, sensor_msgs::msg::Image *pstImage);
    int genRosPeakImage(const AS_Frame_s *pstFrame, sensor_msgs::msg::Image *pstImage);

    void depthInfoPublisher(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData,
                            AS_CAM_Parameter_s &stParameter, builtin_interfaces::msg::Time time);

    void pointcloudPublisher(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pData,
                             AS_CAM_Parameter_s &stParameter);
    void rgbInfoPublisher(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData,
                          AS_CAM_Parameter_s &stParameter, builtin_interfaces::msg::Time time);
    void irInfoPublisher(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData,
                         AS_CAM_Parameter_s &stParameter, builtin_interfaces::msg::Time time);
    void peakInfoPublisher(AS_CAM_PTR pCamera, const AS_SDK_Data_s *pstData,
                           AS_CAM_Parameter_s &stParameter, builtin_interfaces::msg::Time time);
    void depthMergeInfoPublisher(AS_CAM_PTR pCamera, const AS_SDK_MERGE_s *pstData,
                                 AS_CAM_Parameter_s &stParameter, builtin_interfaces::msg::Time time);
    void pointcloudMergePublisher(AS_CAM_PTR pCamera, const AS_SDK_MERGE_s *pData,
                                  AS_CAM_Parameter_s &stParameter);
    void tfPublisher(AS_CAM_PTR pCamera, AS_CAM_Parameter_s &stParameter);

    int setResolution(AS_CAM_PTR pCamera, LAUNCH_CONFI_PARAM_S resolution);

    void saveImage(const std::string &serialno, const AS_SDK_Data_s *pstData);
    int streamController();
    bool virtualMachine();
    int initLaunchParams();
    void logCameraPathInfo(AS_CAM_ATTR_S &attr_t);
    int printLaunchParams(LAUNCH_CONFI_PARAM_S para);
    int checkStreamContrll(unsigned int allType, unsigned int checkType, AS_CAM_PTR pCamera, int &streamFlag,
                           AS_APP_CAMERA_STATUS_E status);
    void getSubListStreamType(const PUBLISHER_INFO_S &publisherInfo, unsigned int &type);

private:
    std::string m_nodeNameSpace;
    std::map<AS_CAM_PTR, AS_SDK_CAM_MODEL_E> m_cam_type_map;
    std::list<PUBLISHER_INFO_S> imgPubList;
    CameraSrv *server = nullptr;
    bool m_logfps = false;
    std::unordered_map<AS_CAM_PTR, std::shared_ptr<Camera>> m_camera_map;
    LAUNCH_CONFI_PARAM_S m_launch_param;
    bool saveImgFlg = false;
    std::thread m_monitor_thd;
    bool m_monitor_flg;
    rclcpp::Logger m_logger;
};
