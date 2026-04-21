/**
 * @file      ascamera_node.h
 * @brief     angstrong camera ROS demo program header.
 *
 * Copyright (c) 2023 Angstrong Tech.Co.,Ltd
 *
 * @author    Angstrong SDK develop Team
 * @date      2023/02/15
 * @version   1.0

 */
#pragma once

typedef struct LAUNCH_CONFI_PARAM {
    int set_depth_width;
    int set_depth_height;
    int set_rgb_width;
    int set_rgb_height;
    int set_ir_width;
    int set_ir_height;
    int set_peak_width;
    int set_peak_height;
    int set_fps;
    int usb_bus_num;
    char usb_port_nums[64];
    bool color_pcl;
    bool pub_tfTree;
} LAUNCH_CONFI_PARAM_S;

