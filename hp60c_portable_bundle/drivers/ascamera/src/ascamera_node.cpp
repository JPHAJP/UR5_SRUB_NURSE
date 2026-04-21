/**
 * @file      ascamera_node.cpp
 * @brief     angstrong ros2 camera publisher node.
 *
 * Copyright (c) 2023 Angstrong Tech.Co.,Ltd
 *
 * @author    Angstrong SDK develop Team
 * @date      2023/03/27
 * @version   1.0

 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"

#include "CameraPublisher.h"
#include "LogRedirectBuffer.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraPublisher>();
    // rclcpp::spin(std::make_shared<CameraPublisher>());

    auto logger = node->get_logger();
    auto buf = std::make_shared<LogRedirectBuffer>(logger);
    std::cout.rdbuf(buf.get());
    std::cerr.rdbuf(buf.get());
    RCLCPP_INFO(logger, "hello world angstrong camera ros2 node");

    node->start();
    rclcpp::WallRate loop_rate(25);

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    node->stop();

    RCLCPP_INFO(logger, "angstrong camera ros2 node shutdown");

    rclcpp::shutdown();
    return 0;
}
