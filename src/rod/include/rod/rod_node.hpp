#pragma once

// STD
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// ROS
#include "interfaces/srv/get_tetris.hpp"
#include "interfaces/srv/pixel_to_base.hpp"
#include "interfaces/srv/put_tetris.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "detector.hpp"
#include "rod.hpp"

class RodNode: public rclcpp::Node {
public:
    explicit RodNode(const rclcpp::NodeOptions& options);

private:
    // 参数阈值
    Param m_param;
    // 传入图片
    cv::Mat m_image;
    // 存放所有rod信息
    std::vector<rod> m_rods;
    // Rod检测器
    RodDetector m_detector;
    // 图像订阅者
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    // 像素与三维坐标转换客户端
    rclcpp::Client<interfaces::srv::PixelToBase>::SharedPtr pixel_to_base_client_;
    // 连杆抓取位置客户端
    rclcpp::Client<interfaces::srv::GetTetris>::SharedPtr get_rod_client_;
    // 连杆放置位置客户端
    rclcpp::Client<interfaces::srv::PutTetris>::SharedPtr put_rod_client_;
    // 像素与三维坐标转换请求
    std::shared_ptr<interfaces::srv::PixelToBase::Request> pixel_to_base_request_;
    // 连杆抓取位置请求
    std::shared_ptr<interfaces::srv::GetTetris::Request> get_rod_request_;
    // 连杆放置位置请求
    std::shared_ptr<interfaces::srv::PutTetris::Request> put_rod_request_;
    // 像素与三维坐标转换响应
    std::shared_ptr<interfaces::srv::PixelToBase::Response> pixel_to_base_response_;
    // 连杆抓取位置响应
    std::shared_ptr<interfaces::srv::GetTetris::Response> get_rod_response_;
    // 连杆放置位置响应
    std::shared_ptr<interfaces::srv::PutTetris::Response> put_rod_response_;
    /**
     * @brief 回调函数，获取图像信息进行检测，发布检测结果
     * 
     * @param img_msg 
     */
    void ImageCallback(const sensor_msgs::msg::Image::SharedPtr img_msg);

    /**
     * @brief 发送当前指令给服务端
     */
    void sendCommand();

    /**
     * @brief 像素与三维坐标转换服务端回调函数
     */
    void handlePixelToBaseResponse();

    /**
     * @brief 连杆抓取位置服务端回调函数
     */
    void handleGetRodResponse();

    /**
     * @brief 连杆放置位置服务端回调函数
     */
    void handlePutRodResponse();
};