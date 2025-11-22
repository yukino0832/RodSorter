#include "rod/rod_node.hpp"

using std::placeholders::_1;

RodNode::RodNode(const rclcpp::NodeOptions& options):
    Node("rod_node", options) {
    RCLCPP_INFO(this->get_logger(), "Starting RodNode!");
    // 创建客户端
    this->pixel_to_base_client_ = this->create_client<interfaces::srv::PixelToBase>("/PixelToBase");
    this->get_rod_client_ = this->create_client<interfaces::srv::GetTetris>("/GetTetris");
    this->put_rod_client_ = this->create_client<interfaces::srv::PutTetris>("/PutTetris");
    // 创建请求
    this->pixel_to_base_request_ = std::make_shared<interfaces::srv::PixelToBase::Request>();
    this->get_rod_request_ = std::make_shared<interfaces::srv::GetTetris::Request>();
    this->put_rod_request_ = std::make_shared<interfaces::srv::PutTetris::Request>();
    // 创建一个图像订阅者，订阅 "/image" 话题，并绑定回调函数
    this->img_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/image", rclcpp::SensorDataQoS().keep_last(2), std::bind(&RodNode::ImageCallback, this, std::placeholders::_1));
}

void RodNode::ImageCallback(const sensor_msgs::msg::Image::SharedPtr img_msg) {
    RCLCPP_INFO(this->get_logger(), "Received an image!");
    m_image = cv::Mat(img_msg->height, img_msg->width, CV_8UC3, img_msg->data.data()).clone();
    m_rods = m_detector.detect(m_image);
    // 获取连杆位置及规划完成，开始与robot_node通信
    // 发送请求
    while (!pixel_to_base_client_->wait_for_service(std::chrono::seconds(1)) || !get_rod_client_->wait_for_service(std::chrono::seconds(1)) || !put_rod_client_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for services.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for services...");
    }
    RCLCPP_INFO(this->get_logger(), "All services are ready.");
    sendCommand();
}

void RodNode::sendCommand() {
    pixel_to_base_request_->pixel_x = static_cast<int>(m_rods.front().center.x);
    pixel_to_base_request_->pixel_y = static_cast<int>(m_rods.front().center.y);
    // 发送异步请求
    auto future = pixel_to_base_client_->async_send_request(
        pixel_to_base_request_,
        [this](rclcpp::Client<interfaces::srv::PixelToBase>::SharedFuture future_response) {
            this->pixel_to_base_response_ = future_response.get();
            handlePixelToBaseResponse();
        }
    );
}

void RodNode::handlePixelToBaseResponse() {
    get_rod_request_->sucker_x = pixel_to_base_response_->base_x;
    get_rod_request_->sucker_y = pixel_to_base_response_->base_y;
    get_rod_request_->sucker_z = pixel_to_base_response_->base_z;

    auto future = get_rod_client_->async_send_request(
        get_rod_request_,
        [this](rclcpp::Client<interfaces::srv::GetTetris>::SharedFuture future_response) {
            this->get_rod_response_ = future_response.get();
            handleGetRodResponse();
        }
    );
}

void RodNode::handleGetRodResponse() {
    put_rod_request_->put_id = m_rods.front().isDefective ? 1 : 0;

    auto future = put_rod_client_->async_send_request(
        put_rod_request_,
        [this](rclcpp::Client<interfaces::srv::PutTetris>::SharedFuture future_response) {
            this->put_rod_response_ = future_response.get();
            handlePutRodResponse();
        }
    );
}

void RodNode::handlePutRodResponse() {
    if (put_rod_response_->result == 1)
        RCLCPP_INFO(this->get_logger(), "Put rod succeeded");
    else
        RCLCPP_WARN(this->get_logger(), "Put rod failed");

    m_rods.erase(m_rods.begin());
    if (m_rods.empty()) {
        RCLCPP_INFO(this->get_logger(), "All rods processed. Working done!(๑˃ᴗ˂)ﻭ");
        return;
    }
    sendCommand();
}

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(RodNode)