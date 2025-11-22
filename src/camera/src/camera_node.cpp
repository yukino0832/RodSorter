#include "camera/camera_node.hpp"
#include "libobsensor/hpp/StreamProfile.hpp"
#include "utils_opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
// Forward declaration of the onMouse function

void onMouse(int event, int x, int y, int flags, void* userdata) {
    (void)flags;                        // Unused parameter
    (void)userdata;                     // Unused parameter
    if (event == cv::EVENT_LBUTTONDOWN) // 检查是否是鼠标左键按下的事件
    {
        std::cout << "cv::Point2f(" << x << ", " << y << ")," << std::endl;
    }
}
CameraNode::CameraNode(const rclcpp::NodeOptions& options):
    Node("camera_node", options),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_) {
    this->base_coordinate = declare_parameter("base_coordinate", "base");
    this->sucker_coordinate = declare_parameter("sucker_coordinate", "sucker");
    this->tool_coordinate = declare_parameter("tool_coordinate", "tool");
    this->camera_coordinate = declare_parameter("camera_coordinate", "camera");

    bool device_found = false; //相机是否找到

    rclcpp::QoS qos_profile(rclcpp::KeepLast(2));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    this->img_pub = this->create_publisher<sensor_msgs::msg::Image>("/image", qos_profile);

    this->camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_info", qos_profile);

    this->depth_pub = this->create_publisher<sensor_msgs::msg::Image>("/depth", rclcpp::SensorDataQoS().keep_last(2));

    this->pixel2base_srv = this->create_service<interfaces::srv::PixelToBase>("/PixelToBase", std::bind(&CameraNode::PixelToBaseCallback, this, std::placeholders::_1, std::placeholders::_2));

    while (!device_found && rclcpp::ok()) {
        try {
            //开启相机
            pipe_ = std::make_shared<ob::Pipeline>();
            RCLCPP_INFO(this->get_logger(), "Device found");
            //配置视频流参数
            this->config_ = std::make_shared<ob::Config>();
            config_->enableVideoStream(OB_STREAM_DEPTH, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY, OB_FORMAT_Y16);
            config_->enableVideoStream(OB_STREAM_COLOR, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY, OB_FORMAT_RGB);
            config_->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);

            this->color2depthAlign_ = std::make_shared<ob::Align>(OB_STREAM_COLOR);

            //开启视频流
            pipe_->start(config_, [&](std::shared_ptr<ob::FrameSet> output) {
                std::lock_guard<std::mutex> lock(framesetMutex);
                frameset = output;
                // RCLCPP_INFO(this->get_logger(), "Get frame");
            });

            RCLCPP_INFO(this->get_logger(), "Start success");
            device_found = true;
        } catch (ob::Error& e) {
            // throw std::runtime_error("Error starting camera: " + std::string(e.what()));
            RCLCPP_ERROR(this->get_logger(), "No device found, retrying...");
            // RCLCPP_ERROR(this->get_logger(), "Error happening when connecting camera: %s", e.what());
            pipe_.reset();
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
    }
    //开线程用于发布相机图片

    sleep(2);

    this->PublishThread_ = std::thread(std::bind(&CameraNode::LoopForPublish, this));

    this->CameraInfoThread_ = std::thread(std::bind(&CameraNode::LoopForCameraInfo, this));

    // color_for_srv = ConvertColor2Mat(this->frameset->colorFrame());

    // // 创建窗口
    // const std::string windowName = "Image";
    // namedWindow(windowName, cv::WINDOW_AUTOSIZE);
    // // 设置鼠标回调函数
    // cv::setMouseCallback(windowName, onMouse, nullptr);

    // while (rclcpp::ok()) // 在ROS节点的主循环中处理GUI事件，而不是阻塞的while(true)
    // {
    //     if (!color_for_srv.empty()) {
    //         imshow(windowName, color_for_srv);
    //     } else {
    //         RCLCPP_WARN_ONCE(this->get_logger(), "color_for_srv is empty, waiting for frame...");
    //     }

    //     int key = cv::waitKey(1); // 使用 rclcpp::ok() 作为循环条件时，waitKey(1) 避免阻塞太久
    //     if (key == 27)            // ESC 键的 ASCII 是 27
    //         break;
    // }

    // cv::destroyAllWindows();
}

//提供二维像素转基坐标系的服务回调 TODO:还没写完捏
void CameraNode::PixelToBaseCallback(const std::shared_ptr<interfaces::srv::PixelToBase::Request> request, std::shared_ptr<interfaces::srv::PixelToBase::Response> response) {
    RCLCPP_WARN(this->get_logger(), "Get request x=%d, y=%d", request->pixel_x, request->pixel_y);
    RCLCPP_INFO(this->get_logger(), "开始干活(解算)");
    //TODO:

    try {
        cv::circle(color_for_srv, cv::Point2d(request->pixel_x, request->pixel_y), 10, cv::Scalar(0, 0, 255), 2);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error converting color frame: %s", e.what());
        response->result = false;
        return;
    }

    cv::Mat transform = cv::Mat::zeros(cv::Size(3, 2), CV_64FC1);

    ////////////////////////////////////////////////////////////////////////
    // 待改
    ////////////////////////////////////////////////////////////////////////
    transform.ptr<double>(0)[0] = 0.4984147829632819;
    transform.ptr<double>(0)[1] = 0.002744070503315559;
    transform.ptr<double>(0)[2] = -3.488111826623243;
    transform.ptr<double>(1)[0] = 0.004643768148017612;
    transform.ptr<double>(1)[1] = -0.5030440715094449;
    transform.ptr<double>(1)[2] = 515.8664424125985;

    cv::Mat pos = transform * cv::Mat(cv::Vec3d(request->pixel_x, request->pixel_y, 1), CV_64F);

    response->base_x = pos.at<double>(0, 0);
    response->base_y = pos.at<double>(0, 1);
    response->base_z = 0;
    response->result = true;

    RCLCPP_WARN(this->get_logger(), "return success");
    RCLCPP_WARN(this->get_logger(), "x=%f, y=%f, z=%f", response->base_x, response->base_y, response->base_z);
}

double CameraNode::GetPixelDepth(int x, int y) {
    // std::lock_guard<std::mutex> lock(framesetMutex);
    RCLCPP_INFO(this->get_logger(), "GetPixelDepth x=%d, y=%d", x, y);
    if (frameset == nullptr) {
        RCLCPP_WARN(this->get_logger(), "Frameset get failed");
        return 0.0;
    }
    std::shared_ptr<ob::DepthFrame> frame_depth = this->align_frameset->depthFrame();
    int width = frame_depth->as<ob::VideoFrame>()->getWidth();
    int height = frame_depth->as<ob::VideoFrame>()->getHeight();
    RCLCPP_INFO(this->get_logger(), "width=%d, height=%d", width, height);
    cv::Mat dis_Mat = cv::Mat(frame_depth->height(), frame_depth->width(), CV_16UC1, frame_depth->data());
    ushort distance = dis_Mat.at<ushort>(y, x) * frame_depth->getValueScale();
    auto res = distance;

    RCLCPP_INFO(this->get_logger(), "res=%d", res);
    RCLCPP_INFO(this->get_logger(), "666");
    cv::Mat color = ConvertColor2Mat(this->align_frameset->colorFrame());
    cv::circle(color, cv::Point2d(x, y), 10, cv::Scalar(0, 0, 255), 2);
    cv::putText(color, std::to_string(res), cv::Point2d(x, y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);

    // cv::imshow("color", color);
    // cv::waitKey(1);

    return res;
}

void CameraNode::FillDepthHoles(cv::Mat& depth, int maxDepth, int minDepth, int kernel) {
    cv::Mat filled = depth.clone();
    for (int y = 0; y < depth.rows; ++y) {
        for (int x = 0; x < depth.cols; ++x) {
            ushort& d = filled.at<ushort>(y, x);
            if (d == 0) {
                std::vector<ushort> neighbors;
                for (int dy = -kernel; dy <= kernel; ++dy) {
                    for (int dx = -kernel; dx <= kernel; ++dx) {
                        int ny = y + dy, nx = x + dx;
                        if (ny >= 0 && ny < depth.rows && nx >= 0 && nx < depth.cols) {
                            ushort nd = depth.at<ushort>(ny, nx);
                            if (nd != 0 && nd >= minDepth && nd <= maxDepth) {
                                neighbors.push_back(nd);
                            }
                        }
                    }
                }
                if (!neighbors.empty()) {
                    // 用邻域的中值
                    std::nth_element(neighbors.begin(), neighbors.begin() + neighbors.size() / 2, neighbors.end());
                    d = neighbors[neighbors.size() / 2];
                }
            }
        }
    }
    depth = filled;
}

void CameraNode::LoopForCameraInfo() {
    auto colorProfile = this->frameset->colorFrame()->getStreamProfile()->as<ob::VideoStreamProfile>();
    auto Intrinsic = colorProfile->getIntrinsic();
    auto Distortion = colorProfile->getDistortion();
    auto width = colorProfile->getWidth();
    auto height = colorProfile->getHeight();
    auto camera_info = std::make_shared<sensor_msgs::msg::CameraInfo>();
    camera_info->header.frame_id = this->camera_coordinate;
    camera_info->header.stamp = this->now();
    camera_info->height = height;
    camera_info->width = width;
    camera_info->distortion_model = "plumb_bob";
    camera_info->k = { Intrinsic.fx, 0.0, Intrinsic.cx, 0.0, Intrinsic.fy, Intrinsic.cy, 0.0, 0.0, 1.0 };
    camera_info->d = { Distortion.k1, Distortion.k2, Distortion.k3, Distortion.k4, Distortion.k5, Distortion.k6, Distortion.p1, Distortion.p2 };
    camera_info->p = { Intrinsic.fx, 0.0, Intrinsic.cx, 0.0, 0.0, Intrinsic.fy, Intrinsic.cy, 0.0, 0.0, 0.0, 1.0 };

    this->camera_info_pub->publish(*camera_info);
}
void CameraNode::LoopForPublish() {
    while (rclcpp::ok()) {
        //不加延时会一直锁住了导致图像无法刷新，加个延时保证图像流畅
        sleep(0.02);
        std::lock_guard<std::mutex> lock(framesetMutex);
        if (frameset == nullptr) {
            RCLCPP_WARN(this->get_logger(), "Frameset get failed");
            continue;
        }
        //对RGB图像和Depth图像进行对齐
        auto align_frame = this->color2depthAlign_->process(frameset);
        this->align_frameset = align_frame->as<ob::FrameSet>();

        //获取RGB图像
        auto colorFrame = align_frameset->colorFrame();
        cv::Mat frame = ConvertColor2Mat(colorFrame);
        // GetPixelDepth(960, 540);

        //获取深度图像
        auto depthFrame = align_frameset->depthFrame();
        cv::Mat depth = ConvertDepth2Mat(depthFrame);
        // FillDepthHoles(depth, 50, 30, 3);

        if (!frame.empty()) {
            PublishImage(frame);
        } else {
            RCLCPP_WARN(this->get_logger(), "Color Mat is empty");
        }

        if (!depth.empty()) {
            // cv::imshow("depth", depth);
            // cv::waitKey(1);
            PublishDepth(depth);
        } else {
            RCLCPP_WARN(this->get_logger(), "Depth Mat is empty");
        }

        // RCLCPP_INFO(this->get_logger(), "before dep");
        // GetPixelDepth(960,540);
        // RCLCPP_INFO(this->get_logger(), "Publish");

        //TODO: 保存图片用，比赛可记录一些图片
        // auto now = std::chrono::system_clock::now();
        // std::time_t t = std::chrono::system_clock::to_time_t(now);
        // std::tm* now_tm = std::localtime(&t);
        // std::string name;
        // name = std::to_string(now_tm->tm_year + 1900) + "-" + std::to_string(now_tm->tm_mon + 1) + "-" + std::to_string(now_tm->tm_mday) + "-" + std::to_string(now_tm->tm_hour) + "-" + std::to_string(now_tm->tm_min) + "-" + std::to_string(now_tm->tm_sec);
        // cv::imwrite("/home/knot/FairinoSingleArm/images" + name + ".jpg", frame);
    }
}

void CameraNode::PublishImage(cv::Mat& image) {
    try {
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera";
        auto msg_img = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
        img_pub->publish(*msg_img);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Publish color error: %s", e.what());
    }
    // RCLCPP_INFO(this->get_logger(), "Color send success");
    // RCLCPP_INFO(this->get_logger(),"Color Size: %d, %d", image.cols, image.rows);
}

void CameraNode::PublishDepth(cv::Mat& image) {
    try {
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera";
        auto msg_img = cv_bridge::CvImage(header, "rgb8", image).toImageMsg();
        depth_pub->publish(*msg_img);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Publish depth error: %s", e.what());
    }
    // RCLCPP_INFO(this->get_logger(), "Depth send success");
    // RCLCPP_INFO(this->get_logger(),"Depth Size: %d, %d", image.cols, image.rows);
}

CameraNode::~CameraNode() {
    if (PublishThread_.joinable()) {
        PublishThread_.join();
    }
    if (CameraInfoThread_.joinable()) {
        CameraInfoThread_.join();
    }
    if (pipe_) {
        try {
            pipe_->stop();
            RCLCPP_INFO(this->get_logger(), "Camera stopped");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error stopping camera: %s", e.what());
        }
        // 智能指针会自动处理析构
    }

    RCLCPP_INFO(this->get_logger(), "CameraNode destroy");
}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(CameraNode)