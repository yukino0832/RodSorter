#include "xarm_robot/robot_node.hpp" 
#include "check_utils.hpp"
#include <chrono>
#include <cmath>
#include <interfaces/msg/detail/tool_center_pose__struct.hpp>
#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <thread>
XarmNode::XarmNode(const rclcpp::NodeOptions& options):
    Node("Xarm_node", options) {
    ArmInit();

    ParaInit();
}

void XarmNode::ArmInit() {
    // Initialize the arm
    std::string port = "192.168.1.229";
    this->xarm_ = new XArmAPI(port,false,false);
    sleep_milliseconds(500);
    int ret;
    ret = this->xarm_->connect();
    if (ret != 0) {
        RCLCPP_ERROR(this->get_logger(), "连接机械臂失败");
        return;
    }
    if (this->xarm_->error_code != 0)
        this->xarm_->clean_error();
    if (this->xarm_->warn_code != 0)
        this->xarm_->clean_warn();
    
    ret = this->xarm_->motion_enable(true);
    if (ret!=0){
        RCLCPP_ERROR(this->get_logger(), "初始化中运动设置失败");
    }
    ret = this->xarm_->set_mode(2);
    if (ret!=0){
        RCLCPP_ERROR(this->get_logger(), "初始化中模式设置失败");
    }

    this->xarm_->set_state(0);
    this->SuckerEnable(0);
    this->xarm_->set_tcp_jerk(8000);
    

    RCLCPP_INFO(this->get_logger(), "初始化成功");
    // Add your initialization code here
}

void XarmNode::ParaInit() { 
    RCLCPP_INFO(this->get_logger(), "正在初始化参数....");
    this->left_bot = [this]() {
        auto param = this->declare_parameter("left_bot", std::vector<double> { 0.0, 0.0, 0.0 });
        if (!Check_is_Init(param)) {
            RCLCPP_ERROR(this->get_logger(), "参数初始化失败");
        }
        return cv::Point3d(param[0], param[1], param[2]);
    }();

    this->left_top = [this]() {
        auto param = this->declare_parameter("left_top", std::vector<double> { 0.0, 0.0, 0.0 });
        if (!Check_is_Init(param)) {
            RCLCPP_ERROR(this->get_logger(), "参数初始化失败");
        }
        return cv::Point3d(param[0], param[1], param[2]);
    }();

    this->right_top = [this]() {
        auto param = this->declare_parameter("right_top", std::vector<double> { 0.0, 0.0, 0.0 });
        if (!Check_is_Init(param)) {
            RCLCPP_ERROR(this->get_logger(), "参数初始化失败");
        }
        return cv::Point3d(param[0], param[1], param[2]);
    }();

    this->right_bot = [this]() {
        auto param = this->declare_parameter("right_bot", std::vector<double> { 0.0, 0.0, 0.0 });
        if (!Check_is_Init(param)) {
            RCLCPP_ERROR(this->get_logger(), "参数初始化失败");
        }
        return cv::Point3d(param[0], param[1], param[2]);
    }();



    auto param_rst_pose = this->declare_parameter("reset_pose", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    if (!Check_is_Init(param_rst_pose)) {
        RCLCPP_ERROR(this->get_logger(), "参数初始化失败");
    }
    for (size_t i = 0; i < 6 && i < param_rst_pose.size(); ++i) {
        this->reset_pose[i] = static_cast<float>(param_rst_pose[i]);
    }

    auto param_second_rst_pose = this->declare_parameter("second_reset_pose", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    if (!Check_is_Init(param_second_rst_pose)) {
        RCLCPP_ERROR(this->get_logger(), "参数初始化失败");
    }
    for (size_t i = 0; i < 6 && i < param_second_rst_pose.size(); ++i) {
        this->second_reset_pose[i] = static_cast<float>(param_second_rst_pose[i]);
    }


    auto param_rst_j_pose = this->declare_parameter("reset_joint_pose", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    if (!Check_is_Init(param_rst_j_pose)) {
        RCLCPP_ERROR(this->get_logger(), "参数初始化失败");
    }
    for (size_t i = 0; i < 7 && i < param_rst_j_pose.size(); ++i) {
        this->reset_joint_pose[i] = static_cast<float>(param_rst_j_pose[i]);
    }

    auto param_second_rst_j_pose = this->declare_parameter("second_reset_joint_pose", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    if (!Check_is_Init(param_second_rst_j_pose)) {
        RCLCPP_ERROR(this->get_logger(), "参数初始化失败");
    }
    for (size_t i = 0; i < 7 && i < param_second_rst_j_pose.size(); ++i) {
        this->second_reset_joint_pose[i] = static_cast<float>(param_second_rst_j_pose[i]);
    }


    // 设置放置连杆关节角度
    auto param_put_joint_pose = this->declare_parameter("put_joint_pose", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    if (!Check_is_Init(param_put_joint_pose)) {
        RCLCPP_ERROR(this->get_logger(), "参数初始化失败");
    }
    for (size_t i = 0; i < 7 && i < param_put_joint_pose.size(); ++i) {
        this->put_joint_pose[i] = static_cast<float>(param_put_joint_pose[i]);
    }

    // 设置第二个放置连杆关节角度
    auto param_second_put_joint_pose = this->declare_parameter("second_put_joint_pose", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    if (!Check_is_Init(param_second_put_joint_pose)) {
        RCLCPP_ERROR(this->get_logger(), "参数初始化失败");
    }
    for (size_t i = 0; i < 7 && i < param_second_put_joint_pose.size(); ++i) {
        this->second_put_joint_pose[i] = static_cast<float>(param_second_put_joint_pose[i]);
    }

    this->using_joint = this->declare_parameter("using_joint",false);

    this->tool2sucker_z = [this]() {
        auto param = this->declare_parameter("tool2sucker_tran", std::vector<double> { 0.0, 0.0, 118.0, 0.0, 0.0, 0.0 });
        if (!Check_is_Init(param)) {
            RCLCPP_ERROR(this->get_logger(), "参数初始化失败");
        }
        return param[2];
    }();



    auto param_rst_j_pose_moving = this->declare_parameter("reset_joint_pose_moving", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    if (!Check_is_Init(param_rst_j_pose_moving)) {
        RCLCPP_ERROR(this->get_logger(), "参数初始化失败");
    }
    for (size_t i = 0; i < 6 && i < param_rst_j_pose_moving.size(); ++i) {
        this->reset_joint_pose_moving[i] = static_cast<float>(param_rst_j_pose_moving[i]);
    }

    RCLCPP_INFO(this->get_logger(), "参数初始化完成");

    // Reset_Pose(1);
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    //tf初始化
    this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // RCLCPP_INFO(this->get_logger(), "tf监听器初始化完成");
    //参数初始化
    this->status_pub_ = this->create_publisher<interfaces::msg::ToolCenterPose>("/TCPStatus", rclcpp::SensorDataQoS().keep_last(2));
    this->quaternion_pub_ = this->create_publisher<interfaces::msg::ToolCenterPose>("/QuaternionStatus", rclcpp::SensorDataQoS().keep_last(2));
    this->sucker_status_pub_ = this->create_publisher<interfaces::msg::ToolCenterPose>("/SuckerStatus", rclcpp::SensorDataQoS().keep_last(2));

    this->get_tetris_srv_ = this->create_service<interfaces::srv::GetTetris>(
        "/GetTetris",
        std::bind(&XarmNode::GetTetrisCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    this->put_tetris_srv_ = this->create_service<interfaces::srv::PutTetris>(
        "/PutTetris",
        std::bind(&XarmNode::PutTetrisCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    this->reset_motion_srv_ = this->create_service<interfaces::srv::ResetMotion>(
        "/ResetMotion",
        std::bind(&XarmNode::ResetMotionCallback, this, std::placeholders::_1, std::placeholders::_2)
    );
    // RCLCPP_INFO(this->get_logger(), "服务注册完成");
    this->PublishThread_ = std::thread(std::bind(&XarmNode::LoopForPublishStatus, this));
    this->SuckerPublishThread_ = std::thread(std::bind(&XarmNode::LoopForPublishSuckerStatus, this));
    RCLCPP_INFO(this->get_logger(), "机械臂节点初始化完成");
}

void XarmNode::GetTetrisCallback(const std::shared_ptr<interfaces::srv::GetTetris::Request> request, std::shared_ptr<interfaces::srv::GetTetris::Response> response) {
    // robot.ResetAllError();

    RCLCPP_INFO(this->get_logger(), "开始取方块");
    RCLCPP_INFO(this->get_logger(), "吸取点x=%f, y=%f, z=%f;旋转角度=%f", request->sucker_x, request->sucker_y, request->sucker_z, request->rotate_angle);

    // 保持姿态不变
    fp32 now_Pos[6],dist_Pos[6];
    if (this->xarm_->get_position(now_Pos)) {
        RCLCPP_ERROR(this->get_logger(), "在拿取方块时获取TCP位置失败");
        response->result = 0;
        return;
    }
    dist_Pos[0] = request->sucker_x;
    dist_Pos[1] = request->sucker_y;
    dist_Pos[2] = request->sucker_z + 170;
    dist_Pos[3] = now_Pos[3];
    dist_Pos[4] = now_Pos[4];
    dist_Pos[5] = now_Pos[5];

    RCLCPP_INFO(this->get_logger(), "tcp目标位置x=%f, y=%f, z=%f", dist_Pos[0], dist_Pos[1], dist_Pos[2]);

    // 移动到目标点
    if (!this->first_get_flag)
    {
        int err = this->xarm_->set_position(dist_Pos, 0, 1500, 6000, 0, true, NO_TIMEOUT, false, 1);
        if (err) {
            RCLCPP_WARN(this->get_logger(), "放置运动失败，错误码：%d", err);
        }
        this->first_get_flag = true;
    } else
    {
        int err = this->xarm_->set_position(dist_Pos, 0, 2500, 40000, 0, true, NO_TIMEOUT, false, 1);
        if (err) {
            RCLCPP_WARN(this->get_logger(), "放置运动失败，错误码：%d", err);
        }
    }

    RCLCPP_INFO(this->get_logger(),"移动到目标点上方完成");
    MoveDown(53);

    //吸取方块
    RCLCPP_INFO(this->get_logger(), "正在吸方块");
    // std::this_thread::sleep_for(std::chrono::milliseconds(600));
    //抬升
    MoveDown(-38);
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));  
    response->result = 1;
    RCLCPP_WARN(this->get_logger(), "取方块成功");
}


void XarmNode::PutTetrisCallback(const std::shared_ptr<interfaces::srv::PutTetris::Request> request, std::shared_ptr<interfaces::srv::PutTetris::Response> response) {
    // // robot.ResetAllError();

    // RCLCPP_INFO(this->get_logger(), "开始放方块");
    // //添加旋转角度
    // if (request->rotate_angle > M_PI || request->rotate_angle < -M_PI) {
    //     RCLCPP_ERROR(this->get_logger(), "传入旋转角度并非弧度制");
    //     response->result = 0;
    //     return;
    // }

    // fp32 dist_Pos[6]={0}, now_Pos[6]={0};

    // int ret = xarm_->get_position(now_Pos);
    // if (ret) {
    //     RCLCPP_ERROR(this->get_logger(), "获取TCP位置失败");
    //     response->result = 0;
    //     return;
    // }
    // RCLCPP_INFO(this->get_logger(), "获取TCP位置成功");

    // cv::Mat transform2 = cv::Mat::zeros(cv::Size(3, 2), CV_64FC1);
    // transform2.ptr<double>(0)[0] = -0.2062832358104908;
    // transform2.ptr<double>(0)[1] = -19.94396139630496;
    // transform2.ptr<double>(0)[2] = 609.8278690413255;
    // transform2.ptr<double>(1)[0] = -20.11229479899172;
    // transform2.ptr<double>(1)[1] = 0.09488184996477254;
    // transform2.ptr<double>(1)[2] = 89.35235926992209;

    // cv::Mat posend = transform2 * cv::Mat(cv::Vec3d(request->x, request->y, 1), CV_64F);

    // dist_Pos[0] = posend.at<double>(0, 0);
    // dist_Pos[1] = posend.at<double>(0, 1);
    // dist_Pos[2] = 183;
    // dist_Pos[3] = now_Pos[3];
    // dist_Pos[4] = now_Pos[4];
    // dist_Pos[5] = now_Pos[5];

    // // if (dist_Pos[0] > 530.8) {
    // // dist_Pos[1] -= 1;
    // // }
    // // if (dist_Pos[0] > 446.6) {
    // //     dist_Pos[1] -= 2.5;
    // //     dist_Pos[0] += 1;
    // // }

    // // if ((522.2 > dist_Pos[0] > 492.6) && (-100.3 < dist_Pos[1] < -82.8)) {
    // //     dist_Pos[1] += 2;
    // // }

    // // if ((473.5 > dist_Pos[0] > 459.3) && (-76.1 < dist_Pos[1] < -59.1)) {
    // //     dist_Pos[0] -= 2;
    // // }
    // if (request->rotate_angle >= 0) {
    //     // 以0度为起点，顺时针旋转
    //     serve.Forward(request->rotate_angle);
    // } else {
    //     // 以180度为起点，逆时针旋转
    //     serve.Backward(-request->rotate_angle);
    // }
    // dist_Pos[0] += request->bias_x;
    // dist_Pos[1] += request->bias_y;
    // RCLCPP_WARN(this->get_logger(), "TCP目标值x=%f, y=%f, z=%f", dist_Pos[0], dist_Pos[1], dist_Pos[2]);

    // // MoveSpline(dist_Pos, 450,20);
    // int err = this->xarm_->set_position(dist_Pos, 0, 2500, 40000, 0, true, NO_TIMEOUT, false, 1);
    // if (err) {
    //     RCLCPP_WARN(this->get_logger(), "放置运动失败，错误码：%d", err);
    // }

    // // std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // MoveDown(43.5);
    // std::this_thread::sleep_for(std::chrono::milliseconds(300));
    // // std::this_thread::sleep_for(std::chrono::milliseconds(200));
    // // std::this_thread::sleep_for(std::chrono::seconds(2));
    // SuckerEnable(0);
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // // MoveDown(-10 );
    // // Reset_in_Moving(0);

    // response->result = 1;
    // RCLCPP_WARN(this->get_logger(), "放方块成功");
    // // return;
    RCLCPP_INFO(this->get_logger(), "开始放方块");
    if (request->put_id == 0) {
        int err = this->xarm_->set_servo_angle(this->put_joint_pose, 300, 200, 0, true, NO_TIMEOUT, -1, false);
        if (err) {
            RCLCPP_WARN(this->get_logger(), "放置运动失败，错误码：%d", err);
            return;
        }
    }
    else if (request->put_id == 1) {
        int err = this->xarm_->set_servo_angle(this->second_put_joint_pose, 300, 200, 0, true, NO_TIMEOUT, -1, false);
        if (err) {
            RCLCPP_WARN(this->get_logger(), "放置运动失败，错误码：%d", err);
            return;
        }
    }
    else {
        RCLCPP_WARN(this->get_logger(), "传入放置id错误");
        response->result = 0;
        return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    MoveDown(10);
}

void XarmNode::ResetMotionCallback(const std::shared_ptr<interfaces::srv::ResetMotion::Request> request, std::shared_ptr<interfaces::srv::ResetMotion::Response> response) {
    RCLCPP_WARN(this->get_logger(), "重置位姿");
    (void)request;
    //TODO:  测试ing
    this->reset_flag = true;
    
    if (this->reset_flag){
        if (!Reset_Pose(request->pose_id)){
            RCLCPP_WARN(this->get_logger(), "重置位姿失败");
            response->result = 0;
            return;
        }
        else{
            RCLCPP_WARN(this->get_logger(), "重置位姿完成");
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                response->result = 1;
            return;
        }
        this->reset_flag = false;
    }
    else{
        if (!Reset_in_Moving(0)){
            RCLCPP_WARN(this->get_logger(), "运动中重置位姿失败");
            response->result = 0;
            return;
        }
        else{
            RCLCPP_WARN(this->get_logger(), "运动中重置位姿完成");
            response->result = 1;
            return;
        }
    }
    
    
}
bool XarmNode::MoveSpline(float(&dist_pose)[6] ,int time , int arc_height ){
    fp32 now_pos[6] = {0};
    int err_gett = this->xarm_->get_position(now_pos);
    if (err_gett){
        RCLCPP_ERROR(this->get_logger(), "在样条运动时获取TCP坐标失败,错误码: %d", err_gett);
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "正在生成样条轨迹");
    auto path = XarmNode::GenerateArcPath(now_pos, dist_pose, time, arc_height, 300);
    int err_set = this->xarm_->set_mode(1);

    if (err_set){
        RCLCPP_ERROR(this->get_logger(),"伺服模式设置失败");
        return false;
    }
    err_set = this->xarm_->set_state(0);
    if (err_set){
        RCLCPP_ERROR(this->get_logger(),"运动模式设置失败");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "开始样条运动");


    while (xarm_->is_connected() && xarm_->state != 4)
    {
        for (const auto& pt : path) {
            fp32 pose[6];
            for (int j = 0; j < 6; ++j) {
                pose[j] = pt[j];
            }

            fp32 now_j_Pos[7];
            int ret_get = xarm_->get_servo_angle(now_j_Pos);
            if (ret_get){
                RCLCPP_WARN(this->get_logger(),"获取当前关节位置失败");
            }
            else{
                RCLCPP_INFO(this->get_logger(),"当前关节rz = %f",now_j_Pos[5]);
            }
            fp32 now_pos[6];
            ret_get = xarm_->get_position(now_pos);
            if (ret_get){
                RCLCPP_WARN(this->get_logger(),"获取当前tcp位置失败");
            }
            else{
                // RCLCPP_INFO(this->get_logger(),"当前tcp,x= %f , y= %f , z=%f , rx = %f , ry = %f , rz = %f",now_pos[0],now_pos[1],now_pos[2],now_pos[3],now_pos[4],now_pos[5]);
                RCLCPP_INFO(this->get_logger(),"当前tcp, rz = %f",now_pos[5]);
            }
            RCLCPP_INFO(this->get_logger(),"当前发送rz值: %f",pose[5]);



            int ret = xarm_->set_servo_cartesian(pose, 300, 200);
            if (ret){
                RCLCPP_WARN(this->get_logger(), "set_servo_cartesian error, ret=%d", ret);
            }
            // RCLCPP_INFO(this->get_logger(), "set_servo_cartesian, ret=%d", ret);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        RCLCPP_INFO(this->get_logger(),"自定义轨迹运动完成");
        break;
    }
    RCLCPP_INFO(this->get_logger(),"踏出循环");
    err_set = this->xarm_->set_mode(0);
    if (err_set){
        RCLCPP_ERROR(this->get_logger(),"位控模式设置失败");
        return false;
    }
    err_set = this->xarm_->set_state(0);
    if (err_set){
        RCLCPP_ERROR(this->get_logger(),"运动模式设置失败");
        return false;
    }

    return true;

}



bool XarmNode::MoveDown(int dist ){
    fp32 now_Pos[6] = { 0 };
    int ret = xarm_->get_position(now_Pos);
    if (ret){
        RCLCPP_ERROR(this->get_logger(),"获取当前位置失败");
        return false;
    }
    now_Pos[2]-=dist;

    ret = this->xarm_->set_position(now_Pos, 0, 2000, 10000, 0, true, NO_TIMEOUT, false, 0);
    if (ret){
        RCLCPP_ERROR(this->get_logger(),"向下移动失败");
        return false;
    }
    return true;

}


bool XarmNode::Reset_in_Moving(bool is_TCP_static) {
    fp32 reset_pose_moving[6] = {0};
    int err_for = this->xarm_->get_forward_kinematics(this->reset_joint_pose_moving, reset_pose_moving);
    if (err_for){
        RCLCPP_ERROR(this->get_logger(), "正运动学求解失败，错误码: %d", err_for);
        return false;
    }
    if (!is_TCP_static){
        //MoveJ版本
        int err = this->xarm_->set_position(reset_pose_moving,0,200,200,0,true,NO_TIMEOUT,false,2);
        //Spline版本
        // robot.SplineStart();
        // int err = robot.SplinePTP(&this->reset_joint_pose_moving, &reset_pose_moving, 0, 0, 100, 100, 100);
        // robot.SplineEnd();
        
        if (err) {
            RCLCPP_ERROR(this->get_logger(), "运动中重置位姿MoveJ调用失败,错误码:%d", err);
            return false;
        }
        return true;
    }
    else{
        fp32 now_pos[6];
        int err_get = this->xarm_->get_position(now_pos);
        if (err_get){
            RCLCPP_ERROR(this->get_logger(), "重置位姿时获取当前位置失败,错误码:%d", err_get);
            return false;
        }
        reset_pose_moving[3] = now_pos[3];
        reset_pose_moving[4] = now_pos[4];
        reset_pose_moving[5] = now_pos[5];
        int err = this->xarm_->set_position(reset_pose_moving,0,200,200,0,true,NO_TIMEOUT,false,2);
        if (err){
            RCLCPP_ERROR(this->get_logger(), "运动中重置位姿MoveJ调用失败,错误码:%d", err);
            return false;
        }
        return true;
    }
}
bool XarmNode::Reset_Pose(int pose_id){
    RCLCPP_INFO(this->get_logger(), "重置位姿");
    if (!this->using_joint) {
        RCLCPP_INFO(this->get_logger(), "使用 movecart 函数归位");
        if (pose_id == 1){
            int err = this->xarm_->set_position(this->reset_pose,0,700,500,0,true,NO_TIMEOUT,false,2);
            if (err) {
                RCLCPP_ERROR(this->get_logger(), "重置位姿调用失败,错误码:%d", err);
                return false;
            }
        }
        else if (pose_id == 2){
            int err = this->xarm_->set_position(this->second_reset_pose,0,700,500,0,true,NO_TIMEOUT,false,2);
            if (err) {
                RCLCPP_ERROR(this->get_logger(), "重置位姿调用失败,错误码:%d", err);
                return false;
            }
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "重置位姿时输入ID违法,请检查输入ID");
            return false;
        }
        
        // if (CheckMotionDone()) {
        //     RCLCPP_WARN(this->get_logger(), "重置位姿完成");
        //     return true;
        // } else {
        //     RCLCPP_WARN(this->get_logger(), "重置位姿失败");
        //     return false;
        // }
        return true;
    } else {
        RCLCPP_INFO(this->get_logger(), "使用 movej 函数归位");
        
        if (pose_id == 1){
            int err = xarm_->set_servo_angle(this->reset_joint_pose, 1000, 500, 0, true, NO_TIMEOUT, -1, false);
            if (err) {
                RCLCPP_ERROR(this->get_logger(), "重置位姿调用失败,错误码:%d", err);
                return false;
            }
        }
        else if (pose_id == 2){
            int err = xarm_->set_servo_angle(this->second_reset_joint_pose, 1000, 300, 0, true, NO_TIMEOUT, -1, false);
            if (err) {
                RCLCPP_ERROR(this->get_logger(), "重置位姿调用失败,错误码:%d", err);
                return false;
            }
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "重置位姿时输入ID违法,请检查输入ID");
            return false;
        }
        // if (CheckMotionDone()) {
        //     RCLCPP_WARN(this->get_logger(), "重置位姿完成");
        //     return true;
        // } else {
        //     RCLCPP_WARN(this->get_logger(), "重置位姿失败");
        //     return false;
        // }
        RCLCPP_INFO(this->get_logger(), "resetmotion位姿完成");
        return true;    
    }
}

bool XarmNode::CheckMotionDone(fp32* dist_Pos , int timeout_time){
    fp32 now_Pos[6] = {0};
    int time = 0;
    while(1){
        int ret = xarm_->get_position(now_Pos);
        if (ret){
            RCLCPP_WARN(this->get_logger(),"获取当前位置失败");
        }
        if (abs(now_Pos[0] - dist_Pos[0] < 0.1)&&
        abs(now_Pos[1] - dist_Pos[1] < 1)&&
        abs(now_Pos[2] - dist_Pos[2] < 1)&&
        abs(now_Pos[3] - dist_Pos[3] < 1)&&
        abs(now_Pos[4] - dist_Pos[4] < 1)&&
        abs(now_Pos[5] - dist_Pos[5] < 1)){
            RCLCPP_INFO(this->get_logger(),"检测到运动完成");
            return true;
        }
        if (time >=timeout_time){
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        time+=100;
    }

    RCLCPP_ERROR(this->get_logger(),"运动超时");
    return false;
}


void XarmNode::LoopForPublishStatus(){
    while(rclcpp::ok()){
        fp32 pose[6] = {0};
        int err =this->xarm_->get_position(pose);
        if (err){
            RCLCPP_WARN(this->get_logger(),"获取当前TCP位置失败");
        }

        interfaces::msg::ToolCenterPose msg;
        msg.header.stamp = this->now();
        msg.tcp_pos.data.push_back(pose[0]);
        msg.tcp_pos.data.push_back(pose[1]);
        msg.tcp_pos.data.push_back(pose[2]);
        msg.tcp_pos.data.push_back(pose[3]);
        msg.tcp_pos.data.push_back(pose[4]);
        msg.tcp_pos.data.push_back(pose[5]);
        this->status_pub_->publish(msg);


        tf2::Quaternion q;
        q.setRPY(pose[3] / 180 * M_PI, pose[4] / 180 * M_PI, pose[5] / 180 * M_PI);
        interfaces::msg::ToolCenterPose msg_q;
        msg_q.header.stamp = this->now();
        msg_q.tcp_pos.data.push_back(q.w());
        msg_q.tcp_pos.data.push_back(q.x());
        msg_q.tcp_pos.data.push_back(q.y());
        msg_q.tcp_pos.data.push_back(q.z());
        this->quaternion_pub_->publish(msg_q);
        sleep(0.01);
    }
}



void XarmNode::LoopForPublishSuckerStatus(){
    double roll, pitch, yaw;
    while (rclcpp::ok()) {
        interfaces::msg::ToolCenterPose msg;
        try {
            // geometry_msgs::msg::TransformStamped tranasform =
            //     this->tf_buffer_->lookupTransform("sucker","base", rclcpp::Time(0.1));
            geometry_msgs::msg::PoseStamped SuckerPose;
            SuckerPose.header.frame_id = "sucker";
            SuckerPose.pose.position.x = 0;
            SuckerPose.pose.position.y = 0;
            SuckerPose.pose.position.z = 0;
            SuckerPose.pose.orientation.w = 1.0;
            SuckerPose.pose.orientation.x = 0;
            SuckerPose.pose.orientation.y = 0;
            SuckerPose.pose.orientation.z = 0;
            SuckerPose.header.stamp = this->now();

            geometry_msgs::msg::TransformStamped tranasform =
                this->tf_buffer_->lookupTransform("base", "sucker", rclcpp::Time(0.1));
            tf2::doTransform(SuckerPose, SuckerPose, tranasform);
            ConvertQuaternionToEuler(SuckerPose, roll, pitch, yaw);

            msg.header.stamp = this->now();
            msg.tcp_pos.data.push_back(SuckerPose.pose.position.x * 1000);
            msg.tcp_pos.data.push_back(SuckerPose.pose.position.y * 1000);
            msg.tcp_pos.data.push_back(SuckerPose.pose.position.z * 1000);
            msg.tcp_pos.data.push_back(roll / M_PI * 180);
            msg.tcp_pos.data.push_back(pitch / M_PI * 180);
            msg.tcp_pos.data.push_back(yaw / M_PI * 180);
        } catch (const tf2::TransformException& ex) {
            // RCLCPP_ERROR(this->get_logger(), "获取变换失败: %s", ex.what());
        }
        this->sucker_status_pub_->publish(msg);
    }
}

std::vector<std::array<float,6>> XarmNode::GenerateArcPath(const float(&start)[6], const float(&end)[6], int num_points, double arc_height, double avoid_radius){
    std::vector<std::array<float,6>> path;
    fp32 now_j_Pos[7] = {0};
    xarm_->get_servo_angle(now_j_Pos);
    float left_able_max_angle , right_able_max_angle;
    left_able_max_angle = now_j_Pos[5] + 360;
    right_able_max_angle = 360 - now_j_Pos[5];
    RCLCPP_WARN(this->get_logger(), "left_able_max_angle:%f, right_able_max_angle:%f", left_able_max_angle, right_able_max_angle);
    float pre_angle;
    RCLCPP_WARN(this->get_logger(), "end[5] = %f, start[5] = %f, diff = %f", end[5], start[5], end[5] - start[5]);
    if(end[5] - start[5] < -180){
        pre_angle = end[5] - start[5]+ 360;
    }
    else if (end[5] - start[5] > 180){
        pre_angle = end[5] - start[5]-360;
    }
    else {
        pre_angle = end[5] - start[5];
    }
   
    if (pre_angle > right_able_max_angle)
        pre_angle -= 360;
    else if (abs(pre_angle) > left_able_max_angle)
        pre_angle += 360;

    RCLCPP_WARN(this->get_logger(), "pre_angle = %f", pre_angle);
    
    for (int i = 0; i <= num_points; ++i) {
        double t = double(i) / num_points;
        std::array<float,6> mid;
        // 线性插值
        mid[0] = start[0] + (end[0] - start[0]) * t;
        mid[1] = start[1] + (end[1] - start[1]) * t;
        // 抛物线插值z
        mid[2] = start[2] + (end[2] - start[2]) * t + arc_height * std::sin(M_PI * t);

        // 姿态保持竖直向下
        mid[3] = 180;
        mid[4] = 0;
        // RCLCPP_INFO(this->get_logger(),"中间路径旋转量= %f",(pre_angle / num_points) * t);
        mid[5] = start[5]+ pre_angle  * t;
        if (mid[5] > 180){
            mid[5]-=360;
        }
        else if (mid[5] < -180){
            mid[5]+=360;
        }
        RCLCPP_INFO(this->get_logger(),"中间路径mid[5]= %f",mid[5]);


        // int ret = xarm_->get_servo_angle(now_j_Pos);
        // if (ret){
        //     RCLCPP_WARN(this->get_logger(),"获取当前关节位置失败");
        // }
        // else{
        //     RCLCPP_INFO(this->get_logger(),"当前关节rz = %f",now_j_Pos[5]);
        // }
        // fp32 now_pos[6];
        // ret = xarm_->get_position(now_pos);
        // if (ret){
        //     RCLCPP_WARN(this->get_logger(),"获取当前tcp位置失败");
        // }
        // else{
        //     // RCLCPP_INFO(this->get_logger(),"当前tcp,x= %f , y= %f , z=%f , rx = %f , ry = %f , rz = %f",now_pos[0],now_pos[1],now_pos[2],now_pos[3],now_pos[4],now_pos[5]);
        //     RCLCPP_INFO(this->get_logger(),"当前tcp, rz = %f",now_pos[5]);
        // }
        



        // 判断是否靠近基座，需要外歪
        double dx = end[0] - start[0];
        double dy = end[1] - start[1];
        double norm = std::sqrt(dx * dx + dy * dy);
        double line_dist = 0.0;
        if (norm > 1e-6) {
            // 直线一般式 Ax+By+C=0，点(0,0)到直线距离
            line_dist = std::abs(dx * start[1] - dy * start[0]) / norm;
        }
        if (avoid_radius > 0 && line_dist < avoid_radius) {
            // 计算垂直于连线、远离原点的单位法向量
            double nx = -dy / norm;
            double ny = dx / norm;
            // 判断法向量方向是否远离原点（点乘判断）
            double mid_x = start[0] + dx * t;
            double mid_y = start[1] + dy * t;
            double dot = mid_x * nx + mid_y * ny;
            if (dot < 0) { nx = -nx; ny = -ny; }
            double offset = (avoid_radius - line_dist) * 0.7 * std::sin(M_PI * t);
            mid[0] += nx * offset;
            mid[1] += ny * offset;
        }
        path.push_back(mid);
    }
    return path;
}


cv::Point3d XarmNode::CalculatePosition(float x, float y) {
    // cv::Point3d Origin = this->left_bot + (2.3 / 30.6) * (this->right_top - this->left_bot);
    // cv::Point3d x_axis = (this->right_top - this->left_top) * (2.0 / 30.6);
    // cv::Point3d y_axis = (this->left_top - this->left_bot) * (2.0 / 22.0);
    cv::Point3d Origin = this->left_top;
    cv::Point3d x_axis = (this->right_top - this->left_top) / 9.0;
    cv::Point3d y_axis = (this->left_bot - this->left_top) / 13.0;
    return Origin + x * x_axis + y * y_axis;
}


XarmNode::~XarmNode(){
    SuckerEnable(0);
    xarm_->disconnect();
    RCLCPP_INFO(this->get_logger(), "已断开机械臂连接");
    if (PublishThread_.joinable()) {
        PublishThread_.join();
    }
    if (SuckerPublishThread_.joinable()) {
        SuckerPublishThread_.join();
    }
}
bool XarmNode::SuckerEnable(int enable){
    if (enable == 1) {
        RCLCPP_INFO(this->get_logger(), "开始送气");
        int err1 = this->xarm_->set_cgpio_digital(0, 1);
        if (err1 != 0) {
            RCLCPP_ERROR(this->get_logger(), "送气失败");
            return false;
        }
        return true;
    } else if (enable == 0) {
        RCLCPP_INFO(this->get_logger(), "停止送气");
        int err2 = this->xarm_->set_cgpio_digital(0, 0);
        if (err2 != 0) {
            RCLCPP_ERROR(this->get_logger(), "停气失败");
            return false;
        }
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "输入参数错误");
        return false;
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(XarmNode)