#include "xarm_robot/servo_driver.h"
#include "xarm/wrapper/xarm_api.h"
#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <interfaces/msg/detail/tool_center_pose__struct.hpp>
#include <interfaces/msg/tool_center_pose.hpp>
#include <interfaces/srv/get_tetris.hpp>
#include <interfaces/srv/put_tetris.hpp>
#include <interfaces/srv/reset_motion.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <thread>
#ifdef WINDOWS_OPTION
    #include <string.h>
    #include <windows.h>
#elif LINUX_OPTION
    #include <cstdlib>
    #include <cstring>
    #include <iostream>
    #include <stdio.h>
    #include <unistd.h>
#endif


class XarmNode: public rclcpp::Node {
public:
    /** 
        @brief: 构造函数，梦的开始
        **/
    explicit XarmNode(const rclcpp::NodeOptions& options);

    /** 
        @brief: 初始化机械臂，连接到机械臂并检查连接状态
        **/
    void ArmInit();
    
    /**
     * @brief 参数初始化
     * 
     */
    void ParaInit();


    /** 
        @brief: 析构函数，用于关闭机械臂连接
        **/
    ~XarmNode();

    /** 
        @brief: 持续发布机械臂TCP状态
    **/
    void LoopForPublishStatus();

    /** 
            @brief: 持续发布吸盘状态
    **/
    void LoopForPublishSuckerStatus();


    /** 
        @brief: 通过棋盘坐标系(存在四个角点的先验知识)计算该点的具体基坐标系下的坐标
        @param x: 棋盘坐标系下的x坐标
        @param y: 棋盘坐标系下的y坐标
        @return: 返回计算后的三维坐标
        **/
    cv::Point3d CalculatePosition(float x, float y);


    /** 
        @brief: 控制机械臂垂直向下移动，单位为mm
        @return: 返回执行结果
        **/
    bool MoveDown(int dist , int time);

    bool MoveDown(int dist);

    // /** 
    //     @brief: 控制机械臂以基坐标系z轴为旋转轴进行旋转指定角度
    //     @param angle 旋转角度
    //     @param dir 方向 1: 正方向 2：负方向
    //     @return: 返回执行结果
    //     **/
    // bool MoveCircle(double angle, int dir);

    
    /**
     * @brief  使用样条运动自动规划一条轨迹点平滑到达目标点
     * 
     * @param dist_pose 目标位姿
     * @param time 平滑次数
     * @return 
     */
    bool MoveSpline(float(&dist_pose)[6] ,int time , int arc_height = 70 );
    
    
    /**
        @brief: 控制吸盘是否打开
        @param enable: 是否打开吸盘 1：打开 0：关闭
        @return: 返回执行结果
        **/
    bool SuckerEnable(int enable);


    /**
        @brief: 通过简单的速度来检测动作是否完成
        @return: 返回检测结果
        **/
    bool CheckMotionDone(fp32* dist_Pos , int timeout_time);
    
    /** 
        @brief: 输入吸盘目标位置和旋转角度，返回TCP位姿的妙妙小工具
        @return: 是否成功
        **/

    /**
     * @brief 实现输入起点和终点，自动插值生成轨迹点
     * 
     * @param start 起始点
     * @param end 终点
     * @param num_points  插值点个数
     * @param arc_height 
     * @param avoid_radius 
     * @return std::vector<DescPose> 
     */
    std::vector<std::array<float, 6>> GenerateArcPath(const float(&start)[6], const float(&end)[6], int num_points, double arc_height, double avoid_radius = 0.0);


    bool Reset_in_Moving(bool is_TCP_static);

    bool Reset_Pose(int pose_id);


inline  geometry_msgs::msg::PoseStamped CalculateToolPoseAfterRotation(double x , double y , double z ,double rotate_angle_rad) {
        try {
            RCLCPP_WARN(this->get_logger(),"获得输入的吸盘目标位置和旋转角度: x=%f, y=%f, z=%f, rotate_angle_rad=%f", x, y, z, rotate_angle_rad);
            geometry_msgs::msg::TransformStamped sucker_to_base_now = 
                tf_buffer_->lookupTransform("base", "sucker", rclcpp::Time(0));
            geometry_msgs::msg::TransformStamped tool_to_sucker_now = 
                tf_buffer_->lookupTransform("sucker", "tool", rclcpp::Time(0));
            //拿到吸盘在基坐标系下的位姿
            geometry_msgs::msg::PoseStamped sucker_pose_in_sucker;
            geometry_msgs::msg::PoseStamped sucker_pose_in_base;
            sucker_pose_in_sucker.header.frame_id = "sucker";
            sucker_pose_in_sucker.header.stamp = this->now();
            sucker_pose_in_sucker.pose.position.x = 0.0;
            sucker_pose_in_sucker.pose.position.y = 0.0;
            sucker_pose_in_sucker.pose.position.z = 0.0;
            sucker_pose_in_sucker.pose.orientation.w = 1.0;
            sucker_pose_in_sucker.pose.orientation.x = 0.0;
            sucker_pose_in_sucker.pose.orientation.y = 0.0;
            sucker_pose_in_sucker.pose.orientation.z = 0.0;

            tf2::doTransform(sucker_pose_in_sucker, sucker_pose_in_base, sucker_to_base_now);
            RCLCPP_WARN(this->get_logger(), "当前吸盘在base坐标系下的位姿: x=%f, y=%f, z=%f",
                sucker_pose_in_base.pose.position.x, sucker_pose_in_base.pose.position.y, sucker_pose_in_base.pose.position.z);


            // 1. 创建绕Z轴的旋转变换
            tf2::Quaternion rotation;
            rotation.setRPY(0, 0, rotate_angle_rad); // 仅绕Z轴旋转
            
            // 2. 获取当前 base_to_sucker 的四元数
            tf2::Quaternion current_rotation;
            tf2::fromMsg(sucker_pose_in_base.pose.orientation, current_rotation);
            
            // 3. 计算旋转后的四元数（当前四元数*旋转四元数）
            tf2::Quaternion new_rotation = current_rotation * rotation;
            new_rotation.normalize();
            
            // 6. 获取旋转后的sucker在基坐标系下的位姿
            geometry_msgs::msg::PoseStamped rotated_base_to_sucker;
            rotated_base_to_sucker.pose.position.x = x/1000.0;
            rotated_base_to_sucker.pose.position.y = y/1000.0;
            rotated_base_to_sucker.pose.position.z = z/1000.0;
            rotated_base_to_sucker.header.frame_id = "sucker";
            rotated_base_to_sucker.header.stamp = this->now();
            geometry_msgs::msg::Quaternion new_rotation_msg;
            tf2::convert(new_rotation, new_rotation_msg);
            rotated_base_to_sucker.pose.orientation = new_rotation_msg;
            
            //转成 TransformStamped
            geometry_msgs::msg::TransformStamped sucker_to_base_after = 
            poseToTransform(rotated_base_to_sucker, "base");
            RCLCPP_WARN(this->get_logger(), "旋转后吸盘在base坐标系下的目标位姿: x=%f, y=%f, z=%f",
                rotated_base_to_sucker.pose.position.x, rotated_base_to_sucker.pose.position.y, rotated_base_to_sucker.pose.position.z);
            // 7. 创建TCP原点的pose
            geometry_msgs::msg::PoseStamped tcp_origin;
            tcp_origin.header.frame_id = "tool";  
            tcp_origin.header.stamp = this->now();
            tcp_origin.pose.position.x = 0.0;
            tcp_origin.pose.position.y = 0.0;
            tcp_origin.pose.position.z = 0.0;
            tcp_origin.pose.orientation.w = 1.0;
            tcp_origin.pose.orientation.x = 0.0;
            tcp_origin.pose.orientation.y = 0.0;
            tcp_origin.pose.orientation.z = 0.0;

            //获取TCP原点在吸盘坐标系下的位姿
            geometry_msgs::msg::PoseStamped tcp_in_sucker;
            tf2::doTransform(tcp_origin, tcp_in_sucker, tool_to_sucker_now);
            RCLCPP_WARN(this->get_logger(), "TCP在吸盘坐标系下的位姿: x=%f, y=%f, z=%f",
                tcp_in_sucker.pose.position.x, tcp_in_sucker.pose.position.y, tcp_in_sucker.pose.position.z);

            geometry_msgs::msg::PoseStamped tcp_in_base;
            tf2::doTransform(tcp_in_sucker, tcp_in_base, sucker_to_base_after);

            RCLCPP_WARN(this->get_logger(), "TCP在base坐标系下的位姿: x=%f, y=%f, z=%f",
                tcp_in_base.pose.position.x, tcp_in_base.pose.position.y, tcp_in_base.pose.position.z);
            return tcp_in_base;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR(this->get_logger(), "计算变换失败: %s", ex.what());
            geometry_msgs::msg::PoseStamped empty_pose;
            empty_pose.header.stamp = this->now();
            empty_pose.header.frame_id = "base";
            return empty_pose;
        }
    }


    /**
        @brief: 将一个Posestamped转换为TransformStamped的妙妙小工具
        @return: 
        **/
inline  geometry_msgs::msg::TransformStamped poseToTransform(
        const geometry_msgs::msg::PoseStamped& pose, 
        const std::string& child_frame_id) {
        
        geometry_msgs::msg::TransformStamped transform;
        
        transform.header = pose.header;
        transform.child_frame_id = child_frame_id;
        
        transform.transform.translation.x = pose.pose.position.x;
        transform.transform.translation.y = pose.pose.position.y;
        transform.transform.translation.z = pose.pose.position.z;
        
        transform.transform.rotation = pose.pose.orientation;
        
        return transform;
    }


    /**
        @brief: 将一个将四元数转换为rpy的妙妙小工具
        @return: 
        **/
inline  void ConvertQuaternionToEuler(const geometry_msgs::msg::PoseStamped& pose_stamped, double& roll, double& pitch, double& yaw) {
        // 获取四元数
        tf2::Quaternion quaternion;
        tf2::fromMsg(pose_stamped.pose.orientation, quaternion);
    
        // 创建旋转矩阵
        tf2::Matrix3x3 rotation_matrix(quaternion);
    
        // 获取欧拉角
        rotation_matrix.getRPY(roll, pitch, yaw);
    }

    

    std::thread PublishThread_;
    std::thread SuckerPublishThread_;

    rclcpp::Publisher<interfaces::msg::ToolCenterPose>::SharedPtr status_pub_; // 发布TCP状态的发布器
    rclcpp::Publisher<interfaces::msg::ToolCenterPose>::SharedPtr quaternion_pub_; // 发布四元数的发布器
    rclcpp::Publisher<interfaces::msg::ToolCenterPose>::SharedPtr sucker_status_pub_; //发布吸盘状态的发布器

    rclcpp::Service<interfaces::srv::GetTetris>::SharedPtr get_tetris_srv_;    // 获取方块的服务
    rclcpp::Service<interfaces::srv::PutTetris>::SharedPtr put_tetris_srv_;    // 放置方块的服务
    rclcpp::Service<interfaces::srv::ResetMotion>::SharedPtr reset_motion_srv_; //重置位姿的服务
private:
    void GetTetrisCallback(
        const std::shared_ptr<interfaces::srv::GetTetris::Request> request,
        std::shared_ptr<interfaces::srv::GetTetris::Response> response
    );

    void PutTetrisCallback(
        const std::shared_ptr<interfaces::srv::PutTetris::Request> request,
        std::shared_ptr<interfaces::srv::PutTetris::Response> response
    );

    void ResetMotionCallback(
        const std::shared_ptr<interfaces::srv::ResetMotion::Request> request,
        std::shared_ptr<interfaces::srv::ResetMotion::Response> response
    );


    XArmAPI* xarm_;
    ServoMotor serve;      // 伺服电机
    cv::Point3d left_bot;  // 左下角
    cv::Point3d left_top;  // 左上角
    cv::Point3d right_top; // 右上角
    cv::Point3d right_bot; // 右下角
    double tool2sucker_z;
    float reset_pose[6]; //重置初始位姿
    float second_reset_pose[6]; //重置第二次位姿
    float reset_joint_pose[7]; //重置初始关节位姿
    float second_reset_joint_pose[7]; //重置第二次关节位姿
    
    float reset_joint_pose_moving[7]; //重置运动开始后关节位姿
    
    bool reset_flag = false;
    bool first_get_flag = false;  //用于记录是否为第一次抓取的标志位，第一次抓取直接点对点，第二次之后全用movespline实现自定义轨迹
    bool using_joint = false;

    float put_joint_pose[7]; //放置连杆关节角度
    float second_put_joint_pose[7]; //放置连杆第二关节角度

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};