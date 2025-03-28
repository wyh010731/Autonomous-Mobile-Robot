#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <mutex>  // 互斥锁
#include <cmath>  // 数学计算
#include <limits> // 数值范围限制
#include "nav_msgs/msg/odometry.hpp"   // 里程计消息头文件
#include <tf2/LinearMath/Quaternion.h> // 四元数转换
#include <tf2/LinearMath/Matrix3x3.h>


using namespace std::chrono_literals;

class VelocityController : public rclcpp::Node
{
public:
    VelocityController() : Node("laser_sensor"), obstacle_detected_(false)
    {
        // 初始化参数
        this->declare_parameter<double>("safety_distance", 0.5);
        this->declare_parameter<double>("detection_angle", M_PI_2); // 默认90度检测范围

        // 创建速度发布者
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // 创建激光雷达订阅者
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&VelocityController::scan_callback, this, std::placeholders::_1));

        // 创建里程计订阅者
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&VelocityController::odom_callback, this, std::placeholders::_1));

        // 创建控制定时器
        timer_ = this->create_wall_timer(
            100ms,
            [this]()
            {
                geometry_msgs::msg::Twist cmd_msg;

                // 加锁读取障碍物状态
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (obstacle_detected_)
                    {
                        // 检测到障碍物，停止运动
                        cmd_msg.linear.x = 0.0;
                        cmd_msg.angular.z = 0.0;
                        RCLCPP_WARN(this->get_logger(), "Obstacle detected! Stopping...");
                    }
                    else
                    {
                        // 正常运动指令
                        cmd_msg.linear.x = 0.2;
                        cmd_msg.angular.z = 0.0;
                    }
                }

                publisher_->publish(cmd_msg);
            });
    }

private:
    // 激光雷达数据回调
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // 获取参数
        const double safety_dist = this->get_parameter("safety_distance").as_double();
        const double detect_angle = this->get_parameter("detection_angle").as_double();

        // 计算检测窗口中心索引（正前方）
        const double center_angle = 0.0; // 假设正前方为0弧度
        const int center_index = static_cast<int>((center_angle - msg->angle_min) / msg->angle_increment);
        const int half_window = static_cast<int>((detect_angle / 2) / msg->angle_increment);
        int start_idx = center_index - half_window;
        int end_idx = center_index + half_window;

        // 处理索引越界
        start_idx = std::max(0, start_idx);
        end_idx = std::min(static_cast<int>(msg->ranges.size() - 1), end_idx);

        // 寻找最小距离
        double min_distance = std::numeric_limits<double>::infinity();
        for (int i = start_idx; i <= end_idx; ++i)
        {
            const float range = msg->ranges[i];
            if (!std::isnan(range) && !std::isinf(range) && range >= msg->range_min)
            {
                min_distance = std::min(min_distance, static_cast<double>(range));
            }
        }

        // 更新障碍物状态
        std::lock_guard<std::mutex> lock(mutex_);
        obstacle_detected_ = (min_distance <= safety_dist);

        // 调试输出
        RCLCPP_DEBUG(this->get_logger(), "Min distance: %.2f, Obstacle detected: %d",
                     min_distance, obstacle_detected_);
    }

    // 里程计回调函数
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 获取位置（x, y, z）
        double pos_x = msg->pose.pose.position.x;
        double pos_y = msg->pose.pose.position.y;
        double pos_z = msg->pose.pose.position.z;

        // 获取姿态（四元数转欧拉角）
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw); // 获取绕X/Y/Z轴的欧拉角

        // 获取线速度和角速度
        double linear_vel_x = msg->twist.twist.linear.x;
        double angular_vel_z = msg->twist.twist.angular.z;

        // 打印信息（实际应用中建议使用调试输出）
        RCLCPP_INFO(this->get_logger(), "Position: (%.2f, %.2f, %.2f)", pos_x, pos_y, pos_z);
        RCLCPP_INFO(this->get_logger(), "Yaw: %.2f rad", yaw);
        RCLCPP_INFO(this->get_logger(), "Velocity: linear=%.2f m/s, angular=%.2f rad/s",
                    linear_vel_x, angular_vel_z);
    }

    // 成员变量
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    std::mutex mutex_;       // 互斥锁
    bool obstacle_detected_; // 障碍物检测标志
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VelocityController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}