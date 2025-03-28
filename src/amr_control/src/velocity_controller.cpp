// 引入ROS2 C++核心库和Twist消息类型
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals; // 简化时间单位（如100ms）

// 控制节点类，继承自ROS2 Node
class VelocityController : public rclcpp::Node
{
public:
    VelocityController() : Node("velocity_controller") // 节点名"velocity_controller"
    {
        /* 创建发布者（Publisher）
         * 话题名称：/cmd_vel
         * 消息类型：geometry_msgs::msg::Twist
         * 队列长度：10
         */
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        /* 创建定时器（Timer）
         * 周期：100ms（10Hz）
         * 回调函数：Lambda表达式，周期性发布速度指令
         */
        timer_ = this->create_wall_timer(
            100ms,
            [this]() { // Lambda捕获this指针以访问类成员
                auto msg = geometry_msgs::msg::Twist();

                // 设置线速度（m/s）- 沿X轴前进
                msg.linear.x = 0.2; // 正数前进，负数后退

                // 设置角速度（rad/s）- 绕Z轴旋转
                msg.angular.z = 0.5; // 正数左转，负数右转

                publisher_->publish(msg); // 发布消息
            });
    }

private:
    // 声明成员变量
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; // Twist消息发布者
    rclcpp::TimerBase::SharedPtr timer_;                                // 定时器
};

int main(int argc, char **argv)
{
    // 初始化ROS2上下文
    rclcpp::init(argc, argv);

    // 创建节点实例并保持运行
    auto node = std::make_shared<VelocityController>();
    rclcpp::spin(node); // 阻塞式等待回调

    // 关闭ROS2上下文
    rclcpp::shutdown();
    return 0;
}