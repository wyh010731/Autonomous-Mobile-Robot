#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <fstream>
#include <vector>
#include <sstream>
#include <chrono>

class MapPublisher : public rclcpp::Node
{
public:
    MapPublisher() : Node("map_publisher")
    {
        // 加载YAML配置文件
        const std::string yaml_path = get_map_yaml_path();
        YAML::Node config = YAML::LoadFile(yaml_path);

        // 解析YAML参数
        parse_yaml_parameters(config);

        // 加载并转换PGM数据
        std::vector<int8_t> map_data = load_pgm_data();

        // 发布地图
        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
        // 在构造函数中添加定时器，持续发布地图
        timer_ = create_wall_timer(
            std::chrono::seconds(1),
            [this]()
            {
                std::vector<int8_t> data = load_pgm_data();
                publish_map(data);
            });

        // 添加成员变量
        rclcpp::TimerBase::SharedPtr timer_;

        // 发布静态TF转换
        publish_static_tf();
    }

private:
    std::string get_map_yaml_path()
    {
        std::string pkg_path = ament_index_cpp::get_package_share_directory("amr_planner");
        std::string yaml_path = pkg_path + "/map/room.yaml";
        RCLCPP_INFO(this->get_logger(), "Loading map YAML from: %s", yaml_path.c_str());
        return yaml_path;
    }

    void parse_yaml_parameters(const YAML::Node &config)
    {
        try
        {
            resolution_ = config["resolution"].as<double>();
            origin_ = {
                config["origin"][0].as<double>(),
                config["origin"][1].as<double>(),
                config["origin"][2].as<double>()};

            // 检查可选参数
            if (config["occupied_thresh"])
            {
                occupied_thresh_ = config["occupied_thresh"].as<double>();
            }
            else
            {
                occupied_thresh_ = 0.65; // 默认值
            }

            if (config["free_thresh"])
            {
                free_thresh_ = config["free_thresh"].as<double>();
            }
            else
            {
                free_thresh_ = 0.25; // 默认值
            }

            std::string pkg_path = ament_index_cpp::get_package_share_directory("amr_planner");
            pgm_path_ = pkg_path + "/map/" + config["image"].as<std::string>();

            RCLCPP_INFO(this->get_logger(), "Map parameters parsed");
        }
        catch (const YAML::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "YAML error: %s", e.what());
            rclcpp::shutdown();
        }
    }

    std::vector<int8_t> load_pgm_data()
    {
        std::ifstream file(pgm_path_, std::ios::binary);
        if (!file)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not open PGM file: %s", pgm_path_.c_str());
            rclcpp::shutdown();
            return {};
        }

        std::string line;
        // 检查PGM格式
        std::getline(file, line);
        if (line != "P5")
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid PGM file type (must be P5)");
            rclcpp::shutdown();
            return {};
        }

        // 跳过注释行
        // 动态跳过所有注释行和空行
        bool found_dimensions = false;
        while (std::getline(file, line))
        {
            if (line.empty())
                continue; // 跳过空行
            if (line[0] == '#')
                continue; // 跳过注释行
            found_dimensions = true;
            break;
        }
        if (!found_dimensions)
        {
            RCLCPP_ERROR(this->get_logger(), "PGM文件中未找到地图尺寸信息");
            rclcpp::shutdown();
            return {};
        }

        // 解析宽度和高度
        std::istringstream iss(line);
        if (!(iss >> map_width_ >> map_height_))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse map dimensions");
            rclcpp::shutdown();
            return {};
        }

        // 读取最大像素值
        std::getline(file, line);
        int max_val;
        try
        {
            max_val = std::stoi(line);
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid max pixel value");
            rclcpp::shutdown();
            return {};
        }

        // 读取像素数据
        std::vector<uint8_t> pgm_data(map_width_ * map_height_);
        file.read(reinterpret_cast<char *>(pgm_data.data()), pgm_data.size());
        size_t bytes_read = file.gcount(); // 获取实际读取的字节数
        if (bytes_read != pgm_data.size())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "PGM文件读取不完整! 预期: %zu字节, 实际: %zu字节",
                pgm_data.size(),
                bytes_read);
            rclcpp::shutdown();
            return {};
        }

        // 转换为OccupancyGrid数据
        std::vector<int8_t> data;
        data.reserve(pgm_data.size());
        for (auto pixel : pgm_data)
        {
            double val = static_cast<double>(pixel) / max_val;
            if (val >= occupied_thresh_)
            {
                data.push_back(100); // 占用
            }
            else if (val <= free_thresh_)
            {
                data.push_back(0); // 空闲
            }
            else
            {
                data.push_back(-1); // 未知
            }
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %zu map cells", data.size());
        return data;
    }

    void publish_map(const std::vector<int8_t> &data)
    {
        auto msg = nav_msgs::msg::OccupancyGrid();
        msg.header.stamp = this->now();
        msg.header.frame_id = "map"; // 必须与TF发布的frame_id一致

        msg.info.resolution = resolution_;
        msg.info.width = map_width_;
        msg.info.height = map_height_;
        msg.info.origin.position.x = origin_[0];
        msg.info.origin.position.y = origin_[1];
        msg.info.origin.position.z = origin_[2];

        msg.data = data;

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Map published successfully");
    }

    void publish_static_tf()
    {
        auto static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "map"; // 父坐标系
        transform.child_frame_id = "odom"; // 子坐标系

        // 设置变换参数（根据实际需要调整）
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;

        static_tf_broadcaster->sendTransform(transform);
        RCLCPP_INFO(this->get_logger(), "Published static TF: map -> odom");
    }

    // 成员变量
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
    double resolution_;
    std::array<double, 3> origin_;
    double occupied_thresh_ = 0.65;
    double free_thresh_ = 0.25;
    std::string pgm_path_;
    int map_width_;
    int map_height_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}