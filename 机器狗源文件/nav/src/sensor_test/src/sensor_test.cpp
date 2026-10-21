#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp" // 添加PointCloud2消息头文件
#include <chrono>

class SensorTest : public rclcpp::Node
{
public:
    SensorTest() : Node("sensor_test")
    {
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10, std::bind(&SensorTest::imuCallback, this, std::placeholders::_1));
        
        lidar_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/rslidar_points", 10, std::bind(&SensorTest::lidarCallback, this, std::placeholders::_1));

        realsense_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/color/points", 10, std::bind(&SensorTest::realsenseCallback, this, std::placeholders::_1));
        
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&SensorTest::timerCallback, this));
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        imu_count_++;
    }

    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        lidar_count_++;
    }

    void realsenseCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        realsense_count_++;
    }

    void timerCallback()
    {
        RCLCPP_INFO(this->get_logger(), 
            "Received %d imu, %d lidar, %d realsense in the last second.",
            imu_count_, lidar_count_, realsense_count_
        );
        imu_count_ = 0;
        lidar_count_ = 0;
        realsense_count_ = 0;
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr realsense_subscription_;

    int imu_count_ = 0;
    int lidar_count_ = 0;
    int realsense_count_ = 0;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
