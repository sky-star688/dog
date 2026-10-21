
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;
class FakeImuPublisher : public rclcpp::Node{
public:
    FakeImuPublisher()
    : Node("fake_imu_publisher"){
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
        timer_ = create_wall_timer(10ms, std::bind(&FakeImuPublisher::PublishFakeImu, this));
    };

    void PublishFakeImu(){
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = rclcpp::Clock(RCL_STEADY_TIME).now();
        imu_msg.header.frame_id = "base_link";
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        imu_msg.orientation = tf2::toMsg(q);
        imu_msg.angular_velocity.x = 0;
        imu_msg.angular_velocity.y = 0;
        imu_msg.angular_velocity.z = 0;
        imu_msg.linear_acceleration.x = 0;
        imu_msg.linear_acceleration.y = 0;
        imu_msg.linear_acceleration.z = 9.8;
        imu_pub_->publish(imu_msg);
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto fake_imu_publisher = std::make_shared<FakeImuPublisher>();
    rclcpp::spin(fake_imu_publisher);
    rclcpp::shutdown();
    return 0;
}