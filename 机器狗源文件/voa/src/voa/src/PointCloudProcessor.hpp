#include "rclcpp/rclcpp.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <queue>
#include <thread>

struct PointCloudProcessParams{
    PointCloudProcessParams()
    : timeout_s(0),
    pass_x_min(0), pass_x_max(0), 
    pass_y_min(0), pass_y_max(0), 
    leaf_size(0) {}
    PointCloudProcessParams(
        double _timeout_s, 
        double _pass_x_min, double _pass_x_max, 
        double _pass_y_min, double _pass_y_max, 
        double _leaf_size)
    : timeout_s(_timeout_s),
    pass_x_min(_pass_x_min), pass_x_max(_pass_x_max), 
    pass_y_min(_pass_y_min), pass_y_max(_pass_y_max), 
    leaf_size(_leaf_size) {}
    double timeout_s;
    double pass_x_min;
    double pass_x_max;
    double pass_y_min;
    double pass_y_max;
    double leaf_size;
};

constexpr int kMaxImuBufferSize = 20;

class PointCloudProcessor : public rclcpp::Node{
public:
    PointCloudProcessor(rclcpp::NodeOptions options)        
    :   Node("realsense_point_cloud_processer", options),
        tf_buffer_(std::make_shared<rclcpp::Clock>(rclcpp::Clock(RCL_STEADY_TIME))),
        transform_listener_(tf_buffer_){
        robot_foot_print_frame_name_ = "robot_foot_print";
        robot_base_frame_name_ = "base_link";
        point_cloud_frame_name_ = "camera_depth_optical_frame";
        std::string point_cloud_in_topic_name = "/camera/depth/color/points";
        std::string point_cloud_out_topic_name = "/camera/depth/color/transformed_points";
        std::string imu_topic_name = "/imu/data";


        tf_buffer_.setUsingDedicatedThread(true);

        declare_parameter("timeout_s_pc", 0.2);
        if(get_parameter("timeout_s_pc", pc_process_params_.timeout_s)){
            RCLCPP_INFO(get_logger(), "timeout_s_pc is: %lf", pc_process_params_.timeout_s);
        }
        else    RCLCPP_WARN(get_logger(), "get_parameter timeout_s_pc failed");

        declare_parameter("pass_x_min", 0.0);
        if(get_parameter("pass_x_min", pc_process_params_.pass_x_min)){
            RCLCPP_INFO(get_logger(), "pass_x_min is: %lf", pc_process_params_.pass_x_min);
        }
        else    RCLCPP_WARN(get_logger(), "get_parameter pass_x_min failed");
        declare_parameter("pass_x_max", 3.0);
        if(get_parameter("pass_x_max", pc_process_params_.pass_x_max)){
            RCLCPP_INFO(get_logger(), "pass_x_max is: %lf", pc_process_params_.pass_x_max);
        }
        else    RCLCPP_WARN(get_logger(), "get_parameter pass_x_max failed");
        RCLCPP_INFO(get_logger(), "pc_process_params_.pass_x_max: %lf", pc_process_params_.pass_x_max);

        declare_parameter("pass_y_min", -1.5);
        if(get_parameter("pass_y_min", pc_process_params_.pass_y_min)){
            RCLCPP_INFO(get_logger(), "pass_y_min is: %lf", pc_process_params_.pass_y_min);
        }
        else    RCLCPP_WARN(get_logger(), "get_parameter pass_y_min failed");

        declare_parameter("pass_y_max", 1.5);
        if(get_parameter("pass_y_max", pc_process_params_.pass_y_max)){
            RCLCPP_INFO(get_logger(), "pass_y_max is: %lf", pc_process_params_.pass_y_max);
        }
        else    RCLCPP_WARN(get_logger(), "get_parameter pass_y_max failed");

        declare_parameter("leaf_size", 0.05);
        if(get_parameter("leaf_size", pc_process_params_.leaf_size)){
            RCLCPP_INFO(get_logger(), "leaf_size is: %lf", pc_process_params_.leaf_size);
        }
        else    RCLCPP_WARN(get_logger(), "get_parameter leaf_size failed");

        using std::placeholders::_1;
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(imu_topic_name, 10, std::bind(&PointCloudProcessor::ImuDataCallback, this, _1));
        point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(point_cloud_in_topic_name, 10, std::bind(&PointCloudProcessor::PointCloudCallback, this, _1));
        point_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(point_cloud_out_topic_name, 10);

        CalCamera2BaseStaticTF();

        RCLCPP_INFO(get_logger(), "PointCloudProcessor for %s initialized!", point_cloud_frame_name_.c_str());
    }

    ~PointCloudProcessor(){
        RCLCPP_INFO(get_logger(), "~PointCloudProcessor()");
    }

private:
    void CalCamera2BaseStaticTF(){
        while(!tf_buffer_.canTransform(robot_base_frame_name_, point_cloud_frame_name_, rclcpp::Clock(RCL_STEADY_TIME).now())){
            RCLCPP_WARN(get_logger(), "Waiting for transform from %s to %s", point_cloud_frame_name_.c_str(), robot_base_frame_name_.c_str());
            sleep(1);
        }
        geometry_msgs::msg::TransformStamped camera2base_transform_ = tf_buffer_.lookupTransform(robot_base_frame_name_, point_cloud_frame_name_, rclcpp::Time(3, 0));
        Eigen::Quaternionf rotation(camera2base_transform_.transform.rotation.w,
                                    camera2base_transform_.transform.rotation.x,
                                    camera2base_transform_.transform.rotation.y,
                                    camera2base_transform_.transform.rotation.z);
        Eigen::Vector3f translation(camera2base_transform_.transform.translation.x,
                                    camera2base_transform_.transform.translation.y,
                                    camera2base_transform_.transform.translation.z);
        camera2base_transform_matrix_.setIdentity();
        camera2base_transform_matrix_.block<3, 3>(0, 0) = rotation.toRotationMatrix();
        camera2base_transform_matrix_.block<3, 1>(0, 3) = translation;

        #ifdef DEBUG_TF
        std::cout << "w,x,y,z: " << camera2base_transform_.transform.rotation.w << " "
            << camera2base_transform_.transform.rotation.x << " "
            << camera2base_transform_.transform.rotation.y << " "
            << camera2base_transform_.transform.rotation.z << std::endl;
        std::cout << "rotation.toRotationMatrix()" << rotation.toRotationMatrix() << std::endl;
        std::cout << "x,y,z: " << camera2base_transform_.transform.translation.x << " "
                    << camera2base_transform_.transform.translation.y << " "
                    << camera2base_transform_.transform.translation.z << " " << std::endl;
        std::cout << "translation" << translation << std::endl;
        std::cout << "camera2base_transform_matrix_: " << camera2base_transform_matrix_ << std::endl;
        #endif
    }

    void CalculateBase2FootPrintTF(sensor_msgs::msg::Imu::SharedPtr msg, Eigen::Matrix4f &base2footprint_transform_matrix){
        tf2::Quaternion quat;
        tf2::fromMsg(msg->orientation, quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf2::Quaternion(quat)).getRPY(roll, pitch, yaw);
        quat.setRPY(roll, pitch, 0.0);
        Eigen::Quaternionf eigen_quat(quat.w(), quat.x(), quat.y(), quat.z());
        base2footprint_transform_matrix = Eigen::Matrix4f::Identity();
        base2footprint_transform_matrix.block<3, 3>(0, 0) = eigen_quat.toRotationMatrix();
        base2footprint_transform_matrix(0, 3) = 0;
        base2footprint_transform_matrix(1, 3) = 0;
        base2footprint_transform_matrix(2, 3) = 0.34751;

        #ifdef DEBUG_TF
        std::cout << "rpy: " << roll << pitch << yaw << std::endl;
        std::cout << "base2footprint_transform_matrix: " << base2footprint_transform_matrix << std::endl;
        #endif        
    }

    void ImuDataCallback(const sensor_msgs::msg::Imu::SharedPtr msg){
        std::lock_guard<std::mutex> lock(imu_data_buffer_mutex_);
        imu_data_buffer_.emplace_back(msg);
        if(imu_data_buffer_.size() > kMaxImuBufferSize){
            imu_data_buffer_.pop_front();
        }
        if(!first_imu_data_received_){
            first_imu_data_received_ = true;
        }
    }

    bool Pop2ClosestImuData(const rclcpp::Time &time){
        std::lock_guard<std::mutex> lock(imu_data_buffer_mutex_);

        if(imu_data_buffer_.empty()){
            return false;
        }

        int index = 0;
        auto &stamp_ref = imu_data_buffer_.front()->header.stamp;
        double min_time_diff = std::abs(((double)stamp_ref.sec + (double)stamp_ref.nanosec/1e9) - time.seconds());
        for(int i = 0; i < imu_data_buffer_.size(); i++){
            auto &stamp_ref_i = imu_data_buffer_.at(i)->header.stamp;
            double time_diff = std::abs(((double)stamp_ref_i.sec + (double)stamp_ref_i.nanosec/1e9) - time.seconds());
            if(time_diff < min_time_diff){
                min_time_diff = time_diff;
                index = i;
            }
        }

        if(index > 0){
            for(int i = 0; i < index; i++){
                imu_data_buffer_.pop_front();
            }
        }

        return true;
    }

    void PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
        if(!first_imu_data_received_){
            RCLCPP_WARN(get_logger(), "First imu data not received!");
            return;
        }

        auto &stamp_ref_pc = msg->header.stamp;
        double t_diff_s_pc = std::abs(((double)stamp_ref_pc.sec + (double)stamp_ref_pc.nanosec/1e9) - rclcpp::Clock(RCL_STEADY_TIME).now().seconds());
        if(t_diff_s_pc > pc_process_params_.timeout_s){
            RCLCPP_WARN(get_logger(), "Point cloud data timeout for %lf", t_diff_s_pc);
            return;
        }

        if(!Pop2ClosestImuData(msg->header.stamp)){
            return;
            RCLCPP_WARN(get_logger(), "imu_data_buffer is empty");
        };

        sensor_msgs::msg::Imu::SharedPtr imu_data;
        {
            std::lock_guard<std::mutex> lock(imu_data_buffer_mutex_);
            imu_data = imu_data_buffer_.front();
            imu_data_buffer_.pop_front();
        }

        auto &stamp_ref_imu = imu_data->header.stamp;
        double t_diff_s_imu = std::abs(((double)stamp_ref_imu.sec + (double)stamp_ref_imu.nanosec/1e9) - rclcpp::Clock(RCL_STEADY_TIME).now().seconds());
        if(t_diff_s_imu > pc_process_params_.timeout_s){
            RCLCPP_WARN(get_logger(), "Imu data timeout for %lf", t_diff_s_imu);
            return;
        }

        Eigen::Matrix4f base2footprint_transform_matrix;
        CalculateBase2FootPrintTF(imu_data, base2footprint_transform_matrix);
        pcl_ros::transformPointCloud(camera2base_transform_matrix_, *msg, *msg);
        pcl_ros::transformPointCloud(base2footprint_transform_matrix, *msg, *msg);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        FilterPointCloudPassThrough(cloud);
        FilterPointCloudVoxelGrid(cloud);

        sensor_msgs::msg::PointCloud2::UniquePtr transformed_point_cloud(new sensor_msgs::msg::PointCloud2());
        pcl::toROSMsg(*cloud, *transformed_point_cloud);
        transformed_point_cloud->header.stamp = msg->header.stamp;
        transformed_point_cloud->header.frame_id = robot_foot_print_frame_name_;
        point_cloud_pub_->publish(std::move(transformed_point_cloud));
    }

    void FilterPointCloudPassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setFilterFieldName("x");
        pass.setFilterLimits(pc_process_params_.pass_x_min, pc_process_params_.pass_x_max);
        pass.setInputCloud(cloud);
        pass.filter(*cloud);

        pass.setFilterFieldName("y");
        pass.setFilterLimits(pc_process_params_.pass_y_min, pc_process_params_.pass_y_max);
        pass.setInputCloud(cloud);
        pass.filter(*cloud);
    }

    void FilterPointCloudVoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setLeafSize(pc_process_params_.leaf_size, pc_process_params_.leaf_size, pc_process_params_.leaf_size);
        sor.setInputCloud(cloud);
        sor.filter(*cloud);
    }

    PointCloudProcessParams pc_process_params_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    std::mutex imu_data_buffer_mutex_;
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_data_buffer_;
    bool first_imu_data_received_ = false;

    std::string robot_foot_print_frame_name_;
    std::string robot_base_frame_name_;
    std::string point_cloud_frame_name_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener transform_listener_;
    Eigen::Matrix4f camera2base_transform_matrix_;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PointCloudProcessor)

