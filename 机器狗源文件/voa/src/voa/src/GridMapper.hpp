
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <thread>
#include <deque>

#include "MyGridMap.hpp"

class GridMapper : public rclcpp::Node{
public:
    GridMapper(rclcpp::NodeOptions options)
    : Node("grid_mapper", options)
    {
        std::vector<std::string> topic_names;
        topic_names.push_back("/camera/depth/color/transformed_points");
        robot_foot_print_frame_name_ = "robot_foot_print";

        declare_parameter("timeout_s_gm", 0.200);        
        if(get_parameter("timeout_s_gm", timeout_s_)){
            RCLCPP_INFO(get_logger(), "timeout_s_gm is: %lf", timeout_s_);
        }
        else    RCLCPP_WARN(get_logger(), "get_parameter timeout_s_gm failed");

        GridMapProcessParams gm_process_params;
        declare_parameter("height", 6);
        if(get_parameter("height", gm_process_params.height)){
            RCLCPP_INFO(get_logger(), "height is: %d", gm_process_params.height);
        }
        else    RCLCPP_WARN(get_logger(), "get_parameter height failed");

        declare_parameter("width", 4);
        if(get_parameter("width", gm_process_params.width)){
            RCLCPP_INFO(get_logger(), "width is: %d", gm_process_params.width);
        }
        else    RCLCPP_WARN(get_logger(), "get_parameter width failed");

        declare_parameter("resolution", 0.05);
        if(get_parameter("resolution", gm_process_params.resolution)){
            RCLCPP_INFO(get_logger(), "resolution is: %lf", gm_process_params.resolution);
        }
        else    RCLCPP_WARN(get_logger(), "get_parameter resolution failed");

        declare_parameter("use_cv", true);
        if(get_parameter("use_cv", gm_process_params.use_cv)){
            RCLCPP_INFO(get_logger(), "use_cv is: %d", gm_process_params.use_cv);
        }
        else    RCLCPP_WARN(get_logger(), "get_parameter use_cv failed");

        declare_parameter("use_cuda", true);
        if(get_parameter("use_cuda", gm_process_params.use_cuda)){
            RCLCPP_INFO(get_logger(), "use_cuda is: %d", gm_process_params.use_cuda);
        }
        else    RCLCPP_WARN(get_logger(), "get_parameter use_cuda failed");

        declare_parameter("is_dilate", true);
        if(get_parameter("is_dilate", gm_process_params.is_dilate)){
            RCLCPP_INFO(get_logger(), "is_dilate is: %d", gm_process_params.is_dilate);
        }
        else    RCLCPP_WARN(get_logger(), "get_parameter is_dilate failed");

        declare_parameter("dilate_kernel_size", 3);
        if(get_parameter("dilate_kernel_size", gm_process_params.dilate_kernel_size)){
            RCLCPP_INFO(get_logger(), "dilate_kernel_size is: %d", gm_process_params.dilate_kernel_size);
        }
        else    RCLCPP_WARN(get_logger(), "get_parameter dilate_kernel_size failed");

        declare_parameter("is_erode", true);
        if(get_parameter("is_erode", gm_process_params.is_erode)){
            RCLCPP_INFO(get_logger(), "is_erode is: %d", gm_process_params.is_erode);
        }
        else    RCLCPP_WARN(get_logger(), "get_parameter is_erode failed");

        declare_parameter("erode_kernel_size", 3);
        if(get_parameter("erode_kernel_size", gm_process_params.erode_kernel_size)){
            RCLCPP_INFO(get_logger(), "erode_kernel_size is: %d", gm_process_params.erode_kernel_size);
        }
        else    RCLCPP_WARN(get_logger(), "get_parameter erode_kernel_size failed");                                        

        grid_map_ = std::make_shared<MyGridMap>((rclcpp::Node::SharedPtr)(this), gm_process_params);

        int i = 0; 
        cloud_buffer_mutexs_.emplace_back(std::shared_ptr<std::mutex>(new std::mutex()));
        cloud_buffers_.emplace_back(std::shared_ptr<std::deque<sensor_msgs::msg::PointCloud2::SharedPtr>>(new std::deque<sensor_msgs::msg::PointCloud2::SharedPtr>()));

        for(auto &topic_name : topic_names){
            point_cloud_subs_.emplace_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(
                topic_name,
                10,
                [i, this](sensor_msgs::msg::PointCloud2::SharedPtr msg){
                    auto &stamp_ref_pc = msg->header.stamp;
                    double t_diff_s = std::abs(((double)stamp_ref_pc.sec + (double)stamp_ref_pc.nanosec/1e9) - rclcpp::Clock(RCL_STEADY_TIME).now().seconds());
                    if(t_diff_s < timeout_s_){
                        {
                            std::lock_guard<std::mutex> lock(*cloud_buffer_mutexs_.at(i));    
                            cloud_buffers_.at(i)->emplace_back(msg);
                        }
                    }
                    else{
                        RCLCPP_WARN(this->get_logger(), "Received a timeout frame, timeout for %lfs", t_diff_s);
                    }
                }
            ));
            i++;
        }
        RCLCPP_INFO(get_logger(), "point_cloud_subs_.size(): %d", point_cloud_subs_.size());

        grid_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("grid_map", 10);
        grid_map_generate_thread_ = std::thread(&GridMapper::GridMapGenerateThreadFunc, this);
    }

    bool CheckDataReady(){
        bool data_ready = true;
        for(int i = 0; i<cloud_buffers_.size(); i++){
            std::lock_guard<std::mutex> lock(*cloud_buffer_mutexs_.at(i));
            if(cloud_buffers_.at(i)->empty()){
                data_ready = false;
                break;
            }
        }
        return data_ready;
    }

    void ClearTimeoutData(){
        for(int i = 0; i<cloud_buffers_.size(); i++){
            std::lock_guard<std::mutex> lock(*cloud_buffer_mutexs_.at(i));
            while(cloud_buffers_.at(i)->size() > 0){
                auto &stamp_ref_pc = cloud_buffers_.at(i)->front()->header.stamp;
                double t_diff_s = std::abs(((double)stamp_ref_pc.sec + (double)stamp_ref_pc.nanosec/1e9) - rclcpp::Clock(RCL_STEADY_TIME).now().seconds());
                if(t_diff_s > timeout_s_){
                    cloud_buffers_.at(i)->pop_front();
                }
                else{
                    break;
                }
            }
        }
    }

    void AddPointCloud2GridMap(){
        for(int i = 0; i<cloud_buffers_.size(); i++){            
            std::lock_guard<std::mutex> lock(*cloud_buffer_mutexs_.at(i));
            if(cloud_buffers_.at(i)->size() > 0){
                grid_map_->Add(cloud_buffers_.at(i)->front());
                cloud_buffers_.at(i)->pop_front();
            }
        }
    }

    void ProcessGridMap(){
        grid_map_->Process();
    }

    void PublishGridMap(){
        grid_map_msgs::msg::GridMap::UniquePtr grid_map_msg_
            = grid_map::GridMapRosConverter::toMessage(grid_map_->GetGridMap());
        grid_map_msg_->header.frame_id = robot_foot_print_frame_name_;
        grid_map_msg_->header.stamp = grid_map_->GetTimeStamp();
        grid_map_msg_->info.pose.position.x = 0.0;
        grid_map_msg_->info.pose.position.y = 0.0;
        grid_map_msg_->info.pose.position.z = 0.0;
        grid_map_msg_->info.pose.orientation.x = 0.0;
        grid_map_msg_->info.pose.orientation.y = 0.0;
        grid_map_msg_->info.pose.orientation.z = 0.0;
        grid_map_msg_->info.pose.orientation.w = 1.0;
        grid_map_pub_->publish(std::move(grid_map_msg_));
    }

    void ClearGridMap(){
        grid_map_->Clear();
    }

    void GridMapGenerateThreadFunc(){
        while(rclcpp::ok()){
            if(!CheckDataReady()){
                // RCLCPP_INFO(get_logger(), "data not ready");
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
            ClearTimeoutData();
            if(!CheckDataReady()){
                // RCLCPP_INFO(get_logger(), "data not ready after clear timeout data");
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
            AddPointCloud2GridMap();
            ProcessGridMap();
            PublishGridMap();
            ClearGridMap();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

private:
    std::string robot_foot_print_frame_name_;
    double timeout_s_;

    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> point_cloud_subs_;
    std::vector<std::shared_ptr<std::mutex>> cloud_buffer_mutexs_;
    std::vector<std::shared_ptr<std::deque<sensor_msgs::msg::PointCloud2::SharedPtr>>> cloud_buffers_;
    std::thread grid_map_generate_thread_;
    std::shared_ptr<MyGridMap> grid_map_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(GridMapper)

