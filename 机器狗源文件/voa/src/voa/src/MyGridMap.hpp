
#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/point_cloud2.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_cv/grid_map_cv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>

#include <chrono>

struct GridMapProcessParams{
    GridMapProcessParams()
    : height(0), width(0), resolution(0),
    use_cv(false), use_cuda(false),
    is_dilate(false), dilate_kernel_size(0),
    is_erode(false), erode_kernel_size(0) {}
    GridMapProcessParams(
        int _height, int _width, double _resolution, 
        bool _use_cv, bool _use_cuda,
        bool _is_dilate, int _dilate_kernel_size,
        bool _is_erode, int _erode_kernel_size)
    : height(_height), width(_width), resolution(_resolution),
    use_cv(_use_cv), use_cuda(_use_cuda),
    is_dilate(_is_dilate), dilate_kernel_size(_dilate_kernel_size),
    is_erode(_is_erode), erode_kernel_size(_erode_kernel_size){}
    int height;
    int width;
    double resolution;
    bool use_cv;
    bool use_cuda;
    bool is_dilate;
    int dilate_kernel_size;
    bool is_erode;
    int erode_kernel_size;
};

class MyGridMap{
public:
    MyGridMap(rclcpp::Node::SharedPtr node, GridMapProcessParams &gm_process_params)
    : node_(node), gm_process_params_(gm_process_params){
        grid_map_.setGeometry(grid_map::Length(gm_process_params_.height, gm_process_params_.width), gm_process_params_.resolution);
        grid_map_.add("elevation");
        time_stamp_ = rclcpp::Clock(RCL_STEADY_TIME).now();
    }

    void Add(sensor_msgs::msg::PointCloud2::SharedPtr msg){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        for(const auto& point : cloud->points)
        {
            grid_map::Index index;
            grid_map::Position position(point.x, point.y);
            if(!grid_map_.getIndex(position, index)) continue;
            auto& elevation = grid_map_.atPosition("elevation", position);
            if(std::isnan(elevation)){
                elevation = point.z;
            }
            else if(elevation < point.z){
                elevation = point.z;
            }

            // RCLCPP_INFO(node_->get_logger(), "height: %lf", point.z);
        }

        auto &stamp_ref_pc = msg->header.stamp;
        rclcpp::Time time((double)stamp_ref_pc.sec, (double)stamp_ref_pc.nanosec/1e9, RCL_STEADY_TIME);
        if(time_stamp_ < time){
            time_stamp_ = time;
        }
    }

    void Process(){
        if(gm_process_params_.use_cv){
            #ifdef DEBUG_GRIDMAP
            auto start = std::chrono::high_resolution_clock::now();
            #endif

            if(gm_process_params_.use_cuda){
                cv::Mat gridMapImage;
                const float min_value = grid_map_.get("elevation").minCoeffOfFinites();
                const float max_value = grid_map_.get("elevation").maxCoeffOfFinites();
                grid_map::GridMapCvConverter::toImage<float, 1>(grid_map_, "elevation", CV_32F, min_value, max_value, gridMapImage);

                cv::cuda::GpuMat gpuGridMapImage;
                gpuGridMapImage.upload(gridMapImage);
                if (gm_process_params_.is_dilate || gm_process_params_.is_erode) {
                    if(gm_process_params_.is_dilate){
                        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(gm_process_params_.dilate_kernel_size, gm_process_params_.dilate_kernel_size));
                        auto dilate_filter = cv::cuda::createMorphologyFilter(cv::MORPH_DILATE, CV_32F, kernel);
                        dilate_filter->apply(gpuGridMapImage, gpuGridMapImage);
                    }

                    if(gm_process_params_.is_erode){
                        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size( gm_process_params_.erode_kernel_size,  gm_process_params_.erode_kernel_size));
                        auto erode_filter = cv::cuda::createMorphologyFilter(cv::MORPH_ERODE, CV_32F, kernel);
                        erode_filter->apply(gpuGridMapImage, gpuGridMapImage);
                    }
                }
                gpuGridMapImage.download(gridMapImage);
                grid_map::GridMapCvConverter::addLayerFromImage<float, 1>(gridMapImage, "elevation", grid_map_, min_value, max_value);

                #ifdef DEBUG_GRIDMAP
                RCLCPP_INFO(node_->get_logger(), "before min_value, max_value: %lf, %lf", min_value, max_value);
                const float after_min_value = grid_map_.get("elevation").minCoeffOfFinites();
                const float after_max_value = grid_map_.get("elevation").maxCoeffOfFinites();
                RCLCPP_INFO(node_->get_logger(), "after min_value, max_value: %lf, %lf", after_min_value, after_max_value);
                #endif
            }
            else{
                cv::Mat gridMapImage;
                const float min_value = grid_map_.get("elevation").minCoeffOfFinites();
                const float max_value = grid_map_.get("elevation").maxCoeffOfFinites();        
                grid_map::GridMapCvConverter::toImage<float, 1>(grid_map_, "elevation", CV_32F, min_value, max_value, gridMapImage);
                if(gm_process_params_.is_dilate){
                    int kernel_size = gm_process_params_.dilate_kernel_size;
                    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
                    cv::dilate(gridMapImage, gridMapImage, kernel);
                }
                if(gm_process_params_.is_erode){
                    int kernel_size = gm_process_params_.erode_kernel_size;
                    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
                    cv::erode(gridMapImage, gridMapImage, kernel); 
                }
                grid_map::GridMapCvConverter::addLayerFromImage<float, 1>(gridMapImage, "elevation", grid_map_, min_value, max_value);

                #ifdef DEBUG_GRIDMAP
                RCLCPP_INFO(node_->get_logger(), "before min_value, max_value: %lf, %lf", min_value, max_value);
                const float after_min_value = grid_map_.get("elevation").minCoeffOfFinites();
                const float after_max_value = grid_map_.get("elevation").maxCoeffOfFinites();
                RCLCPP_INFO(node_->get_logger(), "after min_value, max_value: %lf, %lf", after_min_value, after_max_value);
                #endif
            }

            #ifdef DEBUG_GRIDMAP
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - start);
            RCLCPP_INFO(node_->get_logger(), "Time taken: %d ms", duration.count());
            #endif
        }
    }

    grid_map::GridMap &GetGridMap(){
        return grid_map_;
    }

    rclcpp::Time &GetTimeStamp(){
        return time_stamp_;
    }

    void Clear(){
        grid_map_.clearAll();
    }

    rclcpp::Node::SharedPtr node_;
    GridMapProcessParams gm_process_params_;

    rclcpp::Time time_stamp_;
    grid_map::GridMap grid_map_;
};