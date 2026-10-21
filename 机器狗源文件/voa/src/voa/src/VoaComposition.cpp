
#include "rclcpp/rclcpp.hpp"

#include "PointCloudProcessor.hpp"
#include "GridMapper.hpp"
#include "SafetyController.hpp"

int main(int argc, char **argv){
    rclcpp::init(argc, argv);

    auto intra_comms_options = rclcpp::NodeOptions{}.use_intra_process_comms(true);
    auto point_cloud_processor = std::make_shared<PointCloudProcessor>(intra_comms_options);
    auto grid_mapper = std::make_shared<GridMapper>(intra_comms_options);
    auto safety_controller = std::make_shared<SafetyController>(intra_comms_options);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(point_cloud_processor);
    executor.add_node(grid_mapper);
    executor.add_node(safety_controller);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
