
#include "rclcpp/rclcpp.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <thread>
#include <deque>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

struct SafetyControlParams{
    SafetyControlParams()
    : kMaxOdomBufferLength(0),
        kMaxPredictTimes(0),
        kPredictDtS(0),
        kSafetyDtS(0),
        kRobotLengthM(0),
        kRobotWidthM(0),
        kObstatleHeight(0),
        kP4Vx(0),
        kP4Vyaw(0) {}
    SafetyControlParams(
        int _kMaxOdomBufferLength, 
        int _kMaxPredictTimes, double _kPredictDtS, double _kSafetyDtS, 
        double _kRobotLengthM,
        double _kRobotWidthM,
        double _kObstatleHeight,
        double _kP4Vx, double _kP4Vyaw)
    : kMaxOdomBufferLength(_kMaxOdomBufferLength), 
        kMaxPredictTimes(_kMaxPredictTimes), 
        kPredictDtS(_kPredictDtS), 
        kSafetyDtS(_kSafetyDtS),
        kRobotLengthM(_kRobotLengthM),
        kRobotWidthM(_kRobotWidthM),
        kObstatleHeight(_kObstatleHeight),
        kP4Vx(_kP4Vx), 
        kP4Vyaw(_kP4Vyaw) {}
    int kMaxOdomBufferLength;
    int kMaxPredictTimes;
    double kPredictDtS;
    double kSafetyDtS;
    double kRobotLengthM;
    double kRobotWidthM;
    double kObstatleHeight;
    double kP4Vx;
    double kP4Vyaw;    
};

enum class RobotState : int{
    Slow = 1,
    Fast = 2
};

struct RobotVel{
public:
    RobotVel()
    : RobotVel(0,0,0) {}
    RobotVel(double _x, double _y, double _yaw)
    : x(_x), y(_y), yaw(_yaw) {}
    void Clear(){
        x = 0;
        y = 0;
        yaw = 0;
    }
    void Add(const RobotVel &vel){
        x += vel.x;
        y += vel.y;
        yaw += vel.yaw;
    }
    void Average(int i){
        x /= i;
        y /= i;
        yaw /= i;
    }
    double x;
    double y;
    double yaw;
};

struct RobotPose{
public:
    RobotPose()
    : RobotPose(0,0,0) {}
    RobotPose(double _x, double _y, double _yaw)
    : x(_x), y(_y), yaw(_yaw) {}
    double x;
    double y;
    double yaw;
};

enum class DIRECTION : int{
    left = 1,
    right = 2
};

struct AdjustRouteParams{
    int collision_time;
    DIRECTION direction;
};

struct Point{
public:
    Point()
    : Point(0,0) {};
    Point(double _x, double _y)
    : x(_x), y(_y) {}
    Point(const Point &_point)
    : x(_point.x), y(_point.y) {}
    double x;
    double y;
};

struct RobotCollisionRect{
public:
    RobotCollisionRect(Point _point0, Point _point1, Point _point2, Point _point3)
    : point0(_point0), point1(_point1), point2(_point2), point3(_point3) {}

    bool IsInsideRect(Point point){
        cv::Point2f point2check(point.x, point.y);
        std::vector<cv::Point2f> rect(4);
        rect[0] = cv::Point2f(point0.x, point0.y);
        rect[1] = cv::Point2f(point1.x, point1.y);
        rect[2] = cv::Point2f(point2.x, point2.y);
        rect[3] = cv::Point2f(point3.x, point3.y);
        return pointPolygonTest(rect, point2check, false);
    }

    Point point0;
    Point point1;
    Point point2;
    Point point3;
};

constexpr double kUltrasoudUpperThreshold = 1.5;
constexpr double kUltrasoudLowerThreshold = 0.6;
constexpr double kBackwardVelMax = 0.6;
constexpr double kBackwardVelMin = 0.1;

constexpr double kRobotHeightM = 0.6;

constexpr double kMaxVx = 0.5;
constexpr double kMaxVyaw = 1.2;

class SafetyController : public rclcpp::Node{
public:
    SafetyController(rclcpp::NodeOptions options)
    : Node("safety_controller", options){
        InitParams();

        new_grid_map_.store(false);
        new_odom_.store(false);
        new_ultrasoud_.store(false);
        new_vel_in_.store(false);

        using std::placeholders::_1;
        grid_map_sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>("/grid_map", 10, std::bind(&SafetyController::GridMapCallback, this, _1));
        ultrasound_sub_ = this->create_subscription<std_msgs::msg::Float64>("/us_publisher/ultrasound_distance", 10, std::bind(&SafetyController::UltrasoundCallback, this, _1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/leg_odom2", 10, std::bind(&SafetyController::OdomCallback, this, _1));
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/marker", 10);

        rclcpp::QoS best_effort_qos(1);
        best_effort_qos.best_effort();
        handle_state_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/handle_state", best_effort_qos, std::bind(&SafetyController::HandleStateCallback, this, _1));
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", best_effort_qos, std::bind(&SafetyController::CmdVelCallback, this, _1));
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_corrected", best_effort_qos);
        
        correct_velocity_thread_ = std::thread(&SafetyController::CorrectVelocityThreadFunc, this);

        RCLCPP_INFO(this->get_logger(), "SafetyController initialized!");
    }

private:
    void InitParams(){
        declare_parameter("timeout_s_sc", 0.2);
        if(get_parameter("timeout_s_sc", timeout_s_)){
            RCLCPP_INFO(get_logger(), "timeout_s_sc is: %lf", timeout_s_);
        }
        else    RCLCPP_WARN(get_logger(), "get_parameter timeout_s_sc failed");

        declare_parameter("kMaxOdomBufferLength", 10);
        if(get_parameter("kMaxOdomBufferLength", safety_control_params_.kMaxOdomBufferLength)){
            RCLCPP_INFO(get_logger(), "kMaxOdomBufferLength is: %d", safety_control_params_.kMaxOdomBufferLength);
        }
        else    RCLCPP_WARN(get_logger(), "get_parameter kMaxOdomBufferLength failed");

        declare_parameter("kMaxPredictTimes", 10);
        if(get_parameter("kMaxPredictTimes", safety_control_params_.kMaxPredictTimes)){
            RCLCPP_INFO(get_logger(), "kMaxPredictTimes is: %d", safety_control_params_.kMaxPredictTimes);
        }
        else    RCLCPP_WARN(get_logger(), "get_parameter kMaxPredictTimes failed");

        declare_parameter("kPredictDtS", 0.15);
        if(get_parameter("kPredictDtS", safety_control_params_.kPredictDtS)){
            RCLCPP_INFO(get_logger(), "kPredictDtS is: %lf", safety_control_params_.kPredictDtS);
        }
        else    RCLCPP_WARN(get_logger(), "get_parameter kPredictDtS failed");

        declare_parameter("kSafetyDtS", 1.0);
        if(get_parameter("kSafetyDtS", safety_control_params_.kSafetyDtS)){
            RCLCPP_INFO(get_logger(), "kSafetyDtS is: %lf", safety_control_params_.kSafetyDtS);
        }
        else    RCLCPP_WARN(get_logger(), "get_parameter kSafetyDtS failed");

        declare_parameter("kRobotLengthM", 0.8);
        if(get_parameter("kRobotLengthM", safety_control_params_.kRobotLengthM)){
            RCLCPP_INFO(get_logger(), "kRobotLengthM is: %lf", safety_control_params_.kRobotLengthM);
        }
        else    RCLCPP_WARN(get_logger(), "get_parameter kRobotLengthM failed");

        declare_parameter("kRobotWidthM", 0.6);
        if(get_parameter("kRobotWidthM", safety_control_params_.kRobotWidthM)){
            RCLCPP_INFO(get_logger(), "kRobotWidthM is: %lf", safety_control_params_.kRobotWidthM);
        }
        else    RCLCPP_WARN(get_logger(), "get_parameter kRobotWidthM failed");

        declare_parameter("kObstatleHeight", 0.20);
        if(get_parameter("kObstatleHeight", safety_control_params_.kObstatleHeight)){
            RCLCPP_INFO(get_logger(), "kObstatleHeight is: %lf", safety_control_params_.kObstatleHeight);
        }
        else    RCLCPP_WARN(get_logger(), "get_parameter kObstatleHeight failed");

        declare_parameter("kP4Vx", 0.1);
        if(get_parameter("kP4Vx", safety_control_params_.kP4Vx)){
            RCLCPP_INFO(get_logger(), "kP4Vx is: %lf", safety_control_params_.kP4Vx);
        }
        else    RCLCPP_WARN(get_logger(), "get_parameter kP4Vx failed");

        declare_parameter("kP4Vyaw", 0.1);
        if(get_parameter("kP4Vyaw", safety_control_params_.kP4Vyaw)){
            RCLCPP_INFO(get_logger(), "kP4Vyaw is: %lf", safety_control_params_.kP4Vyaw);
        }
        else    RCLCPP_WARN(get_logger(), "get_parameter kP4Vyaw failed");
    }

    void SwitchRobotState(nav_msgs::msg::Odometry::SharedPtr msg){
        if(std::abs(msg->twist.twist.linear.x) < 0.05
        && std::abs(msg->twist.twist.linear.y) < 0.05
        && std::abs(msg->twist.twist.angular.z) < 0.05){
            if(robot_state_ != RobotState::Slow){
                #ifdef DEBUG_MOTION
                RCLCPP_INFO(this->get_logger(), "robot_state_ switch to Slow");
                #endif
                robot_state_ = RobotState::Slow;
            }
        }
        else{
            if(robot_state_ != RobotState::Fast){
                #ifdef DEBUG_MOTION
                RCLCPP_INFO(this->get_logger(), "robot_state_ switch to Fast");
                #endif
                robot_state_ = RobotState::Fast;
            }
        }
    }

    void OdomCallback(nav_msgs::msg::Odometry::SharedPtr msg){    
        auto &stamp_ref_odom = msg->header.stamp;
        double t_diff_s = std::abs(((double)stamp_ref_odom.sec + (double)stamp_ref_odom.nanosec/1e9) - rclcpp::Clock(RCL_STEADY_TIME).now().seconds());
        if(t_diff_s > timeout_s_){
            RCLCPP_WARN(this->get_logger(), "Odom timeout for %lfs", t_diff_s);
            return;
        }

        SwitchRobotState(msg);

        odom_buffer_.emplace_back(RobotVel(
            msg->twist.twist.linear.x, 
            msg->twist.twist.linear.y, 
            msg->twist.twist.angular.z
        ));
        if(odom_buffer_.size() > safety_control_params_.kMaxOdomBufferLength){
            odom_buffer_.pop_front();
        }

        {
            std::lock_guard<std::mutex> lock(vel_predict_mutex_);
            vel_predict_.Clear();
            for(int i = 0; i < odom_buffer_.size(); i++){
                vel_predict_.Add(odom_buffer_.at(i));
            }
            vel_predict_.Average(odom_buffer_.size());            
        }
        new_odom_ = true;
        if(!first_odom_)    first_odom_ = true;
    }

    void GridMapCallback(grid_map_msgs::msg::GridMap::UniquePtr msg){    
        auto &stamp_ref_grid = msg->header.stamp;
        double t_diff_s = std::abs(((double)stamp_ref_grid.sec + (double)stamp_ref_grid.nanosec/1e9) - rclcpp::Clock(RCL_STEADY_TIME).now().seconds());
        if(t_diff_s > timeout_s_){
            RCLCPP_WARN(this->get_logger(), "GridMap timeout for %lfs", t_diff_s);
            return;
        }

        {
            std::lock_guard<std::mutex> lock(grid_map_mutex_);
            grid_map_ = std::move(msg);
            new_grid_map_.store(true);
        }        
        if(!first_grid_map_)    first_grid_map_ = true;        
    }

    void UltrasoundCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(ultrasound_mutex_);
            ultrasound_ = msg;
            new_ultrasoud_.store(true);
        }        
        if(!first_ultrasound_)    first_ultrasound_ = true;        
    }

    void HandleStateCallback(geometry_msgs::msg::Twist::SharedPtr msg){
        {
            std::lock_guard<std::mutex> lock(vel_in_mutex_);
            vel_in_ = msg;
            new_vel_in_.store(true);
        }        
        if(!first_vel_in_)    first_vel_in_ = true;           
    }

    void CmdVelCallback(geometry_msgs::msg::Twist::SharedPtr msg){
        {
            std::lock_guard<std::mutex> lock(vel_in_mutex_);
            vel_in_ = msg;
            new_vel_in_.store(true);
        }
        if(!first_vel_in_)    first_vel_in_ = true;                 
    }

    void CorrectVelocityThreadFunc(){
        while(rclcpp::ok()){
            if(!first_odom_ || !first_ultrasound_|| !first_grid_map_ || !first_vel_in_){
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }

            if(new_grid_map_.load() || new_odom_.load() || new_ultrasoud_.load() && new_vel_in_.load()){
                grid_map_msgs::msg::GridMap::SharedPtr grid_map;
                std_msgs::msg::Float64::SharedPtr ultrasound;
                RobotVel vel_predict;
                geometry_msgs::msg::Twist::SharedPtr vel_in;
                geometry_msgs::msg::Twist vel_out;
                CopyData(grid_map, ultrasound, vel_predict, vel_in);
                if(vel_in->linear.x == 0.0 && vel_in->linear.y == 0.0 && vel_in->angular.z == 0.0){
                    NoCorrectVelocity(vel_out);
                }                
                if(vel_in->linear.x > 0){
                    CorrectForwardVelocity(grid_map, vel_predict, vel_in, vel_out);                    
                }
                else{
                    CorrectBackwardVelocity(ultrasound, vel_predict, vel_in, vel_out);
                }

                #ifdef DEBUG_MOTION
                RCLCPP_INFO(this->get_logger(), "cmd_vel_pub_ vel_out.linear.x: %lf, vel_out.angular.z: %lf", vel_out.linear.x, vel_out.angular.z);
                #endif                
                cmd_vel_pub_->publish(vel_out);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    void CopyData(grid_map_msgs::msg::GridMap::SharedPtr &grid_map, std_msgs::msg::Float64::SharedPtr &ultrasound, RobotVel &vel_predict, geometry_msgs::msg::Twist::SharedPtr &vel_in){
        {        
            std::lock_guard<std::mutex> lock(grid_map_mutex_);
            grid_map = grid_map_;
            new_grid_map_.store(false);
        }

        {
            std::lock_guard<std::mutex> lock(ultrasound_mutex_);
            ultrasound = ultrasound_;
            new_ultrasoud_.store(false);
        }

        {
            std::lock_guard<std::mutex> lock(vel_predict_mutex_);
            vel_predict = vel_predict_;
            new_odom_.store(false);
        }

        {
            std::lock_guard<std::mutex> lock(vel_in_mutex_);
            vel_in = vel_in_;
            new_vel_in_.store(false);
        }
    }

    void UpdateRobotPose(RobotPose& pose, const RobotVel& vel, double dt){
        double yaw_last = pose.yaw;
        pose.yaw += vel.yaw * dt;

        Eigen::MatrixXd rotation_matrix_last(2, 2);
        double cos_yaw_last = cos(yaw_last);
        double sin_yaw_last = sin(yaw_last);
        rotation_matrix_last << cos_yaw_last, sin_yaw_last,
                                -sin_yaw_last, cos_yaw_last;
        Eigen::MatrixXd rotation_matrix_now(2, 2);
        double cos_yaw_now = cos(pose.yaw);
        double sin_yaw_now = sin(pose.yaw);
        rotation_matrix_now <<  cos_yaw_now, sin_yaw_now,
                                -sin_yaw_now, cos_yaw_now;
        
        Eigen::MatrixXd v_robot(1, 2);
        v_robot <<  vel.x, vel.y;
        
        Eigen::MatrixXd dp_world_last(1, 2);
        dp_world_last = v_robot * rotation_matrix_last * dt;        
        Eigen::MatrixXd dp_world_now(1, 2);
        dp_world_now << v_robot * rotation_matrix_now * dt;
        
        pose.x += 0.5 * (dp_world_last(0) + dp_world_now(0));
        pose.y += 0.5 * (dp_world_last(1) + dp_world_now(1));

        #ifdef DEBUG_MARKER
        std::cout << "yaw_last: " << yaw_last << " yaw_now: " << pose.yaw << std::endl;
        std::cout << "dt: " << dt << std::endl;
        std::cout << "rotation_matrix_last" << rotation_matrix_last << std::endl;
        std::cout << "rotation_matrix_now" << rotation_matrix_now << std::endl;
        std::cout << "v_robot" << v_robot << std::endl;
        std::cout << "dp_world_last" << dp_world_last << std::endl;
        std::cout << "dp_world_now" << dp_world_now << std::endl;
        std::cout << "pose.x: " << pose.x << " pose.y: " << pose.y << std::endl;
        #endif
    }

    void Add2RobotCollisionPath(RobotPose &pose, std::vector<RobotCollisionRect> &robot_collision_path){
        Point point0, point1, point2, point3;
        double sin_yaw = sin(pose.yaw);
        double cos_yaw = cos(pose.yaw);
        double dx_0_2 = (safety_control_params_.kRobotLengthM/2*cos_yaw - safety_control_params_.kRobotWidthM/2*sin_yaw);
        double dy_0_2 = (safety_control_params_.kRobotLengthM/2*sin_yaw + safety_control_params_.kRobotWidthM/2*cos_yaw);
        point0.x = dx_0_2 + pose.x;
        point0.y = dy_0_2 + pose.y;
        point2.x = -dx_0_2 + pose.x;
        point2.y = -dy_0_2 + pose.y;
        double dx_1_3 = (safety_control_params_.kRobotLengthM/2*cos_yaw + safety_control_params_.kRobotWidthM/2*sin_yaw);
        double dy_1_3 = (safety_control_params_.kRobotLengthM/2*sin_yaw - safety_control_params_.kRobotWidthM/2*cos_yaw);
        point1.x = -dx_1_3 + pose.x;
        point1.y = -dy_1_3 + pose.y;
        point3.x = dx_1_3 + pose.x;
        point3.y = dy_1_3 + pose.y;

        robot_collision_path.emplace_back(RobotCollisionRect(point0, point1, point2, point3));

        #ifdef DEBUG_MARKER
        std::cout << "sin_yaw: " << sin_yaw << " cos_yaw: " << cos_yaw << std::endl;
        std::cout << "dx_0_2: " << dx_0_2 << " dy_0_2: " << dy_0_2 << std::endl;
        std::cout << "dx_1_3: " << dx_1_3 << " dy_1_3: " << dy_1_3 << std::endl;
        std::cout << "point0.x: " << point0.x << " point0.y: " << point0.y << std::endl;
        std::cout << "point1.x: " << point1.x << " point1.y: " << point1.y << std::endl;
        std::cout << "point2.x: " << point2.x << " point2.y: " << point2.y << std::endl;
        std::cout << "point3.x: " << point3.x << " point3.y: " << point3.y << std::endl;
        #endif
    }

    void MakeMarkers(std::vector<RobotCollisionRect> &robot_collision_path, visualization_msgs::msg::MarkerArray &markers){
        for(int i = 0; i < robot_collision_path.size(); i++){
            //init the mark
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "robot_foot_print";
            marker.header.stamp = rclcpp::Clock(RCL_STEADY_TIME).now();
            marker.ns = "base_marker_" + std::to_string(i);
            marker.type = visualization_msgs::msg::Marker::LINE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = 0;
            marker.pose.position.y = 0;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.01;
            marker.scale.y = 0.01;
            marker.color.r = 1.0;
            marker.color.a = 1.0;
            //todo
            marker.lifetime = rclcpp::Duration(0, 500000);

            marker.points.clear();
            geometry_msgs::msg::Point point0, point1, point2, point3;
            point0.x = robot_collision_path.at(i).point0.x;       
            point0.y = robot_collision_path.at(i).point0.y;
            point1.x = robot_collision_path.at(i).point1.x;
            point1.y = robot_collision_path.at(i).point1.y;
            point2.x = robot_collision_path.at(i).point2.x;
            point2.y = robot_collision_path.at(i).point2.y;
            point3.x = robot_collision_path.at(i).point3.x;
            point3.y = robot_collision_path.at(i).point3.y;
            marker.points.push_back(point0);
            marker.points.push_back(point1);
            marker.points.push_back(point1);
            marker.points.push_back(point2);
            marker.points.push_back(point2);
            marker.points.push_back(point3);
            marker.points.push_back(point3);
            marker.points.push_back(point0);

            //add marker to markers
            markers.markers.emplace_back(marker);
        }
    }

    void PredictCollisionPath(grid_map_msgs::msg::GridMap::SharedPtr &grid_map, RobotVel &vel_predict, std::vector<RobotCollisionRect> &robot_collision_path){
        RobotPose robot_pose(0.0, 0.0, 0.0);
        auto &stamp_ref_grid = grid_map->header.stamp;
        double t_stamp_s = (double)stamp_ref_grid.sec + (double)stamp_ref_grid.nanosec/1e9;
        UpdateRobotPose(robot_pose, vel_predict, safety_control_params_.kSafetyDtS);
        Add2RobotCollisionPath(robot_pose, robot_collision_path);
        for(int i = 0; i < safety_control_params_.kMaxPredictTimes-1; i++){
            UpdateRobotPose(robot_pose, vel_predict, safety_control_params_.kPredictDtS);
            Add2RobotCollisionPath(robot_pose, robot_collision_path);
        }
    }

    bool DetectCollision(grid_map_msgs::msg::GridMap::SharedPtr &grid_map_msg, std::vector<RobotCollisionRect> &robot_collision_path, geometry_msgs::msg::Twist::SharedPtr &vel_in, AdjustRouteParams &params){
        grid_map::GridMap grid_map;
        grid_map::GridMapRosConverter::fromMessage(*grid_map_msg, grid_map);
        for(int i=0; i<robot_collision_path.size(); i++){
            RobotCollisionRect &collision_rect = robot_collision_path.at(i);
            double x_min = std::min({collision_rect.point0.x, collision_rect.point1.x, collision_rect.point2.x, collision_rect.point3.x});
            double x_max = std::max({collision_rect.point0.x, collision_rect.point1.x, collision_rect.point2.x, collision_rect.point3.x});
            double y_min = std::min({collision_rect.point0.y, collision_rect.point1.y, collision_rect.point2.y, collision_rect.point3.y});
            double y_max = std::max({collision_rect.point0.y, collision_rect.point1.y, collision_rect.point2.y, collision_rect.point3.y});

            //从近到远
            for(double x = x_min; x <= x_max; x += grid_map.getResolution()){
                bool found = false;
                double y_left = 0.0;
                double y_right = 0.0;

                //从左到右
                for(double y = y_min; y <= y_max; y += grid_map.getResolution()){
                    Point point(x,y);
                    if(collision_rect.IsInsideRect(point)){
                        grid_map::Position position(x, y);
                        if(grid_map.isInside(position)){
                            double height = grid_map.atPosition("elevation", position);
                            #ifdef DEBUG_MARKER
                            RCLCPP_INFO(this->get_logger(), "point is in the rect");
                            RCLCPP_INFO(this->get_logger(), "height at %lf,%lf for grid_map is: %lf", x, y, height);
                            #endif
                            if(height > safety_control_params_.kObstatleHeight){
                                found = true;
                                y_left = y;
                            }
                        }
                        else{
                            continue;
                        }
                    }
                }

                //从右到左
                if(found){
                    for(double y = y_max; y >= y_min; y -= grid_map.getResolution()){
                        Point point(x,y);
                            if(collision_rect.IsInsideRect(point)){
                            grid_map::Position position(x, y);
                            if(grid_map.isInside(position)){
                                double height = grid_map.atPosition("elevation", position);
                                #ifdef DEBUG_MARKER
                                RCLCPP_INFO(this->get_logger(), "point is in the rect");
                                RCLCPP_INFO(this->get_logger(), "height at %lf,%lf for grid_map is: %lf", x, y, height);
                                #endif
                                if(height > safety_control_params_.kObstatleHeight){
                                    y_right = y;
                                }
                            }
                            else{
                                continue;
                            }
                        }
                    }

                    params.collision_time = i;
                    Point median_point(x, (y_left+y_right)/2);
                    #ifdef DEBUG_MOTION
                    RCLCPP_INFO(this->get_logger(), "median_point: %lf, %lf", x, (y_left+y_right)/2);
                    #endif
                    if(median_point.y > 0){
                        #ifdef DEBUG_MOTION
                        RCLCPP_INFO(this->get_logger(), "DIRECTION::left");
                        #endif
                        params.direction = DIRECTION::left;
                    }
                    else{
                        #ifdef DEBUG_MOTION
                        RCLCPP_INFO(this->get_logger(), "DIRECTION::right");
                        #endif
                        params.direction = DIRECTION::right;                                
                    }
                    return true;
                }
            }
        }
        return false;
    }

    void CorrectForwardVelocity(grid_map_msgs::msg::GridMap::SharedPtr &grid_map, RobotVel &vel_predict, geometry_msgs::msg::Twist::SharedPtr &vel_in, geometry_msgs::msg::Twist &vel_out){
        std::vector<RobotCollisionRect> original_collision_path;
        if(robot_state_ == RobotState::Fast){
            PredictCollisionPath(grid_map, vel_predict, original_collision_path);
        }
        else{
            #ifdef DEBUG_MOTION
            RCLCPP_INFO(this->get_logger(), "robot start predict");
            #endif
            RobotVel vel_control(vel_in->linear.x, vel_in->linear.y, vel_in->angular.z);
            PredictCollisionPath(grid_map, vel_control, original_collision_path);
        }
        AdjustRouteParams adjust_route_params;
        std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
        bool is_collision = DetectCollision(grid_map, original_collision_path, vel_in, adjust_route_params);
        std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
        double t_diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        #ifdef DEBUG_MOTION
        RCLCPP_INFO(this->get_logger(), "t_diff_ms for DetectCollision() is: %lf", t_diff_ms);
        #endif
        if(is_collision){
            #ifdef DEBUG_MOTION
            RCLCPP_INFO(this->get_logger(), "into is_collision");
            RCLCPP_INFO(this->get_logger(), "vel_in is: %lf, %lf, %lf", vel_in->linear.x, vel_in->linear.y, vel_in->angular.z);
            RCLCPP_INFO(this->get_logger(), "adjust_route_params is: %d, %d", 
                adjust_route_params.collision_time, 
                static_cast<int>(adjust_route_params.direction));
            #endif
            if(adjust_route_params.collision_time > 0){
                std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

                bool found_no_collision = false;
                
                double no_collision_best_score = 9999999;
                RobotVel no_collision_best_vel;

                double collision_best_score = 999999;
                RobotVel collision_best_vel;

                std::vector<RobotCollisionRect> best_path;

                for(int i=0; i<5; i++){
                    for(int j=-5; j<6; j++){
                        double vx = 0.20*i;
                        double vz = 0.10*j;
                        if(vx==0.0 && vz==0.0){
                            continue;
                        }

                        RobotVel test_vel(vx, 0, vz);
                        std::vector<RobotCollisionRect> test_path;
                        PredictCollisionPath(grid_map, test_vel, test_path);

                        AdjustRouteParams test_route_params;
                        bool test_is_collision = DetectCollision(grid_map, test_path, vel_in, test_route_params);

                        if(!test_is_collision){
                            found_no_collision = true;
                            double score = abs(vx - vel_in->linear.x)*abs(vx - vel_in->linear.x) + abs(vz - vel_in->angular.z)*abs(vz - vel_in->angular.z);
                            if(score < no_collision_best_score){
                                no_collision_best_score = score;
                                #ifdef DEBUG_MOTION
                                RCLCPP_INFO(this->get_logger(), "before test_vel.yaw: %lf", test_vel.yaw);
                                #endif
                                if(test_vel.yaw < 0)        test_vel.yaw -= 0.1;
                                else if(test_vel.yaw > 0)   test_vel.yaw += 0.1;
                                no_collision_best_vel = test_vel;

                                best_path = test_path;

                                #ifdef DEBUG_MOTION
                                RCLCPP_INFO(this->get_logger(), "after test_vel.yaw: %lf, no_collision_best_vel.yaw: %lf", test_vel.yaw, no_collision_best_vel.yaw);
                                #endif
                            }
                        }
                        else{
                            double score = test_route_params.collision_time;
                            if(score < collision_best_score){
                                collision_best_score = score;
                                #ifdef DEBUG_MOTION
                                RCLCPP_INFO(this->get_logger(), "before test_vel.yaw: %lf", test_vel.yaw);
                                #endif
                                if(test_vel.yaw < 0)        test_vel.yaw -= 0.1;
                                else if(test_vel.yaw > 0)   test_vel.yaw += 0.1;
                                collision_best_vel = test_vel;

                                #ifdef DEBUG_MOTION
                                RCLCPP_INFO(this->get_logger(), "after test_vel.yaw: %lf, collision_best_vel.yaw: %lf", test_vel.yaw, collision_best_vel.yaw);
                                #endif

                                best_path = test_path;
                            }
                        }
                    }
                }

                std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
                double t_diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
                #ifdef DEBUG_MOTION
                RCLCPP_INFO(this->get_logger(), "t_diff_ms for loop() is: %lf", t_diff_ms);
                #endif

                if(found_no_collision){
                    #ifdef DEBUG_MOTION
                    RCLCPP_INFO(this->get_logger(), "no_collision_best_score: %lf, no_collision_best_vel.x: %lf, no_collision_best_vel.yaw: %lf", no_collision_best_score, no_collision_best_vel.x, no_collision_best_vel.yaw);
                    #endif
                    vel_out.linear.x = no_collision_best_vel.x;
                    vel_out.linear.y = 0.0;
                    vel_out.angular.z = no_collision_best_vel.yaw;
                }
                else{
                    #ifdef DEBUG_MOTION
                    RCLCPP_INFO(this->get_logger(), "collision_best_score: %lf, collision_best_vel.x: %lf, collision_best_vel.yaw: %lf", collision_best_score, collision_best_vel.x, collision_best_vel.yaw);
                    #endif
                    vel_out.linear.x = collision_best_vel.x;
                    vel_out.linear.y = 0;
                    vel_out.angular.z = collision_best_vel.yaw;
                }

                visualization_msgs::msg::MarkerArray markers;
                MakeMarkers(best_path, markers);
                marker_pub_->publish(markers);
            }
            else{
                #ifdef DEBUG_MOTION
                RCLCPP_INFO(this->get_logger(), "immediate collision, stop!");
                #endif
                vel_out.linear.x = 0;
                vel_out.linear.y = 0;
                vel_out.angular.z = 0;
            }

            #ifdef DEBUG_MOTION
            RCLCPP_INFO(this->get_logger(), "vel_out is: %lf, %lf, %lf", vel_out.linear.x, vel_out.linear.y, vel_out.angular.z);
            #endif
        }
        else{
            vel_out.linear.x = vel_in->linear.x;
            vel_out.linear.y = vel_in->linear.y;
            vel_out.angular.z = vel_in->angular.z;
        }
    }

    void CorrectBackwardVelocity(std_msgs::msg::Float64::SharedPtr &ultrasound, RobotVel &vel_predict, geometry_msgs::msg::Twist::SharedPtr &vel_in, geometry_msgs::msg::Twist &vel_out){
        if(ultrasound->data > kUltrasoudUpperThreshold){
            vel_out.linear.x = vel_in->linear.x * kBackwardVelMax;
        }
        else if(ultrasound->data > kUltrasoudLowerThreshold){
            vel_out.linear.x = vel_in->linear.x * kBackwardVelMax * (ultrasound->data - kUltrasoudLowerThreshold)/(kUltrasoudUpperThreshold - kUltrasoudLowerThreshold);
            vel_out.linear.x = std::min(vel_out.linear.x, kBackwardVelMin);
        }
        else{
            vel_out.linear.x = 0;
        }
        vel_out.linear.y = vel_in->linear.y;
        vel_out.angular.z = 0.7 * vel_in->angular.z;
    }

    void NoCorrectVelocity(geometry_msgs::msg::Twist &vel_out){
        vel_out.linear.x = 0.0;
        vel_out.linear.y = 0.0;
        vel_out.angular.z = 0.0;
    }

private:
    double timeout_s_;

    RobotState robot_state_ = RobotState::Slow;

    std::mutex grid_map_mutex_;
    grid_map_msgs::msg::GridMap::SharedPtr grid_map_;  
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_sub_;
    std::atomic<bool> new_grid_map_;
    bool first_grid_map_ = false;

    std::mutex ultrasound_mutex_;
    std_msgs::msg::Float64::SharedPtr ultrasound_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ultrasound_sub_;
    std::atomic<bool> new_ultrasoud_;
    bool first_ultrasound_ = false;

    std::mutex vel_predict_mutex_;
    RobotVel vel_predict_;
    std::deque<RobotVel> odom_buffer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::atomic<bool> new_odom_;
    bool first_odom_ = false;

    std::mutex vel_in_mutex_;
    geometry_msgs::msg::Twist::SharedPtr vel_in_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr handle_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::atomic<bool> new_vel_in_;
    bool first_vel_in_ = false;

    std::thread correct_velocity_thread_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    SafetyControlParams safety_control_params_;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(SafetyController)

