#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "MotionControl.hpp"

using namespace std::chrono_literals; // DOPLNĚNO: Pro práci s časem (1s)

MotionControlNode::MotionControlNode() :
    rclcpp::Node("motion_control_node") {

        // Subscribers for odometry and laser scans
        // add code here
        // Posloucháme tvůj uzel Localization na tématu "odometry"
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry", 10, std::bind(&MotionControlNode::odomCallback, this, std::placeholders::_1));
            
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(), std::bind(&MotionControlNode::lidarCallback, this, std::placeholders::_1));
        
        // Publisher for robot control
        // add code here
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Client for path planning
        // add code here
        plan_client_ = this->create_client<nav_msgs::srv::GetPlan>("plan_path");

        // Action server
        // add code here
        nav_server_ = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(
            this,
            "go_to_goal",
            std::bind(&MotionControlNode::navHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MotionControlNode::navHandleCancel, this, std::placeholders::_1),
            std::bind(&MotionControlNode::navHandleAccepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(get_logger(), "Motion control node started.");

        // Connect to path planning service server
        // add code here
        while (!plan_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the planning service.");
                return;
            }
            RCLCPP_INFO(get_logger(), "Waiting for planning service to appear...");
        }
    }

void MotionControlNode::checkCollision() {
    // add code here
    if (laser_scan_.ranges.empty()) return;

    collision_detected_ = false;

    int center_idx = laser_scan_.ranges.size() / 2;
    int check_range = 12; // Úzký výřez před robotem
    float thresh = 0.30;  // Stop vzdálenost 30cm

    for (int i = center_idx - check_range; i <= center_idx + check_range; i++) {
        if (i >= 0 && i < (int)laser_scan_.ranges.size()) {
            // Ignorujeme tělo robota a hlídáme překážky
            if (laser_scan_.ranges[i] < thresh && laser_scan_.ranges[i] > 0.18) { 
                geometry_msgs::msg::Twist stop; 
                twist_publisher_->publish(stop);

                collision_detected_ = true;

                RCLCPP_ERROR(get_logger(), "COLLISION DETECTED! EMERGENCY STOP!");
                
                if (goal_handle_ && goal_handle_->is_active()) {
                    auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
                    goal_handle_->abort(result);
                }
                return; 
            }
        }
    }

    // ********
    // * Help *
    // ********
    /*
    if (laser_scan_.ranges[i] < thresh) {
        geometry_msgs::msg::Twist stop;
        twist_publisher_->publish(stop);
    }
    */
}

void MotionControlNode::updateTwist() {
    // add code here

    if (collision_detected_) {
        geometry_msgs::msg::Twist stop;
        twist_publisher_->publish(stop);
        return;
    }

    if (path_.poses.empty() || !goal_handle_ || !goal_handle_->is_active()) {
        geometry_msgs::msg::Twist stop;
        twist_publisher_->publish(stop);
        return;
    }

    double curr_x = current_pose_.pose.position.x;
    double curr_y = current_pose_.pose.position.y;
    
    tf2::Quaternion q(
        current_pose_.pose.orientation.x,
        current_pose_.pose.orientation.y,
        current_pose_.pose.orientation.z,
        current_pose_.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, curr_yaw;
    m.getRPY(roll, pitch, curr_yaw);

    // Lookahead vzdálenost 0.4m 
    double lookahead_dist = 0.4; 
    int target_idx = -1;

    // najdi nejbližší bod na trajektorii
    double min_dist = std::numeric_limits<double>::max();
    int closest_idx = 0;
    for (size_t i = 0; i < path_.poses.size(); i++) {
        double dist = std::hypot(path_.poses[i].pose.position.x - curr_x, path_.poses[i].pose.position.y - curr_y);
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }

    // hledej lookahead od nejbližšího bodu (ne od začátku)
    for (size_t i = closest_idx; i < path_.poses.size(); i++) {
        double dist = std::hypot(path_.poses[i].pose.position.x - curr_x, path_.poses[i].pose.position.y - curr_y);
        if (dist > lookahead_dist) {
            target_idx = i;
            break;
        }
    }

    if (target_idx == -1) target_idx = path_.poses.size() - 1;

    double target_x = path_.poses[target_idx].pose.position.x;
    double target_y = path_.poses[target_idx].pose.position.y;

    double target_yaw = std::atan2(target_y - curr_y, target_x - curr_x);
    double error_yaw = std::atan2(std::sin(target_yaw - curr_yaw), std::cos(target_yaw - curr_yaw));

    geometry_msgs::msg::Twist twist;
    
    double P_rot = 0.8; 
    twist.angular.z = P_rot * error_yaw;

    // Dopředná rychlost dle zadání (bezpečných 0.15 m/s)
    double v_max = 0.15; 
    
    // LIDAR CONTROL: Dynamické zpomalování před překážkou
    double speed_scaler = 1.0;
    if (!laser_scan_.ranges.empty()) {
        int center = laser_scan_.ranges.size() / 2;
        float min_front_dist = 2.0; // inicializace na "daleko"
        
        // Sledujeme úzký kužel (cca 20°) přímo před robotem
        for (int i = center - 15; i <= center + 15; i++) {
            if (i >= 0 && i < (int)laser_scan_.ranges.size()) {
                float d = laser_scan_.ranges[i];
                if (d > 0.18 && d < min_front_dist) min_front_dist = d;
            }
        }
        
        // Pokud je překážka blíž než 0.8m, začneme plynule zpomalovat k 0.3m
        if (min_front_dist < 0.8) {
            speed_scaler = (min_front_dist - 0.30) / (0.8 - 0.30);
            if (speed_scaler < 0.0) speed_scaler = 0.0;
        }
    }

    // plynulá rychlost místo zastavení (kombinace chyby úhlu a lidaru)
    twist.linear.x = v_max * std::exp(-2.0 * std::abs(error_yaw)) * speed_scaler;

    // Limitace pro plynulost pohybu
    if (twist.angular.z > 0.6) twist.angular.z = 0.6;
    if (twist.angular.z < -0.6) twist.angular.z = -0.6;
    
    twist_publisher_->publish(twist);

    // ********
    // * Help *
    // ********
    /*
    geometry_msgs::msg::Twist twist;
    twist.angular.z = P * xte;
    twist.linear.x = v_max;

    twist_publisher_->publish(twist);
    */
}

rclcpp_action::GoalResponse MotionControlNode::navHandleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal) {
    // add code here
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionControlNode::navHandleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    // add code here
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionControlNode::navHandleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    // add code here
    goal_handle_ = goal_handle;

    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
    request->start.header.frame_id = "map";
    request->start.pose = current_pose_.pose;
    request->goal = goal_handle->get_goal()->pose;

    auto future = plan_client_->async_send_request(request,
        std::bind(&MotionControlNode::pathCallback, this, std::placeholders::_1));
}

void MotionControlNode::execute() {
    // add code here
    rclcpp::Rate loop_rate(10.0);
    auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();

    while (rclcpp::ok() && goal_handle_ && goal_handle_->is_active()) {
        if (goal_handle_->is_canceling()) {
            goal_handle_->canceled(result);
            return;
        }

        if (!path_.poses.empty()) {
            double dist = std::hypot(path_.poses.back().pose.position.x - current_pose_.pose.position.x, 
                                     path_.poses.back().pose.position.y - current_pose_.pose.position.y);
            
            if (dist < 0.2) {
                path_.poses.clear(); 
                goal_handle_->succeed(result);
                return;
            }
        }
        loop_rate.sleep();
    }

    // ********
    // * Help *
    // ********
    /*
    rclcpp::Rate loop_rate(1.0); // 1 Hz

    while (rclcpp::ok()) {

        if (goal_handle_->is_canceling()) {
            ...
            return;
        }

        ...

        goal_handle_->publish_feedback(feedback);

        loop_rate.sleep();
    }
    */
}

void MotionControlNode::pathCallback(rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future) {
    // add code here
    auto response = future.get();
    if (response && response->plan.poses.size() > 0) {
        path_ = response->plan;
        if (goal_handle_ && goal_handle_->is_active()) {
            std::thread(&MotionControlNode::execute, this).detach();
        }
    } else {
        if (goal_handle_ && goal_handle_->is_active()) {
            auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
            goal_handle_->abort(result);
        }
    }
}

void MotionControlNode::odomCallback(const nav_msgs::msg::Odometry & msg) {
    // add code here
    current_pose_.header = msg.header;
    current_pose_.pose = msg.pose.pose;
    
    checkCollision();
    updateTwist();

    // ********
    // * Help *
    // ********
    /*
    checkCollision();
    updateTwist();
    */
}

void MotionControlNode::lidarCallback(const sensor_msgs::msg::LaserScan & msg) {
    // add code here
    laser_scan_ = msg;
}