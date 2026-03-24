#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "Localization.hpp"
#include <cmath>

LocalizationNode::LocalizationNode() : 
    rclcpp::Node("localization_node"), 
    last_time_(this->get_clock()->now()) {

    // Odometry message initialization
    odometry_.header.frame_id = "map";
    odometry_.child_frame_id = "base_link";
    // add code here
    odometry_.pose.pose.orientation.w = 1.0; // Inicializace kvaternionu, aby nebyl neplatny(nulovy)

    odometry_.pose.pose.position.x = -0.5;  // Inicializace polohy
    odometry_.pose.pose.position.y = 0;

    // Subscriber for joint_states
    // add code here
    joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&LocalizationNode::jointCallback, this, std::placeholders::_1));  // Poslouchani dat 
    // Publisher for odometry
    // add code here
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);      // POslani dat

    // tf_briadcaster 
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "Localization node started.");
}

void LocalizationNode::jointCallback(const sensor_msgs::msg::JointState & msg) {
    // add code here
    if (msg.velocity.size() < 2) return; // Ochrana proti prazdne zprve
    
    // --- PŘIDÁNO: Načtení času přesně ze zprávy senzoru (timestamp) ---
    rclcpp::Time current_time = msg.header.stamp;   // Casovy krok dt (delta_t)
    double dt = (current_time - last_time_).seconds();
    
    // Ochrana pro první spuštění (rozdíl mezi časem PC a časem simulátoru)
    if (dt <= 0.0 || dt > 10.0) {
        dt = 0.01; 
    }
    last_time_ = current_time;

    // Uložíme si přesný čas rovnou do odometrie, aby se použil pro publishOdometry a publishTransform
    odometry_.header.stamp = current_time;
    // ------------------------------------------------------------------

    updateOdometry(msg.velocity[0], msg.velocity[1], dt);   // Zavolani vlastni funkce s rychlosti jednoho a druheho kola a dt
    publishOdometry();  // Posle nove souradnice X,Y  na topic odometr_
    publishTransform(); // Aktualizace polohy v transformacnim stromu

    // ********
    // * Help *
    // ********
    /*
    auto current_time = this->get_clock()->now();

    updateOdometry(msg.velocity[0], msg.velocity[1], dt);
    publishOdometry();
    publishTransform();
    */
}

void LocalizationNode::updateOdometry(double left_wheel_vel, double right_wheel_vel, double dt) {
    // add code here
    double v_l = left_wheel_vel * robot_config::WHEEL_RADIUS;   // Ziskani rychlosti kola v pred
    double v_r = right_wheel_vel * robot_config::WHEEL_RADIUS;

    double linear = (v_r + v_l) / 2.0;  // Vypocet kynematiky robotu
    double angular = (v_l - v_r) / (2.0 * robot_config::HALF_DISTANCE_BETWEEN_WHEELS);

    tf2::Quaternion tf_quat;    // Ziskani natoce Yaw z kvaternihonu (ziskani delta)
    tf2::fromMsg(odometry_.pose.pose.orientation, tf_quat);
    double roll, pitch, theta;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, theta);

    // Eulerova integrace
    odometry_.pose.pose.position.x += linear * std::cos(theta) * dt;    // Samotny vypocet novych souradnic
    odometry_.pose.pose.position.y += linear * std::sin(theta) * dt;
    theta += angular * dt;

    theta = std::atan2(std::sin(theta), std::cos(theta));   // Normalizace theta na <-pi ; +pi>

    tf2::Quaternion q;  // Zpetna transformace theta do kvaternionu
    q.setRPY(0, 0, theta);
    odometry_.pose.pose.orientation = tf2::toMsg(q);

    // Uloyeni rychlosti pro zpravu
    odometry_.twist.twist.linear.x = linear;
    odometry_.twist.twist.angular.z = angular;

    // ********
    // * Help *
    // ********
    /*
    double linear =  ;
    double angular = ;  //robot_config::HALF_DISTANCE_BETWEEN_WHEELS

    tf2::Quaternion tf_quat;
    tf2::fromMsg(odometry_.pose.pose.orientation, tf_quat);
    double roll, pitch, theta;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, theta);

    theta = std::atan2(std::sin(theta), std::cos(theta));

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    */
}

void LocalizationNode::publishOdometry() {
    // add code here
    // --- UPRAVENO: Čas už nenastavujeme z PC, ale použijeme ten uložený ze senzoru ---
    // odometry_.header.stamp = this->get_clock()->now();  
    odometry_publisher_->publish(odometry_);   // Odeslani zpravy
}

void LocalizationNode::publishTransform() {
    // add code here
    geometry_msgs::msg::TransformStamped t; // Priprava zpravy
    
    // --- UPRAVENO: Stejný přesný čas jako má odometrie ---
    t.header.stamp = odometry_.header.stamp;
    
    t.header.frame_id = "map";      // Definice zpravy jakou budeme posilat
    t.child_frame_id = "base_link";

    t.transform.translation.x = odometry_.pose.pose.position.x;     // Prekopirovani dat
    t.transform.translation.y = odometry_.pose.pose.position.y;
    t.transform.translation.z = 0.0;
    t.transform.rotation = odometry_.pose.pose.orientation;

    tf_broadcaster_->sendTransform(t);  // Poslani dat ven
    
    // ********
    // * Help *
    // ********
    //tf_broadcaster_->sendTransform(t);
}