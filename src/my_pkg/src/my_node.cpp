#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include <chrono>

using namespace std::chrono_literals;

class MyNode : public rclcpp::Node {
public:
  MyNode() : Node("my_node") {
    // Registrace parametrů s defaultními hodnotami z bodu 5
    this->declare_parameter("volt_min", 32.0);
    this->declare_parameter("volt_max", 42.0);

    publisher_name_ = this->create_publisher<std_msgs::msg::String>("node_name", 10);
    publisher_batt_ = this->create_publisher<std_msgs::msg::Float32>("battery_percentage", 10);
    
    subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "battery_voltage", 
      rclcpp::SensorDataQoS(), 
      std::bind(&MyNode::battery_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(500ms, std::bind(&MyNode::publish_name, this));
    
    RCLCPP_INFO(this->get_logger(), "NODE S PARAMETRY JE READY");
  }

private:
  void battery_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    float voltage = msg->data;
    
    // Načtení aktuálních hodnot parametrů
    float v_min = this->get_parameter("volt_min").as_double();
    float v_max = this->get_parameter("volt_max").as_double();
    
    // Výpočet procent s využitím parametrů
    float range = v_max - v_min;
    float percentage = 0.0;
    
    if (range > 0.0) {
        percentage = (voltage - v_min) / range * 100.0f;
    }
    
    // Ošetření mezí
    if (percentage > 100.0f) percentage = 100.0f;
    if (percentage < 0.0f) percentage = 0.0f;

    auto out_msg = std_msgs::msg::Float32();
    out_msg.data = percentage;
    
    publisher_batt_->publish(out_msg);
    RCLCPP_INFO(this->get_logger(), "V_min: %.1fV, V_max: %.1fV | PRIJATO: %.2fV -> %.1f%%", 
                v_min, v_max, voltage, percentage);
  }

  void publish_name() {
    auto message = std_msgs::msg::String();
    message.data = "my_node";
    publisher_name_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_name_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_batt_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyNode>());
  rclcpp::shutdown();
  return 0;
}