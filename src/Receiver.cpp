#include <mpc-rbt-solution/Receiver.hpp>

void Receiver::Node::run()
{
  while (errno != EINTR) {
    RCLCPP_INFO(logger, "Waiting for data ...");
    Socket::IPFrame frame{};
    if (receive(frame)) {
      RCLCPP_INFO(logger, "Received data from host: '%s:%d'", frame.address.c_str(), frame.port);

      callback(frame);

    } else {
      RCLCPP_WARN(logger, "Failed to receive data.");
    }
  }
}

void Receiver::Node::onDataReceived(const Socket::IPFrame & frame)
{
// deserialize v Utils.cpp používá vnitřně frame.serializedData
  if (Utils::Message::deserialize(frame, data)) {
      RCLCPP_INFO(logger, "RECEIVER: Přijato x: %f, y: %f, z: %f", data.x, data.y, data.z);
  }
}
