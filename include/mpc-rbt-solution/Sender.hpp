#ifndef COMMUNICATION_EXAMPLE_SENDER_H
#define COMMUNICATION_EXAMPLE_SENDER_H

// clang-format off
#include <chrono>

#include <mpc-rbt-solution/util/Socket.hpp>
#include <mpc-rbt-solution/util/Utils.hpp>
// clang-format on

namespace Sender
{
class Node : public Socket::UDP
{
public:
  explicit Node(const Utils::Config::Sender & senderConfig)
  : Socket::UDP(senderConfig.localPort),
    config(senderConfig),
    timer_period(config.sendingPeriodMillis)
  {
    
    //*********************TOTO JSEM PRIDAL****************************** */

    create();
    //connect(config.remoteAddress, config.remotePort); // Toto musí být aktivní
    callback = [this] { onDataTimerTick(); };

    //**************************AZ SEM************************* */

    //UNIMPLEMENTED(__PRETTY_FUNCTION__);  // TOTO JSEM ZAKOMETOVAL

  }

  void run();

private:
  void onDataTimerTick();

  Utils::Message data{};

  Utils::Config::Sender config;
  std::chrono::milliseconds timer_period;
  std::chrono::time_point<std::chrono::steady_clock> timer_tick = std::chrono::steady_clock::now();
  std::function<void()> callback;
  rclcpp::Logger logger{rclcpp::get_logger("sender")};
};
}  // namespace Sender

#endif  // COMMUNICATION_EXAMPLE_SENDER_H
