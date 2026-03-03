#include <mpc-rbt-solution/Sender.hpp>

void Sender::Node::run()
{
  while (errno != EINTR) {
    if ((std::chrono::steady_clock::now() - timer_tick) < timer_period) continue;
    timer_tick = std::chrono::steady_clock::now();

    callback();
  }
}

void Sender::Node::onDataTimerTick()
{

// 1. Aktualizace dat (pohyb)
  data.x += 0.1;
  data.y += 0.2;
  data.z = 1.0;
  data.timestamp = std::chrono::system_clock::now().time_since_epoch().count();

  // 2. Příprava paketu
  Socket::IPFrame packet;
  
  // NASTAVENÍ ADRESY PŘÍMO DO PAKETU:
  packet.address = config.remoteAddress;
  packet.port = config.remotePort;

  // 3. Serializace (naplnění packet.serializedData)
  Utils::Message::serialize(packet, data);

  // 4. Odeslání
  // (přidáme (void), abychom umlčeli to varování 'ignoring return value')
  (void)send(packet);

  // 5. Logování
  RCLCPP_INFO(logger, "SENDER: Odesláno x: %f na %s:%d", 
              data.x, packet.address.c_str(), packet.port);      
}
