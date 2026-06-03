// 探测 rt/arm_sdk 是否被运控监听 —— 只创建 publisher，绝不 Write 数据，机器人不会动。
// 创建后保持一段时间，期间用 ros2 topic info -v /arm_sdk 查 subscriber 数。
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>

#include <chrono>
#include <iostream>
#include <thread>

using namespace unitree::robot;
using LowCmd = unitree_hg::msg::dds_::LowCmd_;

int main(int argc, char **argv) {
  std::string iface = (argc > 1) ? argv[1] : "eth0";
  int hold = (argc > 2) ? std::stoi(argv[2]) : 20;
  ChannelFactory::Instance()->Init(0, iface);

  ChannelPublisherPtr<LowCmd> arm(new ChannelPublisher<LowCmd>("rt/arm_sdk"));
  arm->InitChannel();
  ChannelPublisherPtr<LowCmd> low(new ChannelPublisher<LowCmd>("rt/lowcmd"));
  low->InitChannel();

  std::cout << "[probe] rt/arm_sdk & rt/lowcmd publishers created (NO data "
               "written). holding "
            << hold << "s. iface=" << iface << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(hold));
  std::cout << "[probe] done" << std::endl;
  return 0;
}
