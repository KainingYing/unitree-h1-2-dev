// H1-2 最小读状态程序 —— 纯只读，机器人不会动。
// 用 /opt/unitree_robotics 已装的官方 SDK（IDL 与固件配套），订阅 rt/lowstate。
// 用法: ./read_state [网卡名，默认 eth0]
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowState_.hpp>

#include <atomic>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

using namespace unitree::robot;
using LowState = unitree_hg::msg::dds_::LowState_;

static std::atomic<bool> g_got{false};
static LowState g_state;

void Handler(const void *msg) {
  g_state = *static_cast<const LowState *>(msg);
  g_got = true;
}

int main(int argc, char **argv) {
  std::string iface = (argc > 1) ? argv[1] : "eth0";
  std::cout << "[read_state] init domain=0 iface=" << iface << std::endl;
  ChannelFactory::Instance()->Init(0, iface);

  ChannelSubscriberPtr<LowState> sub(
      new ChannelSubscriber<LowState>("rt/lowstate"));
  sub->InitChannel(Handler, 1);

  for (int i = 0; i < 500 && !g_got; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

  if (!g_got) {
    std::cout << "NO_LOWSTATE_RECEIVED (iface=" << iface
              << "). 试试: ./read_state \"\"  (监听所有网卡)" << std::endl;
    return 1;
  }

  auto rpy = g_state.imu_state().rpy();
  auto &m = g_state.motor_state();
  std::cout << std::fixed << std::setprecision(4);
  std::cout << "=== LowState OK ===\n";
  std::cout << "IMU rpy(rad) = " << rpy[0] << ", " << rpy[1] << ", " << rpy[2]
            << "\n";
  std::cout << "motor_state array size = " << m.size() << "\n";
  std::cout << "idx       q       dq      tau_est\n";
  for (size_t i = 0; i < m.size(); ++i) {
    std::cout << "[" << std::setw(2) << i << "] " << std::setw(9) << m[i].q()
              << " " << std::setw(8) << m[i].dq() << " " << std::setw(8)
              << m[i].tau_est() << "\n";
  }
  std::cout << "(q≈0 且 tau≈0 的多半是未使用的电机槽位)\n";
  return 0;
}
