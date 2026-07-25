#include "odrive_s1_hw_interface/socketcan_bus.hpp"

#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>

namespace odrive_s1_hw_interface
{

SocketCanBus::~SocketCanBus()
{
  close();
}

bool SocketCanBus::open(const std::string & interface_name)
{
  close();

  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    return false;
  }

  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);

  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  return true;
}

void SocketCanBus::close()
{
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool SocketCanBus::send(const can_frame & frame)
{
  if (socket_fd_ < 0) {
    return false;
  }
  const ssize_t written = write(socket_fd_, &frame, sizeof(frame));
  return written == static_cast<ssize_t>(sizeof(frame));
}

std::optional<can_frame> SocketCanBus::receive(int timeout_ms)
{
  if (socket_fd_ < 0) {
    return std::nullopt;
  }

  struct pollfd pfd;
  pfd.fd = socket_fd_;
  pfd.events = POLLIN;
  pfd.revents = 0;

  const int ret = poll(&pfd, 1, timeout_ms);
  if (ret <= 0) {
    return std::nullopt;  // timeout or interrupted/error - treat both as "nothing available"
  }

  can_frame frame;
  const ssize_t nbytes = read(socket_fd_, &frame, sizeof(frame));
  if (nbytes != static_cast<ssize_t>(sizeof(frame))) {
    return std::nullopt;
  }

  return frame;
}

}  // namespace odrive_s1_hw_interface
