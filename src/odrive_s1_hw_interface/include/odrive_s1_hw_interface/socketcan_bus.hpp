#pragma once

#include <linux/can.h>

#include <optional>
#include <string>

namespace odrive_s1_hw_interface
{

/// Minimal wrapper around a raw Linux SocketCAN socket (AF_CAN / CAN_RAW).
///
/// This talks directly to the kernel's can0-style interface - it assumes the
/// interface has already been brought up and configured (bitrate, etc.) by the
/// system beforehand (e.g. via `ip link set can0 up type can bitrate 250000`),
/// the same way sample_can.py's bring_up_can() does it. This class does not
/// touch interface configuration at all, it only opens/binds a socket to an
/// already-up interface.
class SocketCanBus
{
public:
  SocketCanBus() = default;
  ~SocketCanBus();

  SocketCanBus(const SocketCanBus &) = delete;
  SocketCanBus & operator=(const SocketCanBus &) = delete;

  /// Opens and binds the socket to the given interface (e.g. "can0").
  /// Returns false on failure (interface missing/down, socket()/bind() failed).
  bool open(const std::string & interface_name);

  /// Closes the socket if open. Safe to call multiple times.
  void close();

  bool is_open() const {return socket_fd_ >= 0;}

  /// Sends a single CAN frame. Returns false on write failure.
  bool send(const can_frame & frame);

  /// Attempts to read one frame, waiting at most timeout_ms milliseconds.
  /// Pass 0 for a non-blocking poll (used in the real-time read() loop).
  /// Returns std::nullopt if nothing was available within the timeout.
  std::optional<can_frame> receive(int timeout_ms);

private:
  int socket_fd_ = -1;
};

}  // namespace odrive_s1_hw_interface
