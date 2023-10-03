#include <iostream>
#include <string>
#include "serial/serial.h"
#include "mdspan.hpp"

namespace stdex = std::experimental;

struct robot_arm {
  bool is_bin_occupied(int bin) const {
    if (!robot_arm_.isOpen()) {
      std::cout << "Error opening serial port\n";
      return false;
    }
    auto const bin_str = std::to_string(bin) + "\n";
    robot_arm_.write(bin_str);
    // TODO wrap in try catch and return unexpected
    auto const result = std::stoi(robot_arm_.readline());
    return result == 1;
  }
  
  mutable serial::Serial robot_arm_ = serial::Serial("/dev/ttyACM1", 9600, serial::Timeout::simpleTimeout(10000));
};

// TODO Covert this to an enum
struct bin_state {
  bool is_occupied;
};

template <int nbins>
struct robot_command_accessor {
  using element_type = bin_state;
  using reference = bin_state&;
  using data_handle_type = bin_state*;

    reference access(data_handle_type ptr, std::ptrdiff_t offset) const {
    // TODO: check nbins and return UNKNOWN enum
      auto const arm = robot_arm{};
      ptr[offset].is_occupied = arm.is_bin_occupied(static_cast<int>(offset));
      return ptr[offset];
    }
};
 using ext_t = stdex::extents<uint32_t, 4>;
 using acc_t = robot_command_accessor<4>;

int main(int, char **) {
  auto bin_array = std::array<bin_state, 4>{};
  auto bins = stdex::mdspan<bool, ext_t, stdex::layout_right, acc_t>(bin_array.data());
  for (auto ndx = 0; ndx < 4; ++ndx) {
    std::cout << "bin[" << ndx << "] is occupied: " << (bins(ndx).is_occupied ? "yes" : "no") << "\n";
  }
  return 0;
}
