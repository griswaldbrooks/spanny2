#include <iostream>
#include <string>
#include "serial/serial.h"
#include "mdspan.hpp"
#include "expected.hpp"

namespace stdex = std::experimental;

enum struct bin_state {
  OCCUPIED,
  EMPTY
};

struct robot_arm {
  robot_arm(
    std::string port,
    uint32_t baudrate,
    serial::Timeout timeout = serial::Timeout::simpleTimeout(10000))
    : serial_(std::move(port), baudrate, std::move(timeout)) {}

  tl::expected<bin_state, std::string> is_bin_occupied(int bin) const {
    if (!serial_.isOpen()) {
      return tl::make_unexpected("Error opening serial port");
    }
    serial_.write(std::to_string(bin));
    try {
      auto const result = std::stoi(serial_.readline());
      if (result == 1) {
        return bin_state::OCCUPIED;
      } 
      return bin_state::EMPTY;
    } catch (std::invalid_argument const& e) {
        return tl::make_unexpected("Invalid argument from serial port");
    } catch (std::out_of_range const& e) {
        return tl::make_unexpected("Out of range from serial port");
    }
  }
  
  mutable serial::Serial serial_; 
};


template <int nbins>
struct bin_checker {
  using element_type = tl::expected<bin_state, std::string>;
  using reference = tl::expected<bin_state, std::string> const&;
  using data_handle_type = robot_arm*;

  inline static element_type const no_bin = tl::make_unexpected("Invalid bin index");
  mutable element_type recent_ = tl::make_unexpected("Bin has not been accessed");

  reference access(data_handle_type arm, std::ptrdiff_t offset) const {
    if (offset < 0 || offset >= nbins) {
      return no_bin;
    }
    try {
      recent_ = arm->is_bin_occupied(static_cast<int>(offset));
    } catch (std::exception const& e) {
      recent_ = tl::make_unexpected(std::string{"Error opening serial port"});
    }
    return recent_;
  }
};

using ext_t = stdex::extents<uint32_t, 4>;
using acc_t = bin_checker<4>;
using bin_view = stdex::mdspan<bin_state, ext_t, stdex::layout_right, acc_t>;

auto print_state = [](bin_state const& state) -> tl::expected<void, std::string> {
  switch (state) {
    case bin_state::OCCUPIED:
      std::cout << "OCCUPIED";
      break;
    case bin_state::EMPTY:
      std::cout << "EMPTY";
      break;
  }
  return {};
};

auto shrug = [](std::string const& msg) -> void {
  std::cout << "¯\\_(ツ)_/¯ " << msg;
};

int main(int, char **) {
  auto arm = robot_arm{"/dev/ttyACM0", 9600};
  auto bins = bin_view(&arm, {}, bin_checker<4>{});
  for (auto ndx = 0; ndx < 4; ++ndx) {
    std::cout << "Bin " << ndx << " is ";
    bins(ndx).and_then(print_state).or_else(shrug);
    std::cout << "\n";
  }
  return 0;
}
