#include <iostream>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include "serial/serial.h"
#include "mdspan.hpp"

namespace stdex = std::experimental;

enum struct bin_occupancy {
  OCCUPIED,
  EMPTY
};

struct bin_coordinate {
  int x, y;
};

struct bin_state {
  bin_occupancy occupancy;
  bin_coordinate coordinate;
};


bin_coordinate offset_to_coord(int offset, int width = 3) {
    int const row = offset / width;
    int const col = offset % width;
    return {row, col};
}

struct robot_arm {
  robot_arm(
    std::string port,
    uint32_t baudrate,
    serial::Timeout timeout = serial::Timeout::simpleTimeout(10000))
    : serial_(std::move(port), baudrate, std::move(timeout)) {}

  bin_occupancy is_bin_occupied(int bin) {
    if (!serial_.isOpen()) {
      throw std::runtime_error("Error opening serial port");
    }
    serial_.write(std::to_string(bin));
    auto const result = std::stoi(serial_.readline());
    if (result == 1) {
      return bin_occupancy::OCCUPIED;
    } 
    return bin_occupancy::EMPTY;
  }
  
  serial::Serial serial_;
};

struct bounds_checked_layout_policy {
  template <class Extents>
  struct mapping : stdex::layout_right::mapping<Extents> {
    using base_t = stdex::layout_right::mapping<Extents>;
    using base_t::base_t;
    std::ptrdiff_t operator()(auto... idxs) const {
      [&]<size_t... Is>(std::index_sequence<Is...>) {
        if (((idxs < 0 || idxs > this->extents().extent(Is)) || ...)) {
          throw std::out_of_range("Invalid bin index");
        }
      }(std::make_index_sequence<sizeof...(idxs)>{});
      return this->base_t::operator()(idxs...);
    }
  };
};

struct bin_checker {
  using element_type = bin_state;
  using reference = std::future<bin_state>;
  using data_handle_type = robot_arm*;

  reference access(data_handle_type ptr, std::ptrdiff_t offset) const {
    // We know ptr will be valid asynchronously because we construct it on the stack
    // of main. In real code we might want to use a shared_ptr or something
    return std::async([=]{
      return bin_state{ptr->is_bin_occupied(static_cast<int>(offset)), offset_to_coord(static_cast<int>(offset))};
    });
  }
};

using bin_view = stdex::mdspan<bin_state, 
  // We're treating our 6 bins as a 3x2 matrix
  stdex::extents<uint32_t, 3, 2>,
  // Our layout should do bounds-checking
  bounds_checked_layout_policy,
  // Our accessor should tell the robot to asynchronously access the bin
  bin_checker
>;

auto print_state = [](bin_state const& state) {
  std::cout << "Bin at (" << state.coordinate.x << ", " << state.coordinate.y << ") is ";
  switch (state.occupancy) {
    case bin_occupancy::OCCUPIED:
      std::cout << "OCCUPIED";
      break;
    case bin_occupancy::EMPTY:
      std::cout << "EMPTY";
      break;
  }
  std::cout << "\n";
};

int main() {
  auto arm = robot_arm{"/dev/ttyACM0", 9600};
  auto bins = bin_view(&arm);
  while(true) {
    std::vector<std::future<bin_state>> futures;
    for (auto i = 0u; i < bins.extent(0); ++i) {
      for (auto j = 0u; j < bins.extent(1); ++j) {
          futures.push_back(bins(i, j));
      }
    }
    for (auto const& future : futures) {
        future.wait();
    }
    for (auto& future : futures) {
        print_state(future.get());
    }
    std::cout << "====================" << std::endl;
  }
  return 0;
}
