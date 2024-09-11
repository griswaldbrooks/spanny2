#include "json.hpp"
#include "mdspan.hpp"
#include "serial/serial.h"
#include <cmath>
#include <fstream>
#include <future>
#include <iostream>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

namespace stdex = std::experimental;
using json = nlohmann::json;
enum struct bin_occupancy { OCCUPIED, EMPTY };

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

struct position_t {
  double x, y;
};

// Parameters that change... robot state?
// First joint (shoulder) is in index 0
using joint_angles = std::array<double, 2>;

// Parameters that don't change
struct kinematics_t {
  // link lengths
  // link from shoulder to elbow is index 0
  std::array<double, 2> link;
  // offset from {0, 0}
  position_t origin;
  // rotation from x forward
  double orientation;
  // which ik solution should be chosen
  bool elbow_up;
};

// Kinematics for left arm
// Given two joint andles, what is the x, y
position_t forward_kinematics(joint_angles const& joint, kinematics_t config) {
  position_t const tool{
      config.link[0] * std::cos(joint[0]) + config.link[1] * std::cos(joint[0] + joint[1]),
      config.link[0] * std::sin(joint[0]) + config.link[1] * std::sin(joint[0] + joint[1])};
  // Transform the solution by the robot pose
  return {tool.x * std::cos(config.orientation) - tool.y * std::sin(config.orientation) +
              config.origin.x,
          tool.x * std::sin(config.orientation) + tool.y * std::cos(config.orientation) +
              config.origin.y};
}

joint_angles bounded_range(joint_angles const& angles) {
  double const first = std::atan2(std::sin(angles[0]), std::cos(angles[0]));
  double const second = std::atan2(std::sin(angles[1]), std::cos(angles[1]));
  return {first, second};
}

// Inverse Kinematics
// Given an x, y, and elbow config what are the joint angles
joint_angles inverse_kinematics(position_t goal, kinematics_t config) {
  // Transform the goal back into the arm frame
  goal.x -= config.origin.x;
  goal.y -= config.origin.y;
  double const x = goal.x * std::cos(config.orientation) + goal.y * std::sin(config.orientation);
  double const y = -goal.x * std::sin(config.orientation) + goal.y * std::cos(config.orientation);
  double const elbow = std::acos((std::pow(x, 2) + std::pow(y, 2) - std::pow(config.link[0], 2) -
                                  std::pow(config.link[1], 2)) /
                                 (2. * config.link[0] * config.link[1]));
  double const shoulderish = std::atan2(config.link[1] * std::sin(elbow),
                                        config.link[0] + config.link[1] * std::cos(elbow));
  if (config.elbow_up) {
    return bounded_range({std::atan2(y, x) - shoulderish, elbow});
  }
  return bounded_range({std::atan2(y, x) + shoulderish, -elbow});
};

struct robot_arm {
  robot_arm(std::string port, uint32_t baudrate,
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

struct bounds_checked_layout {
  template <class Extents>
  struct mapping : stdex::layout_right::mapping<Extents> {
    using extents_type = Extents;
    using size_type = typename extents_type::size_type;
    using base_t = stdex::layout_right::mapping<Extents>;
    using base_t::base_t;
    size_type operator()(auto... idxs) const {
      [&]<size_t... Is>(std::index_sequence<Is...>) {
        if (((idxs < 0 || idxs >= this->extents().extent(Is)) || ...)) {
          throw std::out_of_range("Invalid bin index");
        }
      }
      (std::make_index_sequence<sizeof...(idxs)>{});
      return this->base_t::operator()(idxs...);
    }
  };
};

struct bin_checker {
  using element_type = bin_state;
  using reference = std::future<bin_state>;
  using data_handle_type = robot_arm*;

  reference access(data_handle_type ptr, std::ptrdiff_t offset) const {
    // We know ptr will be valid asynchronously because we construct it on the
    // stack of main. In real code we might want to use a shared_ptr or
    // something
    return std::async([=] {
      return bin_state{ptr->is_bin_occupied(static_cast<int>(offset)),
                       offset_to_coord(static_cast<int>(offset))};
    });
  }
};

// Also need a view to get the coordinates of a bin from json

using bin_view = stdex::mdspan<bin_state,
                               // We're treating our 6 bins as a 3x2 matrix
                               stdex::extents<uint32_t, 3, 2>,
                               // Our layout should do bounds-checking
                               bounds_checked_layout,
                               // Our accessor should tell the robot to
                               // asynchronously access the bin
                               bin_checker>;

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

#include <random>
bool near(double a, double b, double tolerance = 0.00001) { return std::abs(a - b) < tolerance; }

bool same(joint_angles const& lhs, joint_angles const& rhs) {
  return near(lhs[0], rhs[0]) && near(lhs[1], rhs[1]);
}

int main() {
  std::ifstream bin_file{"src/spanny2/config/bin_config.json"};
  json bin_config = json::parse(bin_file);
  std::cout << bin_config.dump(2) << std::endl;

  std::array<position_t, 6> bin_positions;
  std::mdspan<position_t, std::extents<std::size_t, 6>, bounds_checked_layout> bin_setter{
      bin_positions.data()};
  for (std::size_t ndx = 0; ndx < bin_setter.extent(0); ndx++) {
    auto const position = bin_config["locations"][ndx];
    bin_setter(ndx) = position_t{position[0], position[1]};
  }
  std::mdspan<position_t, std::extents<std::size_t, 3, 2>, bounds_checked_layout> bin_grid{
      bin_positions.data()};

  kinematics_t left_arm, right_arm;
  // 8.5 mm is the offset of the light detector from the end of the forearm
  left_arm.link = {0.150, 0.160 - 0.0085};
  left_arm.origin = {0.026, 0.078};
  left_arm.orientation = std::numbers::pi / 2.;
  left_arm.elbow_up = false;

  right_arm.link = left_arm.link;
  // 0.381 is the offset for the right arm base
  right_arm.origin = {0.026 + 0.381, 0.078};
  right_arm.orientation = std::numbers::pi / 2.;
  right_arm.elbow_up = true;

  // Testing
  {
    auto const angles = inverse_kinematics({0.185, 0.067}, right_arm);
    auto const end_effector = forward_kinematics(angles, right_arm);
    std::cout << "right end_effector = {" << end_effector.x << ", " << end_effector.y << "}\n";
    std::cout << "angles = {" << angles[0] << ", " << angles[1] << "}\n";
  }
  {
    auto const angles = inverse_kinematics({0.185, 0.067}, left_arm);
    auto const end_effector = forward_kinematics(angles, left_arm);
    std::cout << "left end_effector = {" << end_effector.x << ", " << end_effector.y << "}\n";
    std::cout << "angles = {" << angles[0] << ", " << angles[1] << "}\n";
  }
  return 0;

  auto arm = robot_arm{"/dev/ttyACM0", 9600};
  auto bins = bin_view(&arm);
  while (true) {
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
