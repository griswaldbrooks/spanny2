#include "expected.hpp"
#include "json.hpp"
#include "mdspan.hpp"
#include "serial/serial.h"
#include <cmath>
#include <fstream>
#include <future>
#include <iostream>
#include <memory>
#include <numbers>
#include <ranges>
#include <stdexcept>
#include <string>
#include <vector>

namespace stdex = std::experimental;
using json = nlohmann::json;
enum struct bin_occupancy { OCCUPIED, EMPTY };

struct position_t {
  double x, y;
};

struct bin_state {
  bin_occupancy occupancy;
  position_t location;
};

struct bin_coordinate {
  int x, y;
};

bin_coordinate offset_to_coord(int offset, int width = 3) {
  int const row = offset / width;
  int const col = offset % width;
  return {row, col};
}

// Parameters that change... robot state
// First joint (shoulder) is in index 0
using joint_angles = std::array<double, 2>;

using path_t = std::vector<position_t>;

// Parameters that don't change
struct kinematics_t {
  std::string description;
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

struct bounds_checked_layout {
  template <class Extents>
  struct mapping : std::layout_right::mapping<Extents> {
    using extents_type = Extents;
    using size_type = typename extents_type::size_type;
    using base_t = std::layout_right::mapping<Extents>;
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

using bin_grid_t = std::mdspan<position_t, std::extents<std::size_t, 2, 3>, bounds_checked_layout>;

template <typename Arbiter>
struct bin_checker_t {
  using element_type = tl::expected<bin_state, std::string>;
  using reference = element_type;
  using data_handle_type = position_t*;
  using size_type = std::size_t;

  explicit bin_checker_t(Arbiter* checker) : checker_{std::move(checker)} {}

  reference access(data_handle_type bin_positions, size_type offset) const {
    auto const goal = bin_positions[offset];
    auto const state_maybe = (*checker_)(goal);
    if (!state_maybe.has_value()) {
      return tl::make_unexpected(std::string{state_maybe.error()});
    }
    return bin_state{state_maybe.value(), goal};
  }

 private:
  Arbiter* checker_;
};

template <typename Arbiter>
struct bin_checker_async_t {
  using element_type = tl::expected<bin_state, std::string>;
  using reference = std::future<element_type>;
  using data_handle_type = position_t*;
  using size_type = std::size_t;

  explicit bin_checker_async_t(Arbiter* checker) : checker_{std::move(checker)} {}

  reference access(data_handle_type bin_positions, size_type offset) const {
    auto const goal = bin_positions[offset];
    return (*checker_)(goal);
  }

 private:
  Arbiter* checker_;
};

// template <typename Arbiter>
// using bin_view_t = std::mdspan<tl::expected<bin_state, std::string>,
//                                std::extents<std::size_t, 2, 3>,
//                                bounds_checked_layout,
//                                bin_checker_t<Arbiter>>;
template <typename Arbiter>
using bin_view_t = std::mdspan<typename bin_checker_t<Arbiter>::element_type,
                               // We're treating our 6 bins as a 2x3 matrix
                               std::extents<std::size_t, 2, 3>,
                               // Our layout should do bounds-checking
                               bounds_checked_layout,
                               // Tell the robot to synchronously access the bin
                               bin_checker_t<Arbiter>>;

template <typename Arbiter>
using bin_view_async_t = std::mdspan<typename bin_checker_t<Arbiter>::element_type,
                                     // We're treating our 6 bins as a 2x3 matrix
                                     std::extents<std::size_t, 2, 3>,
                                     // Our layout should do bounds-checking
                                     bounds_checked_layout,
                                     // Tell the robot to synchronously access the bin
                                     bin_checker_async_t<Arbiter>>;

std::ostream& operator<<(std::ostream& os, tl::expected<bin_state, std::string> const& bin_maybe) {
  if (!bin_maybe.has_value()) {
    os << bin_maybe.error();
    return os;
  }
  auto const& state = bin_maybe.value();
  os << "(" << state.location.x << ", " << state.location.y << ") = ";
  switch (state.occupancy) {
    case bin_occupancy::OCCUPIED:
      std::cout << "OCCUPIED";
      break;
    case bin_occupancy::EMPTY:
      std::cout << "EMPTY";
      break;
  }
  return os;
}

double to_radians(double degrees) { return degrees * std::numbers::pi / 180.; }
double to_degrees(double radians) { return radians * 180. / std::numbers::pi; }

bool near(double a, double b, double tolerance = 0.00001) { return std::abs(a - b) < tolerance; }

bool same(joint_angles const& lhs, joint_angles const& rhs) {
  return near(lhs[0], rhs[0]) && near(lhs[1], rhs[1]);
}

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

path_t forward_kinematics(std::vector<joint_angles> const& joints, kinematics_t const& model) {
  path_t path;
  std::ranges::transform(joints, std::back_inserter(path),
                         [&](auto const& j) { return forward_kinematics(j, model); });
  return path;
}

joint_angles bounded_range(joint_angles const& angles) {
  double const first = std::atan2(std::sin(angles[0]), std::cos(angles[0]));
  double const second = std::atan2(std::sin(angles[1]), std::cos(angles[1]));
  return {first, second};
}

// Inverse Kinematics
// Given an x, y, and elbow config what are the joint angles
joint_angles inverse_kinematics(position_t goal, kinematics_t const& config) {
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

std::vector<joint_angles> inverse_kinematics(path_t const& path, kinematics_t const& model) {
  std::vector<joint_angles> angles;
  std::ranges::transform(path, std::back_inserter(angles),
                         [&](auto const& p) { return inverse_kinematics(p, model); });
  return angles;
}

path_t lerp(position_t const& start, position_t const& goal, double step = 0.1) {
  path_t path{start};
  for (double t = step; t < 1.; t += step) {
    path.emplace_back(std::lerp(start.x, goal.x, t), std::lerp(start.y, goal.y, t));
  }
  path.push_back(goal);
  return path;
}

std::vector<joint_angles> lerp(joint_angles const& start, joint_angles const& goal,
                               double step = 0.1) {
  std::vector<joint_angles> path{start};
  for (double t = step; t < 1.; t += step) {
    path.push_back({std::lerp(start[0], goal[0], t), std::lerp(start[1], goal[1], t)});
  }
  path.push_back(goal);
  return path;
}

position_t generate_waypoint(position_t const& p, kinematics_t const& model) {
  auto const offset = std::copysign(0.02, model.origin.x - p.x);
  return {p.x + offset, p.y};
}

// function that takes a goal and arm and returns a plan
path_t plan_cartesian(path_t const& waypoints) {
  if (waypoints.empty()) {
    return {{0., 0.}};  // Convert to std::expected
  }
  if (waypoints.size() == 1) {
    return {waypoints[0]};
  }
  // Add the first point to the path so that with subsequent plans
  // we can prevent endpoint duplication
  path_t path = {waypoints[0]};

  for (std::size_t i = 0u; i < waypoints.size() - 1; ++i) {
    auto const start = waypoints[i];
    auto const goal = waypoints[i + 1];
    auto const line = lerp(start, goal);
    // Do not add the first point as it is already in the path
    path.insert(path.end(), ++line.begin(), line.end());
  }
  return path;
}

// function that takes a goal and arm and returns a plan
std::vector<joint_angles> plan_joint(path_t const& waypoints, kinematics_t const& model) {
  if (waypoints.empty()) {
    return {{0., 0.}};  // Convert to std::expected
  }
  if (waypoints.size() == 1) {
    return inverse_kinematics(waypoints, model);
  }
  // Add the first joint angle to the path so that with subsequent plans
  // we can prevent endpoint duplication
  std::vector<joint_angles> path = {inverse_kinematics(waypoints[0], model)};
  for (std::size_t i = 0u; i < waypoints.size() - 1; ++i) {
    auto const start = inverse_kinematics(waypoints[i], model);
    auto const goal = inverse_kinematics(waypoints[i + 1], model);
    auto const line = lerp(start, goal);
    // Do not add the first point as it is already in the path
    path.insert(path.end(), ++line.begin(), line.end());
  }
  return path;
}

// function that takes arms current goal and returns which arm should service the goal
kinematics_t which_arm_nearest(position_t const& goal, std::array<position_t, 2> const& goals,
                               std::array<kinematics_t, 2> const& arms) {
  // Pair up the distance metrics with the arm configurations
  std::array<std::pair<double, kinematics_t>, 2> const distances = {
      {{std::hypot(goal.x - goals[0].x, goal.y - goals[0].y), arms[0]},
       {std::hypot(goal.x - goals[1].x, goal.y - goals[1].y), arms[1]}}};
  // Find the nearest arm to goal
  auto const argmin = std::min_element(
      distances.begin(), distances.end(),
      [](auto const& lhs, auto const& rhs) { return std::get<0>(lhs) < std::get<0>(rhs); });
  // Return the arm configuration
  return std::get<1>(*argmin);
}

// function that takes a goal and returns which arm can service it, based on left/right partition
kinematics_t which_arm_parition(position_t const& goal, std::array<kinematics_t, 2> const& arms) {
  return which_arm_nearest(goal, {{arms[0].origin, arms[1].origin}}, arms);
}

// function to generate goals available to each arm, taking into account the 2d nature of the bins
// and the arm being on left or right, not considering other arm goals
std::vector<position_t> available_goals(bin_grid_t bins, kinematics_t arm) {
  // Check if the arm is to the left or the right
  // If it is on the left, then the indices are:
  if (arm.origin.x < bins(0, 0).x) {
    return {bins(0, 0), bins(0, 1), bins(0, 2), bins(1, 0), bins(1, 2)};
  }
  return {bins(0, 0), bins(0, 2), bins(1, 0), bins(1, 1), bins(1, 2)};
}

// function that takes available goal list, other arm goal, active arm and returns goal subset
std::vector<position_t> available_goals(bin_grid_t bins, kinematics_t arm, position_t other_goal) {
  // if the goal is within a ball of the other_goal and not in front (relative to the arm
  // direction)
  auto const in_ball = [&](auto const& goal) {
    auto const adjacent = std::hypot(bins(0, 0).x - bins(0, 1).x, bins(0, 0).y - bins(0, 1).y);
    // offset was tuned to prevent any goals from being available when an arm is crossing into
    // the middle of the other robot's side
    auto const offset = 0.03;
    return std::hypot(other_goal.x - goal.x, other_goal.y - goal.y) < (adjacent + offset);
  };

  auto const is_behind = [&](auto const& goal) {
    return (arm.origin.x <= other_goal.x && other_goal.x <= goal.x) ||
           (arm.origin.x >= other_goal.x && other_goal.x >= goal.x);
  };

  // Get the normal goal set
  auto goals = available_goals(bins, arm);
  std::erase_if(goals, [&](auto const& goal) { return is_behind(goal) && in_ball(goal); });
  return goals;
}

std::string serialize([[maybe_unused]] std::array<int, 2> const& joint_angle) { return ""; }

struct hardware_interface {
  virtual ~hardware_interface() = default;
  virtual tl::expected<bin_occupancy, std::string> operator()(
      std::vector<joint_angles> const& path) const = 0;
};

struct mock_hardware : public hardware_interface {
  explicit mock_hardware(kinematics_t model) : model_(std::move(model)) {}
  tl::expected<bin_occupancy, std::string> operator()(std::vector<joint_angles> const& path) const {
    std::cout << model_.description << "\n";
    std::cout << "x, y\n";
    std::ranges::for_each(path, [&](auto const& j) {
      auto const p = forward_kinematics(j, model_);
      std::cout << p.x << ", " << p.y << "\n";
    });
    return bin_occupancy::EMPTY;
  };
  kinematics_t model_;
};

struct hardware : public hardware_interface {
  hardware(std::string port, uint32_t baudrate,
           serial::Timeout timeout = serial::Timeout::simpleTimeout(10000))
      : serial_(std::move(port), baudrate, std::move(timeout)) {}

  tl::expected<bin_occupancy, std::string> operator()(std::vector<joint_angles> const& path) const {
    if (!serial_.isOpen()) {
      return tl::make_unexpected("Error opening serial port");
    }
    // Convert the angles to degrees for arduino, rounded
    std::vector<std::array<int, 2>> path_d;
    std::ranges::transform(
        path, std::back_inserter(path_d), [](auto const& j) -> std::array<int, 2> {
          return {static_cast<int>(to_radians(j[0])), static_cast<int>(to_radians(j[1]))};
        });
    // Parse angles into string, maybe
    // shoulder, shoulder\n
    // as in
    // 176, 25\n
    // but it wouldn't surprise me if a header byte is needed, in which case
    // {176, 25} might be just as well
    std::ranges::for_each(
        path_d, [this](auto const& joint_angle) { serial_.write(serialize(joint_angle)); });
    try {
      // Send read sensor command so there isn't ambiguity about return values from the serial
      serial_.write("this will be dependent on the parsing format");
      auto const result = std::stoi(serial_.readline());
      if (result == 1) {
        return bin_occupancy::OCCUPIED;
      }
      return bin_occupancy::EMPTY;
    } catch (std::invalid_argument const& e) {
      return tl::make_unexpected("Invalid argument from serial port");
    } catch (std::out_of_range const& e) {
      return tl::make_unexpected("Out of range from serial port");
    }
  }

  mutable serial::Serial serial_;
};
// TODO: robot should return to waypoint after reading state
// TODO: on start up, robots should go to home
// TODO: on shut down, robots should go to home
// TODO: on the synchronous ones, should the value be reported after the moves are done?
// TODO: in arduino, need a joint angle to pwm function
//
// goal arbiter, single arm
// - goal comes in, add to queue
// - if robot is not available, wait until it is done with its current goal
// - now that robot is available
//    - generate path to goal
//    - command path
//      - send joint command
//      - wait for arm to report success
//    - read sensor
//      - report value
//    - generate path to home
//      - generate path to waypoint
//      - generate path to home
//    - command path

struct arbiter_single {
  arbiter_single(kinematics_t model, std::unique_ptr<hardware_interface> hw)
      : model_{std::move(model)},
        state_{0., 0.},
        home_{forward_kinematics(state_, model_)},
        command_path_{std::move(hw)} {
    (*command_path_)({state_});
  }

  tl::expected<bin_occupancy, std::string> operator()(position_t const& goal) {
    auto const near_goal = generate_waypoint(goal, model_);
    auto const goto_goal = inverse_kinematics(plan_cartesian({home_, near_goal, goal}), model_);
    auto const sensor_reading_maybe = (*command_path_)(goto_goal);
    // Out current cartesian position should be at the goal
    auto const goto_home = inverse_kinematics(plan_cartesian({goal, near_goal, home_}), model_);
    (*command_path_)(goto_home);
    return sensor_reading_maybe;
  }

 private:
  kinematics_t model_;
  joint_angles state_;
  position_t home_;
  std::unique_ptr<hardware_interface> command_path_;
};

struct arbiter_dual {
  arbiter_dual(kinematics_t model_left, std::unique_ptr<hardware_interface> hw_left,
               kinematics_t model_right, std::unique_ptr<hardware_interface> hw_right)
      : model_{{std::move(model_left), std::move(model_right)}},
        home_{{model_[0].description, forward_kinematics({0., 0.}, model_[0])},
              {model_[1].description, forward_kinematics({0., 0.}, model_[1])}} {
    hw_.emplace(model_[0].description, std::move(hw_left));
    hw_.emplace(model_[1].description, std::move(hw_right));
  }

  tl::expected<bin_occupancy, std::string> operator()(position_t const& goal) {
    auto const model = which_arm_parition(goal, model_);
    auto& command_path = *hw_.at(model.description);
    auto const home = home_.at(model.description);
    auto const near_goal = generate_waypoint(goal, model);
    auto const goto_goal = inverse_kinematics(plan_cartesian({home, near_goal, goal}), model);
    auto const sensor_reading_maybe = command_path(goto_goal);
    // Out current cartesian position should be at the goal
    auto const goto_home = inverse_kinematics(plan_cartesian({goal, near_goal, home}), model);
    command_path(goto_home);
    return sensor_reading_maybe;
  }

 private:
  std::array<kinematics_t, 2> model_;
  std::map<std::string, position_t> home_;
  std::map<std::string, std::unique_ptr<hardware_interface>> hw_;
};

struct arbiter_dual_async {
  arbiter_dual_async(kinematics_t model_left, std::unique_ptr<hardware_interface> hw_left,
                     kinematics_t model_right, std::unique_ptr<hardware_interface> hw_right)
      : model_{{std::move(model_left), std::move(model_right)}},
        home_{{model_[0].description, forward_kinematics({0., 0.}, model_[0])},
              {model_[1].description, forward_kinematics({0., 0.}, model_[1])}} {
    hw_.emplace(model_[0].description, std::move(hw_left));
    hw_.emplace(model_[1].description, std::move(hw_right));
  }

  std::future<tl::expected<bin_state, std::string>> operator()(
      [[maybe_unused]] position_t const& goal) {
    return std::async([=] {
      return tl::expected<bin_state, std::string>{
          tl::make_unexpected(std::string{"Unimplemented"})};
    });
  }

 private:
  std::array<kinematics_t, 2> model_;
  std::map<std::string, position_t> home_;
  std::map<std::string, std::unique_ptr<hardware_interface>> hw_;
};
// goal arbiter, dual arm partitioned
// - goal comes in
// - add goal to the queue of the correct robot
// - goal arbiter single arm

// goal arbiter, dual arm non-partitioned
// - goal comes in, add to set
//
// - when an arm becomes available
//   - get the current goal of the other arm
//   - get the list of available goals
//   - check if any of the available goals are in the goal set
//   - if there is more than one, choose one that is closest to current position
//   - generate path to goal
//   - command path
//   - read sensor, report async?
//   - get waypoint next to goal
//   - generate path to waypoint
//   - command path
//

// Bin checker for single arm sync should
//   reference access(data_handle_type ptr, std::ptrdiff_t offset) const {
//    auto const goal = ptr[offset];
//    // give goal to robot arm
//    // return what the robot arm sensed
//

// TODO: This should still be shown somehow to demonstrate that no stack/heap memory
//        is actually needed
// struct bin_checker {
//   using element_type = bin_state;
//   using reference = std::future<bin_state>;
//   using data_handle_type = robot_arm*;
//
//   reference access(data_handle_type ptr, std::ptrdiff_t offset) const {
//     // We know ptr will be valid asynchronously because we construct it on the
//     // stack of main. In real code we might want to use a shared_ptr or
//     // something
//     return std::async([=] {
//       return bin_state{ptr->is_bin_occupied(static_cast<int>(offset)),
//                        offset_to_coord(static_cast<int>(offset))};
//     });
//   }
// };

int main() {
  std::ifstream bin_file{"src/spanny2/config/bin_config.json"};
  json bin_config = json::parse(bin_file);
  /* TEST json loading */
  // std::cout << bin_config.dump(2) << std::endl;

  std::array<position_t, 6> bin_positions;
  std::ranges::transform(bin_config["locations"].get<std::vector<std::vector<double>>>(),
                         bin_positions.begin(), [](auto const& p) {
                           return position_t{p[0], p[1]};
                         });

  /* TEST 2x3 mdspan */
  // bin_grid_t bin_grid{bin_positions.data()};
  // for (auto i = 0u; i < bin_grid.extent(0); ++i) {
  //   for (auto j = 0u; j < bin_grid.extent(1); ++j) {
  //     auto const p = bin_grid(i, j);
  //     std::cout << "(" << i << ", " << j << ") = {" << p.x << ", " << p.y << "}\n";
  //   }
  // }

  kinematics_t left_arm, right_arm;
  left_arm.description = "left arm";
  // 8.5 mm is the offset of the light detector from the end of the forearm
  left_arm.link = {0.150, 0.160 - 0.0085};
  left_arm.origin = {0.026, 0.078};
  left_arm.orientation = std::numbers::pi / 2.;
  left_arm.elbow_up = false;

  right_arm.description = "right arm";
  right_arm.link = left_arm.link;
  // 0.431 is the offset for the right arm base
  right_arm.origin = {0.026 + 0.431, 0.078};
  right_arm.orientation = std::numbers::pi / 2.;
  right_arm.elbow_up = true;

  /* TEST KINEMATICS */
  // {
  //   auto const angles = inverse_kinematics({0.185, 0.067}, right_arm);
  //   auto const end_effector = forward_kinematics(angles, right_arm);
  //   std::cout << "right end_effector = {" << end_effector.x << ", " << end_effector.y << "}\n";
  //   std::cout << "angles = {" << angles[0] << ", " << angles[1] << "}\n";
  // }
  // {
  //   auto const angles = inverse_kinematics({0.185, 0.067}, left_arm);
  //   auto const end_effector = forward_kinematics(angles, left_arm);
  //   std::cout << "left end_effector = {" << end_effector.x << ", " << end_effector.y << "}\n";
  //   std::cout << "angles = {" << angles[0] << ", " << angles[1] << "}\n";
  // }

  /* TEST PLANNER */
  // auto const path = plan_cartesian({forward_kinematics({0, 0}, right_arm),
  //                                   generate_waypoint(bin_grid(0, 0), right_arm), bin_grid(0,
  //                                   0)});
  // for (auto const& p : path) {
  // auto const j = inverse_kinematics(p, right_arm);
  // std::cout << j[0] << ", " << j[1] << "\n";
  // std::cout << p.x << ", " << p.y << "\n";
  // }

  /* TEST ARM PARTITIONING */
  // std::ranges::for_each(bin_positions, [&](auto const& goal) {
  //   auto const arm = which_arm_parition(goal, {left_arm, right_arm});
  //   std::cout << arm.description << " {" << goal.x << ", " << goal.y << "}\n";
  // });

  /* TEST AVAILABLE GOALS */
  // std::ranges::for_each(bin_positions, [&](auto const other_goal) {
  //   std::cout << left_arm.description << " other_goal = {" << other_goal.x << ", " <<
  //   other_goal.y
  //             << "}\n";
  //   // auto const goals = available_goals(bin_grid, left_arm);
  //   auto const goals = available_goals(bin_grid, left_arm, other_goal);
  //   std::ranges::for_each(goals, [](auto const& p) { std::cout << p.x << ", " << p.y << "\n";
  //   });
  // });

  /* TEST SINGLE ARM SYNCHRONOUS */
  // {
  //   auto arbiter = arbiter_single{left_arm, std::make_unique<mock_hardware>(left_arm)};
  //   auto bin_checker = bin_checker_t{&arbiter};
  //   auto bins = bin_view_t(bin_positions.data(), {}, bin_checker);
  //   for (auto i = 0u; i != bins.extent(0); ++i) {
  //     for (auto j = 0u; j != bins.extent(1); ++j) {
  //       std::cout << bins(i, j) << "\n";
  //     }
  //   }
  // }
  /* TEST DUAL ARM SYNCHRONOUS */
  // {
  //   auto arbiter = arbiter_dual{left_arm, std::make_unique<mock_hardware>(left_arm), right_arm,
  //                               std::make_unique<mock_hardware>(right_arm)};
  //   auto bin_checker = bin_checker_t{&arbiter};
  //   auto bins = bin_view_t(bin_positions.data(), {}, bin_checker);
  //   for (auto i = 0u; i != bins.extent(0); ++i) {
  //     for (auto j = 0u; j != bins.extent(1); ++j) {
  //       std::cout << bins(i, j) << "\n";
  //     }
  //   }
  // }
  /* TEST DUAL ARM ASYNCHRONOUS */
  {
    auto arbiter = arbiter_dual_async{left_arm, std::make_unique<mock_hardware>(left_arm),
                                      right_arm, std::make_unique<mock_hardware>(right_arm)};
    auto bin_checker = bin_checker_async_t{&arbiter};
    auto bins = bin_view_async_t(bin_positions.data(), {}, bin_checker);
    std::vector<std::future<tl::expected<bin_state, std::string>>> futures;
    for (auto i = 0u; i != bins.extent(0); ++i) {
      for (auto j = 0u; j != bins.extent(1); ++j) {
        futures.push_back(bins(i, j));
      }
    }
    // Let the arms resolve the moves
    for (auto const& future : futures) {
      future.wait();
    }

    for (auto& future : futures) {
      std::cout << future.get() << "\n";
    }
  }
  return 0;

  // auto arm = robot_arm{"/dev/ttyACM0", 9600};
  // auto bins = bin_view(&arm);
  // while (true) {
  //   std::vector<std::future<bin_state>> futures;
  //   for (auto i = 0u; i < bins.extent(0); ++i) {
  //     for (auto j = 0u; j < bins.extent(1); ++j) {
  //       futures.push_back(bins(i, j));
  //     }
  //   }
  //   for (auto const& future : futures) {
  //     future.wait();
  //   }
  //   for (auto& future : futures) {
  //     print_state(future.get());
  //   }
  //   std::cout << "====================" << std::endl;
  // }
  // return 0;
}
