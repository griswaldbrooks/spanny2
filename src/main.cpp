#include "expected.hpp"
#include "json.hpp"
#include "mdspan.hpp"
#include "serial/serial.h"
#include <atomic>
#include <cmath>
#include <fstream>
#include <future>
#include <iostream>
#include <memory>
#include <numbers>
#include <queue>
#include <random>
#include <ranges>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace stdex = std::experimental;
using json = nlohmann::json;
enum struct bin_occupancy { OCCUPIED, EMPTY };
/**
 * @brief Thread-safe queue. Particularly useful when multiple threads need to write to and/or read
 * from a queue. Move-only
 */
template <typename T>
class thread_safe_queue {
  std::queue<T> queue_;
  std::condition_variable cv_;
  mutable std::mutex mutex_;

 public:
  /**
   * @brief Get the size of the queue
   * @return Queue size
   */
  [[nodiscard]] auto size() const noexcept {
    auto const lock = std::lock_guard(mutex_);
    return queue_.size();
  }

  /**
   * @brief Check if the queue is empty
   * @return True if the queue is empty, otherwise false
   */
  [[nodiscard]] auto empty() const noexcept {
    auto const lock = std::lock_guard(mutex_);
    return queue_.empty();
  }

  /**
   * @brief Push data into the queue
   * @param value Data to push into the queue
   */
  template <typename U>
  void push(U&& value) noexcept {
    {
      auto const lock = std::lock_guard(mutex_);
      queue_.push(std::forward<U>(value));
    }
    cv_.notify_one();
  }

  /**
   * @brief Clear the queue
   */
  void clear() noexcept {
    auto const lock = std::lock_guard(mutex_);

    // Swap queue with an empty queue of the same type to ensure queue_ is left in a
    // default-constructed state
    decltype(queue_)().swap(queue_);
  }

  /**
   * @brief Wait for given duration then pop from the queue and return the element
   * @param wait_time Maximum time to wait for queue to be non-empty
   * @return Data popped from the queue or error
   */
  [[nodiscard]] auto pop(std::chrono::nanoseconds wait_time = {}) -> std::optional<T> {
    auto lock = std::unique_lock(mutex_);

    // If queue is empty after wait_time, return nothing
    if (!cv_.wait_for(lock, wait_time, [this] { return !queue_.empty(); })) return std::nullopt;

    auto value = std::move(queue_.front());
    queue_.pop();
    return value;
  }
};

struct position_t {
  double x, y;
};

std::ostream& operator<<(std::ostream& os, position_t p) {
  os << "{" << p.x << ", " << p.y << "}";
  return os;
}

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
std::ostream& operator<<(std::ostream& os, tl::expected<bin_occupancy, std::string> const& bin_maybe) {
  if (!bin_maybe.has_value()) {
    os << bin_maybe.error();
    return os;
  }
  auto const& state = bin_maybe.value();
  switch (state) {
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

bool same(position_t const& lhs, position_t const& rhs) {
  return near(lhs.x, rhs.x) && near(lhs.y, rhs.y);
}

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
  constexpr double waypoint_offset = 0.02;
  auto const offset = std::copysign(waypoint_offset, model.origin.x - p.x);
  return {p.x + offset, p.y};
}

// function that takes waypoints and linearly interpolates between them
path_t plan_cartesian(path_t const& waypoints, double step = 0.1) {
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
    auto const line = lerp(start, goal, step);
    // Do not add the first point as it is already in the path
    path.insert(path.end(), ++line.begin(), line.end());
  }
  return path;
}

// function that takes a goal and arm and returns a plan
std::vector<joint_angles> plan_joint(path_t const& waypoints, kinematics_t const& model) {
  if (waypoints.empty()) {
    return {};  // Convert to std::expected
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

std::string serialize_mock([[maybe_unused]] std::array<int, 2> const& joint_angle) { return ""; }

std::string serialize_newline(std::array<int, 2> const& joint_angle) {
  return std::to_string(joint_angle[0]) + ", " + std::to_string(joint_angle[1]) + "\n";
}

std::string serialize_brace(std::array<int, 2> const& joint_angle) {
  return "{" + std::to_string(joint_angle[0]) + ", " + std::to_string(joint_angle[1]) + "}";
}

struct hardware_interface {
  virtual ~hardware_interface() = default;
  virtual tl::expected<bin_occupancy, std::string> operator()(
      std::vector<joint_angles> const& path) const = 0;
};

struct mock_hardware : public hardware_interface {
  explicit mock_hardware(kinematics_t model) : model_(std::move(model)) {}
  tl::expected<bin_occupancy, std::string> operator()(std::vector<joint_angles> const& path) const {
    // Seed with a real random value, if available
    std::random_device rd;
    std::mt19937 gen(rd());

    // Generate a random number between min_ms and max_ms
    std::uniform_int_distribution<> dist(500, 1000);

    // Get a random sleep duration in milliseconds
    int sleep_duration = dist(gen);

    std::cout << model_.description << "\n";
    std::cout << "sleeping for " << sleep_duration << " milliseconds." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_duration));
    std::cout << "x, y\n";
    // std::ranges::for_each(path, [&](auto const& j) {
      // auto const p = forward_kinematics(j, model_);
      // std::cout << p.x << ", " << p.y << "\n";
    // });
    std::vector<std::array<int, 2>> path_degrees;
    std::ranges::transform(
        path, std::back_inserter(path_degrees), [](auto const& j) -> std::array<int, 2> {
          return {static_cast<int>(to_degrees(j[0])), static_cast<int>(to_degrees(j[1]))};
        });
    std::ranges::for_each(path_degrees, [this](auto const& joint_angle) {
      std::cout << serialize_newline(joint_angle);
    });
    return bin_occupancy::EMPTY;
  };
  kinematics_t model_;
};

// TODO: in arduino, need a joint angle to pwm function
struct hardware : public hardware_interface {
  hardware(std::string port, uint32_t baudrate,
           serial::Timeout timeout = serial::Timeout::simpleTimeout(10000))
      : serial_(std::move(port), baudrate, std::move(timeout)) {}

  tl::expected<bin_occupancy, std::string> operator()(std::vector<joint_angles> const& path) const {
    if (!serial_.isOpen()) {
      return tl::make_unexpected("Error opening serial port");
    }
    // Convert the angles to degrees for arduino, rounded
    std::vector<std::array<int, 2>> path_degrees;
    std::ranges::transform(
        path, std::back_inserter(path_degrees), [](auto const& j) -> std::array<int, 2> {
          return {static_cast<int>(to_degrees(j[0])), static_cast<int>(to_degrees(j[1]))};
        });
    std::ranges::for_each(path_degrees, [this](auto const& joint_angle) {
      // serial_.write(serialize_newline(joint_angle));
      auto ser = serialize_newline(joint_angle);
      // std::cout << "serialized = " << ser;
      serial_.write(ser);
    });
    try {
      // Send read sensor command so there isn't ambiguity about return values from the serial
      serial_.write("1000, 1000\n");
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

struct arbiter_single {
  arbiter_single(kinematics_t model, std::unique_ptr<hardware_interface> hw)
      : model_{std::move(model)},
        state_{0., 0.},
        home_{forward_kinematics(state_, model_)},
        command_path_{std::move(hw)} {
    // Command the robot to home
    (*command_path_)({state_});
  }

  tl::expected<bin_occupancy, std::string> operator()(position_t const& goal) {
    auto const near_goal = generate_waypoint(goal, model_);
    auto const goto_near = inverse_kinematics(plan_cartesian({home_, near_goal}, 0.05), model_);
    // std::cout << "goto near\n";
    (*command_path_)(goto_near);
    // std::cout << "### wentto near\n";
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    auto const goto_goal = inverse_kinematics(plan_cartesian({near_goal, goal}, 0.1), model_);
    // std::cout << "goto goal\n";
    auto const sensor_reading_maybe = (*command_path_)(goto_goal);
    // std::cout << "### wentto goal\n";
    // std::cout << "reading = " << sensor_reading_maybe << std::endl;
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    // Out current cartesian position should be at the goal
    auto const goto_home = inverse_kinematics(plan_cartesian({goal, near_goal, home_}, 0.05), model_);
    // std::cout << "goto home\n";
    (*command_path_)(goto_home);
    // std::cout << "### wentto home\n";
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return sensor_reading_maybe;
  }
  // tl::expected<bin_occupancy, std::string> operator()(position_t const& goal) {
  //   auto const goto_goal = inverse_kinematics(goal, model_);
  //   std::cout << "goto goal = " << goto_goal[0] << ", " << goto_goal[1] << "\n";
  //   auto const sensor_reading_maybe = (*command_path_)({goto_goal});
  //   std::cout << "### wentto goal\n";
  //   std::cout << "reading = " << sensor_reading_maybe << std::endl;
  //   std::this_thread::sleep_for(std::chrono::seconds(1));
  //   return sensor_reading_maybe;
  // }

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
    // Command the robots to home
    (*hw_.at(model_[0].description))({{0., 0.}});
    (*hw_.at(model_[1].description))({{0., 0.}});
  }

  tl::expected<bin_occupancy, std::string> operator()(position_t const& goal) {
    auto const model = which_arm_parition(goal, model_);
    auto& command_path = *hw_.at(model.description);
    auto const home = home_.at(model.description);
    auto const near_goal = generate_waypoint(goal, model);
    auto const goto_near = inverse_kinematics(plan_cartesian({home, near_goal}, 0.05), model);
    command_path(goto_near);
    auto const goto_goal = inverse_kinematics(plan_cartesian({near_goal, goal}, 0.1), model);
    auto const sensor_reading_maybe = command_path(goto_goal);
    // Out current cartesian position should be at the goal
    auto const goto_home = inverse_kinematics(plan_cartesian({goal, near_goal, home}, 0.05), model);
    command_path(goto_home);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return sensor_reading_maybe;
  }

 private:
  std::array<kinematics_t, 2> model_;
  std::map<std::string, position_t> home_;
  std::map<std::string, std::unique_ptr<hardware_interface>> hw_;
};

tl::expected<bin_state, std::string> is_bin_occupied(position_t const& goal,
                                                     kinematics_t const& model, position_t& state,
                                                     hardware_interface& hw) {
  auto const near_goal = generate_waypoint(goal, model);
  auto const goto_goal = inverse_kinematics(plan_cartesian({state, near_goal, goal}, 0.05), model);
  auto const sensor_reading_maybe = hw(goto_goal);
  auto const goto_waypoint = inverse_kinematics(plan_cartesian({goal, near_goal}, 0.05), model);
  hw(goto_waypoint);
  state = near_goal;
  if (!sensor_reading_maybe.has_value()) {
    return tl::make_unexpected(sensor_reading_maybe.error());
  }
  return bin_state{sensor_reading_maybe.value(), goal};
}

struct arbiter_dual_async {
  using result_type = tl::expected<bin_state, std::string>;
  using task_type = std::packaged_task<result_type(position_t const&, kinematics_t const&,
                                                   position_t&, hardware_interface&)>;
  arbiter_dual_async(kinematics_t model_left, std::unique_ptr<hardware_interface> hw_left,
                     kinematics_t model_right, std::unique_ptr<hardware_interface> hw_right,
                     bin_grid_t bins)
      : model_{{std::move(model_left), std::move(model_right)}},
        bins_{std::move(bins)},
        home_{{model_[0].description, forward_kinematics({0., 0.}, model_[0])},
              {model_[1].description, forward_kinematics({0., 0.}, model_[1])}},
        left_state_{forward_kinematics({0., 0.}, model_[0])},
        right_state_{forward_kinematics({0., 0.}, model_[1])} {
    hw_.emplace(model_[0].description, std::move(hw_left));
    hw_.emplace(model_[1].description, std::move(hw_right));
    // Command the robots to home
    (*hw_.at(model_[0].description))({{0., 0.}});
    (*hw_.at(model_[1].description))({{0., 0.}});
    left_engine_ = std::jthread{[this](std::stop_token stoken) {
      try {
        while (!stoken.stop_requested()) {
          // get the goal of the other arm
          auto const other_goal = right_goal();
          // Get the list of available goals
          auto const allowed_goals = available_goals(bins_, model_[0], other_goal);
          // Check if any of the goals on the queue are available
          auto work_maybe = queue_.pop(std::chrono::milliseconds{500});
          if (work_maybe.has_value()) {
            auto& [goal, task] = work_maybe.value();
            // If the goal is allowed, do it
            if (std::ranges::any_of(allowed_goals,
                                    [&](auto const& allowed) { return same(goal, allowed); })) {
              left_goal(goal);
              task(goal, model_[0], left_state_, *hw_.at(model_[0].description).get());
              // Reset to indicate that the goal is not being worked on
              left_goal({0., 0.});
            } else {
              // put it back
              queue_.push(std::move(work_maybe.value()));
            }
          }
        }
      } catch (std::exception const& e) {
        std::cerr << e.what() << std::endl;
      }
    }};
    right_engine_ = std::jthread{[this](std::stop_token stoken) {
      try {
        while (!stoken.stop_requested()) {
          // get the goal of the other arm
          auto const other_goal = left_goal();
          // Get the list of available goals
          auto const allowed_goals = available_goals(bins_, model_[1], other_goal);
          // Check if any of the goals on the queue are available
          auto work_maybe = queue_.pop(std::chrono::milliseconds{500});
          if (work_maybe.has_value()) {
            auto& [goal, task] = work_maybe.value();
            // If the goal is allowed, do it
            if (std::ranges::any_of(allowed_goals,
                                    [&](auto const& allowed) { return same(goal, allowed); })) {
              right_goal(goal);
              task(goal, model_[1], right_state_, *hw_.at(model_[1].description).get());
              // Reset to indicate that the goal is not being worked on
              right_goal({0., 0.});
            } else {
              // put it back
              queue_.push(std::move(work_maybe.value()));
            }
          }
        }
      } catch (std::exception const& e) {
        std::cerr << e.what() << std::endl;
      }
    }};
  }

  ~arbiter_dual_async() {
    // Join the threads to be sure that they aren't commanding
    // the robots before commanding them to go home
    left_engine_.request_stop();
    right_engine_.request_stop();
    if (left_engine_.joinable()) {
      left_engine_.join();
    }
    if (right_engine_.joinable()) {
      right_engine_.join();
    }
    // Command the arms to home
    auto const gohome_left = inverse_kinematics(
        plan_cartesian({left_state_, home_.at(model_[0].description)}), model_[0]);
    (*hw_.at(model_[0].description))(gohome_left);
    auto const gohome_right = inverse_kinematics(
        plan_cartesian({right_state_, home_.at(model_[1].description)}), model_[1]);
    (*hw_.at(model_[1].description))(gohome_right);
  }

  std::future<result_type> operator()(position_t goal) {
    task_type task(is_bin_occupied);
    std::future<result_type> result = task.get_future();
    queue_.push(std::make_pair(std::move(goal), std::move(task)));
    return result;
  }

  position_t left_goal() {
    std::lock_guard{left_mutex_};
    return left_goal_;
  }

  void left_goal(position_t const& value) {
    std::lock_guard{left_mutex_};
    left_goal_ = value;
  }

  position_t right_goal() {
    std::lock_guard{right_mutex_};
    return right_goal_;
  }

  void right_goal(position_t const& value) {
    std::lock_guard{right_mutex_};
    right_goal_ = value;
  }

 private:
  std::array<kinematics_t, 2> model_;
  bin_grid_t bins_;
  std::map<std::string, position_t> home_;
  std::map<std::string, position_t> goal_;
  std::map<std::string, std::unique_ptr<hardware_interface>> hw_;
  thread_safe_queue<std::pair<position_t, task_type>> queue_;
  std::mutex left_mutex_;
  std::mutex right_mutex_;
  position_t left_state_;
  position_t right_state_;
  position_t left_goal_{0., 0.};
  position_t right_goal_{0., 0.};
  std::jthread left_engine_;
  std::jthread right_engine_;
};

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
template <typename R>
bool is_ready(std::future<R> const& f) {
  return f.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready;
}

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
  bin_grid_t bin_grid{bin_positions.data()};
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
  {
    // auto arbiter = arbiter_single{left_arm, std::make_unique<mock_hardware>(left_arm)};
    // auto arbiter = arbiter_single{right_arm, std::make_unique<hardware>("/dev/ttyACM0", 9600)};
    // auto bin_checker = bin_checker_t{&arbiter};
    // auto bins = bin_view_t(bin_positions.data(), {}, bin_checker);
    // for (auto i = 0u; i != bins.extent(0); ++i) {
    //   for (auto j = 0u; j != bins.extent(1); ++j) {
    //     std::cout << bins(i, j) << "\n";
    //   }
    // }
  }
  // auto hw = hardware("/dev/ttyACM0", 9600);
  // hw({joint_angles{std::numbers::pi / 2., 0.}});
  // return 0;
  // right_arm.origin = {0.0, 0.0};
  // right_arm.orientation = 0.;
  // while (true) {
  //       std::cout << "Enter command: ";
  //   char input_char;
  //   std::cin >> input_char;
  //
  //       if (input_char == 'u') {
  //           right_arm.origin.x += 0.01;
  //       } else if (input_char == 'i') {
  //           right_arm.origin.x -= 0.01;
  //       } else if (input_char == 'j') {
  //           right_arm.origin.y += 0.01;
  //       } else if (input_char == 'k') {
  //           right_arm.origin.y -= 0.01;
  //       } else if (input_char == 'm') {
  //           right_arm.orientation += 0.1;
  //       } else if (input_char == ',') {
  //           right_arm.orientation -= 0.1;
  //       } else if (input_char == 'h') {
  //           // right_arm.orientation += 0.;
  //           right_arm.origin.x += 0.;
  //       } else if (input_char == 'x') {
  //           std::cout << "Exiting program." << std::endl;
  //           break;
  //       } else {
  //           std::cout << "Invalid input. Please press 'j', 'k', or 'x'." << std::endl;
  //           continue;
  //       }
  //       std::cout << "T = {" << right_arm.origin.x << ", " << right_arm.origin.y << ", " << right_arm.orientation << "}" << std::endl;
  //       auto arbiter = arbiter_single{right_arm, std::make_unique<hardware>("/dev/ttyACM0", 9600)};
  //       arbiter({0.210, 0.068});
  //       arbiter({0.210, 0.194});
  //       arbiter({0.273, 0.194});
  //       arbiter({0.273, 0.068});
  //   }
  /* TEST DUAL ARM SYNCHRONOUS */
  {
    // auto arbiter = arbiter_dual{left_arm, std::make_unique<hardware>("/dev/ttyACM0", 9600), right_arm,
    //                             std::make_unique<mock_hardware>(right_arm)};
    // auto bin_checker = bin_checker_t{&arbiter};
    // auto bins = bin_view_t(bin_positions.data(), {}, bin_checker);
    // for (auto i = 0u; i != bins.extent(0); ++i) {
    //   for (auto j = 0u; j != bins.extent(1); ++j) {
    //     std::cout << bins(i, j) << "\n";
    //   }
    // }
  }
  /* TEST DUAL ARM ASYNCHRONOUS */
  {
    // auto left_arm = std::make_unique<hardware>("/dev/ttyACM0", 9600);
    // auto right_arm = std::make_unique<hardware>("/dev/ttyUSB0", 9600);
    auto arbiter =
        arbiter_dual_async{left_arm, std::make_unique<hardware>("/dev/ttyACM0", 9660), right_arm,
                           std::make_unique<hardware>("/dev/ttyACM1", 9600), bin_grid};
    auto bin_checker = bin_checker_async_t{&arbiter};
    auto bins = bin_view_async_t(bin_positions.data(), {}, bin_checker);
    std::vector<std::future<tl::expected<bin_state, std::string>>> futures;
    for (auto i = 0u; i != bins.extent(0); ++i) {
      for (auto j = 0u; j != bins.extent(1); ++j) {
        futures.push_back(bins(i, j));
      }
    }

    while (!futures.empty()) {
      std::erase_if(futures, [](auto& future) {
        if (is_ready(future)) {
          std::cout << future.get() << "\n";
          return true;
        }
        return false;
      });
    }
  }
  return 0;
}
