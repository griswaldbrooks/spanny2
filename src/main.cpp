#include <iostream>
#include "serial/serial.h"
#include "mdspan.hpp"

namespace stdex = std::experimental;

template <typename T, std::ptrdiff_t Width, std::ptrdiff_t Height>
struct robot_command_accessor {
    using data_handle_type = T*;
    using reference = T&;
    using element_type = T;

    T& access(data_handle_type ptr, std::ptrdiff_t offset) const {
        std::cout << "did access\n";
        std::cin >> ptr[offset];
        return ptr[offset];
    }
    
    std::pair<std::ptrdiff_t, std::ptrdiff_t> offset_to_coord(std::ptrdiff_t offset) const {
        std::ptrdiff_t row = offset / Width;
        std::ptrdiff_t col = offset % Width;
        return {row, col};
    }

    void send_robot_command(std::ptrdiff_t row, std::ptrdiff_t col) const {
        std::cout << "Sending robot to position corresponding to pixel (" << row << ", " << col << ")\n";
    }
};

int main(int, char **) {
  // port, baudrate, timeout in milliseconds
  auto robot_arm = serial::Serial("/dev/ttyACM0", 9600, serial::Timeout::simpleTimeout(1000));
  if(!robot_arm.isOpen()) {
    std::cout << "Error opening serial port\n";
    return -1;
  }

  size_t bytes_wrote = robot_arm.write("0");
  std::cout << "Wrote " << bytes_wrote << " bytes" << std::endl;
  auto const result = robot_arm.read(2);
  std::cout << result;

  static constexpr int width = 100;
  std::vector<int> image_data(width * width);
  using ext_t = stdex::extents<uint32_t, width, width>;
  using acc_t = robot_command_accessor<int, width, width>;
  stdex::mdspan<int, ext_t, stdex::layout_right, acc_t> image_view(image_data.data());
  // robot arm should fetch the value for the access
  std::cout << "image_view[50, 50] = " << image_view(50, 50) << "\n";
  image_view(50, 50) = 1;
  std::cout << "image_view[50, 50] = " << image_view(50, 50) << "\n";

  return 0;
}
