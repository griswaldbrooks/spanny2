// Interface to ingest data into a map
// and then present submaps based on a central pose
// basically how local costmaps work.
#include <algorithm>
#include <array>
#include <iostream>
#include <iterator>
#include "mdspan.hpp"

namespace stdex = std::experimental;

struct coordinate {
  std::size_t x, y;
};

struct window {
  std::size_t height, width;
};

// struct bounds_checked_layout_policy {
//   template <class Extents>
//   struct mapping : stdex::layout_right::mapping<Extents> {
//     using base_t = stdex::layout_right::mapping<Extents>;
//     using base_t::base_t;
//     std::ptrdiff_t operator()(auto... idxs) const {
//       [&]<size_t... Is>(std::index_sequence<Is...>) {
//         if (((idxs < 0 || idxs > this->extents().extent(Is)) || ...)) {
//           throw std::out_of_range("Invalid bin index");
//         }
//       }(std::make_index_sequence<sizeof...(idxs)>{});
//       return this->base_t::operator()(idxs...);
//     }
//   };
// };

// std::size_t coord_to_offset(coordinate const& coord) {
//     int const row = offset / width;
//     int const col = offset % width;
//     return {row, col};
// }

struct window_accessor {
  using element_type = int;
  using reference = int*;
  using data_handle_type = int*;

  reference access(data_handle_type ptr, std::ptrdiff_t offset) const {
    // Must change this so the window accessor actually travels int the 2d array in the main map
    return ptr + offset;
  }
};

using submap = stdex::mdspan<int, 
  // We're treating our 6 bins as a 3x2 matrix
  stdex::extents<uint32_t, 3, 2>,
  // Our layout should do bounds-checking
  stdex::layout_right,
  window_accessor
>;

template <std::size_t N, std::size_t M>
struct grid_2d {
  // If the grid takes an array as a pointer, then this is really just another view type.
  // Which, is maybe fine, but then it might be better for this class to own that data
  explicit grid_2d(int value = 0) {
    // initialize the mdspan
    std::fill(data_.begin(), data_.end(), value);
  }

  stdex::mdspan<int, stdex::extents<std::size_t, N, M>> map() {
    return stdex::mdspan(data_.data(), N, M);
  }

  // what we want to do here is return an mdspan that is a particular
  // window size centered around a location
  // how do you return an mdspan of a particular window?
  // do you have to specify the window size at compile time?
  // eventually, we want a version that takes a metric position rather
  // than a grid coordinate
  // stdex::mdspan<int, std::dextents<std::size_t, 2>> window(window const& window, coordinate const& coord) {
    // Check if the grid coordinates are in bounds
    // auto const offset = coord.x * N + coord.y;
    // auto first = data_.begin();
    // std::advance(first, offset);
    // return stdex::mdspan(first, window.height, window.width);
  // }

  stdex::mdspan<int, std::dextents<std::size_t, 2>> window(window const& win, coordinate const& coord) {
    // Calculate the start row and column to keep the window centered
    std::size_t start_row = (coord.x >= win.height / 2) ? coord.x - win.height / 2 : 0;
    std::size_t start_col = (coord.y >= win.width / 2) ? coord.y - win.width / 2 : 0;

    // Ensure the window does not go out of bounds
    if (start_row + win.height > N) start_row = N - win.height;
    if (start_col + win.width > M) start_col = M - win.width;

    return stdex::mdspan<int, std::dextents<std::size_t, 2>>(
      data_.data() + start_row * M + start_col, win.height, win.width);
  }

private:
  std::array<int, N*M> data_;

};

int main (int /* argc */, char** /* argv[] */) {
  auto map = grid_2d<30, 30>{};
  auto whole = map.map();
  for (auto i = 0u; i != whole.extent(0); i++) {
      for (auto j = 0u; j != whole.extent(1); j++) { 
      std::cout << whole(i, j);
    }
    std::cout << "\n";
  }
  std::cout << "===\n";
  auto window = map.window({4, 4}, {10, 10});
  std::cout << "window.extent = " << window.extent(0) << ", " << window.extent(1) << "\n";
  for (auto i = 0u; i != window.extent(0); i++) {
      for (auto j = 0u; j != window.extent(1); j++) { 
        window(i, j) = 1;
    }
  }
  // std::fill(window, 1);
  for (auto i = 0u; i != whole.extent(0); i++) {
      for (auto j = 0u; j != whole.extent(1); j++) { 
      std::cout << whole(i, j);
    }
    std::cout << "\n";
  }
  return 0;
}
