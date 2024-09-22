// Interface to ingest data into a map
// and then present submaps based on a central pose
// basically how local costmaps work.
#include <opencv2/imgcodecs.hpp>

#include "expected.hpp"
#include "mdspan.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <iterator>
#include <numbers>
#include <ranges>
#include <set>
#include <string>

namespace stdex = std::experimental;

struct coordinate_t {
  std::size_t x, y;
};

struct window_t {
  std::size_t height, width;
};

namespace geometry {

struct Cell {
  int x;  ///< The x coordinate of the cell
  int y;  ///< The y coordinate of the cell
};

struct CellCompare {
  bool operator()(Cell const& lhs, Cell const& rhs) const {
    return std::tie(lhs.x, lhs.y) < std::tie(rhs.x, rhs.y);
  }
};

std::vector<std::vector<int>> bresenham_conversion(int x0, int y0, int x1, int y1) {
  // Vector of points
  std::vector<std::vector<int>> v;

  // Beginning point
  int const p0[2] = {x0, y0};

  // Ending point
  int const p1[2] = {x1, y1};

  // Change in x and y from beginning to end
  int const deltas[2] = {(x1 - x0), (y1 - y0)};

  // Absolute values of  the changes
  int const changes[2] = {abs(deltas[0]), abs(deltas[1])};

  // Lambda that emulates signum
  auto const sgn = [](auto a) { return (a > 0) - (a < 0); };

  // Which direction to step towards for the x and y functions
  int const steps[2] = {sgn(deltas[0]), sgn(deltas[1])};

  // Array that dictates the dominant direction
  int idx[2] = {0};

  if (changes[0] > changes[1]) {
    idx[0] = 0;
    idx[1] = 1;
  } else {
    idx[0] = 1;
    idx[1] = 0;
  }

  // error scaled off of change in dominant direction
  float err = static_cast<float>(changes[idx[0]]) / 2.f;

  // points that will be pushed back
  int point[2] = {p0[0], p0[1]};

  // while the dominant direction coordinate isn't equal to the ending
  // coordinate
  while (point[idx[0]] != p1[idx[0]]) {
    // Create and push back vector into final vector
    std::vector<int> const temp{point[0], point[1]};

    v.push_back(temp);

    // bresenham magic
    err -= static_cast<float>(changes[idx[1]]);

    if (err < 0) {
      point[idx[1]] += steps[idx[1]];

      err += static_cast<float>(changes[idx[0]]);
    }

    point[idx[0]] += steps[idx[0]];
  }

  // Push final point
  // v.push_back({p0[0], p1[1]});
  v.push_back({p1[0], p1[1]});

  return v;
}

std::vector<Cell> bresenham_conversion(Cell const& start, Cell const& end) {
  auto const pixels = bresenham_conversion(start.x, start.y, end.x, end.y);

  std::vector<Cell> cells;
  std::transform(std::begin(pixels), std::end(pixels), std::back_inserter(cells),
                 [](auto const& pixel) -> Cell {
                   return {pixel[0], pixel[1]};
                 });
  return cells;
}

std::vector<Cell> raytrace(Cell const& start, Cell const& end, int max_length) {
  if (max_length < 1) {
    return {};
  }
  // Get the interpolated pixels bresenham
  auto const pixels = bresenham_conversion(start, end);

  return {pixels.begin(), pixels.begin() + std::min(static_cast<int>(pixels.size()), max_length)};
}

std::vector<Cell> polygonOutline(std::vector<Cell> const& polygon, int size_x) {
  // If the input is less than or equal to one element, return the input
  if (polygon.size() <= 1) {
    return polygon;
  }

  // The output vector of cells
  std::vector<Cell> out;

  // Iterator to the next vertex
  auto next_vrtx = polygon.begin();

  for (auto const& vertex : polygon) {
    // If the next vertex is not the end iterator, advance the next_vertex to
    // point to the next iterator
    if (next_vrtx != polygon.end()) {
      std::advance(next_vrtx, 1);
    }
    // After advancing, if the iterator points to the end, go back to the
    // beginning, essentially closing the polygon
    if (next_vrtx == polygon.end()) {
      next_vrtx = polygon.begin();
    }

    // Generate the pixels from one vertex to the next
    auto temp = raytrace(vertex, (*next_vrtx), size_x);

    // Remove the last element of the vector because the next vertex will use it
    // as a starting point i.e. don't duplicate pixels
    temp.pop_back();

    // Push the pixels into the out vector
    for (auto const& pixel : temp) {
      out.push_back(pixel);
    }
  }
  return out;
}

std::vector<Cell> convexFill(std::vector<Cell> const& polygon, window_t const& bounds) {
  // we need a minimum polygon of a triangle
  if (polygon.size() < 3) return polygon;

  auto outline = polygonOutline(polygon, std::numeric_limits<int>::max());
  // Clamp each pixel to the bounds
  std::for_each(outline.begin(), outline.end(), [&](auto& pixel) {
    pixel.x = std::clamp(pixel.x, 0, static_cast<int>(bounds.width));
    pixel.y = std::clamp(pixel.y, 0, static_cast<int>(bounds.height));
  });

  std::unordered_map<int, std::pair<int, int>> extrema;

  // Find all the x values and extrema
  for (auto const pixel : outline) {
    // Check if the x value for the pixel is in extrema
    if (extrema.find(pixel.x) == extrema.end()) {
      // if it isn't in extrema push the x in and with the y value as both the
      // min and the max
      extrema[pixel.x] = {pixel.y, pixel.y};
    } else if (pixel.y < extrema.at(pixel.x).first) {  // If it get here that must mean
                                                       // the x value was found.
      // if it is in extrema, check the y value. If the y value is lower than
      // the minima, replace the first value, else if it is greater than the
      // maxima, replace the second value, else do nothing
      extrema.at(pixel.x).first = pixel.y;
    } else if (pixel.y > extrema.at(pixel.x).second) {
      extrema.at(pixel.x).second = pixel.y;
    }
  }

  // Output vector
  std::vector<Cell> out;

  // lambda function
  auto const fill_cells = [&out](auto const& extremum) {
    for (auto lower_bound = (extremum.second).first; lower_bound <= (extremum.second).second;
         lower_bound++) {
      out.push_back({extremum.first, lower_bound});
    }
  };

  // push all the cells in between the minima and maxima into the out vector
  std::for_each(extrema.begin(), extrema.end(), fill_cells);

  return out;
}

double degrees_to_radians(double degrees) { return degrees * std::numbers::pi / 180.; }

}  // namespace geometry

struct layout_rotatable {
  template <typename Extents>
  requires(Extents::rank() == 2) struct mapping {
    using extents_type = Extents;
    using size_type = typename extents_type::size_type;
    using index_type = typename extents_type::index_type;

    constexpr mapping(extents_type parent_shape, extents_type shape, coordinate_t translation,
                      double theta)
        : shape_{std::move(shape)},
          translation_{std::move(translation)},
          theta_{theta},
          parent_shape_{std::move(parent_shape)},
          span_size_{[this]() {
            // Compute span size by the maximum index created by the vertices
            size_type max_index = 0;
            for (size_type y = 0; y < shape_.extent(0); ++y) {
              for (size_type x = 0; x < shape_.extent(1); ++x) {
                auto const index = operator()(x, y);
                if (index > max_index) {
                  max_index = index;
                }
              }
            }
            return max_index + 1;
          }()} {}
    constexpr extents_type const& extents() const noexcept { return shape_; }

    constexpr size_type operator()(size_type x, size_type y) const noexcept {
      // Use the three skew algorithm to rotate the elements
      // because a normal transformation matrix produces holes and off elements
      double const alpha = -std::tan(theta_ / 2.);
      double const beta = std::sin(theta_);
      auto tx = static_cast<double>(x);
      auto ty = static_cast<double>(y);
      tx += std::round(alpha * ty);
      ty += std::round(beta * tx);
      tx += std::round(alpha * ty);
      x = static_cast<size_type>(std::round(tx)) + translation_.x;
      y = static_cast<size_type>(std::round(ty)) + translation_.y;
      return x + y * parent_shape_.extent(1);
    }

    constexpr size_type required_span_size() const noexcept { return span_size_; }

    constexpr bool is_unique() const noexcept { return true; }
    constexpr bool is_exhaustive() const noexcept { return false; }
    constexpr bool is_strided() const noexcept { return false; }

    static constexpr bool is_always_unique() noexcept { return true; }
    static constexpr bool is_always_exhaustive() noexcept { return false; }
    static constexpr bool is_always_strided() noexcept { return false; }

   private:
    extents_type shape_;
    coordinate_t translation_;
    double theta_;
    extents_type parent_shape_;
    size_type span_size_;
  };
};

template <typename T>
bool in_bounds(T const& value, T const& lo, T const& hi) {
  return lo <= value && value <= hi;
}

bool in_bounds(geometry::Cell const& value, std::dextents<std::size_t, 2> bounds) {
  return in_bounds(value.x, 0, static_cast<int>(bounds.extent(1))) &&
         in_bounds(value.y, 0, static_cast<int>(bounds.extent(0)));
}

struct layout_vertices {
  template <typename Extents>
  requires(Extents::rank() == 1) struct mapping {
    using extents_type = Extents;
    using size_type = typename extents_type::size_type;
    using index_type = typename extents_type::index_type;
    constexpr mapping(std::dextents<std::size_t, 2> parent_shape,
                      std::vector<geometry::Cell> vertices, geometry::Cell const& p, double theta)
        : parent_shape_{std::move(parent_shape)},
          vertices_{[&] {
            std::set<geometry::Cell, geometry::CellCompare> unique_vertices;
            for (auto& v : vertices) {
              auto x = static_cast<int>(v.x * std::cos(theta) - v.y * std::sin(theta) + p.x);
              auto y = static_cast<int>(v.x * std::sin(theta) + v.y * std::cos(theta) + p.y);
              v.x = x;
              v.y = y;
              if (in_bounds(v, parent_shape_)) {
                unique_vertices.insert(v);
              }
            }
            return std::vector<geometry::Cell>{unique_vertices.begin(), unique_vertices.end()};
          }()},
          extents_{vertices_.size()},
          span_size_{[this]() {
            // Compute span size by the maximum index created by the vertices
            index_type max_index = 0;
            for (const auto& coord : vertices_) {
              auto const index = static_cast<index_type>(coord.x) +
                                 static_cast<index_type>(coord.y) * parent_shape_.extent(1);
              if (index > max_index) {
                max_index = index;
              }
            }
            return max_index + 1;
          }()} {}

    constexpr extents_type const& extents() const noexcept { return extents_; }

    constexpr size_type operator()(size_type i) const noexcept {
      auto const& coord = vertices_[i];
      auto const x = static_cast<size_type>(coord.x);
      auto const y = static_cast<size_type>(coord.y);

      auto const offset = x + y * parent_shape_.extent(1);
      return offset;
    }

    constexpr size_type required_span_size() const noexcept { return span_size_; }

    static constexpr bool is_always_unique() noexcept { return true; }
    static constexpr bool is_always_exhaustive() noexcept { return false; }
    static constexpr bool is_always_strided() noexcept { return false; }

    constexpr bool is_unique() const noexcept { return true; }
    constexpr bool is_exhaustive() const noexcept { return false; }
    constexpr bool is_strided() const noexcept { return false; }

   private:
    std::dextents<std::size_t, 2> parent_shape_;
    std::vector<geometry::Cell> vertices_;
    extents_type extents_;
    size_type span_size_;
  };
};

struct layout_raytrace {
  template <typename Extents>
  requires(Extents::rank() == 1) struct mapping {
    using extents_type = Extents;
    using size_type = typename extents_type::size_type;
    using index_type = typename extents_type::index_type;

    constexpr mapping(std::dextents<std::size_t, 2> parent_shape,
                      std::vector<geometry::Cell> vertices, geometry::Cell const& p, double theta)
        : parent_shape_{std::move(parent_shape)},
          rays_{[&] {
            std::set<geometry::Cell, geometry::CellCompare> unique_cells;
            for (auto& v : vertices) {
              auto x = static_cast<int>(v.x * std::cos(theta) - v.y * std::sin(theta) + p.x);
              auto y = static_cast<int>(v.x * std::sin(theta) + v.y * std::cos(theta) + p.y);
              v.x = x;
              v.y = y;
              auto const ray = raytrace(p, v, std::numeric_limits<int>::max());
              for (auto const& pixel : ray) {
                if (in_bounds(pixel, parent_shape_)) {
                  unique_cells.insert(pixel);
                }
              }
            }
            return std::vector<geometry::Cell>{unique_cells.begin(), unique_cells.end()};
          }()},
          extents_{rays_.size()},
          span_size_{[this]() {
            // Compute span size by the maximum index created by the vertices
            index_type max_index = 0;
            for (const auto& coord : rays_) {
              auto const index = static_cast<index_type>(coord.x) +
                                 static_cast<index_type>(coord.y) * parent_shape_.extent(1);
              if (index > max_index) {
                max_index = index;
              }
            }
            return max_index + 1;
          }()} {}

    constexpr extents_type const& extents() const noexcept { return extents_; }

    constexpr size_type operator()(size_type i) const noexcept {
      auto const& coord = rays_[i];
      auto const x = static_cast<size_type>(coord.x);
      auto const y = static_cast<size_type>(coord.y);

      auto const offset = x + y * parent_shape_.extent(1);
      return offset;
    }

    constexpr size_type required_span_size() const noexcept { return span_size_; }

    static constexpr bool is_always_unique() noexcept { return true; }
    static constexpr bool is_always_exhaustive() noexcept { return false; }
    static constexpr bool is_always_strided() noexcept { return false; }

    constexpr bool is_unique() const noexcept { return true; }
    constexpr bool is_exhaustive() const noexcept { return false; }
    constexpr bool is_strided() const noexcept { return false; }

   private:
    std::dextents<std::size_t, 2> parent_shape_;
    std::vector<geometry::Cell> rays_;
    extents_type extents_;
    size_type span_size_;
  };
};

struct layout_polygonal {
  template <typename Extents>
  requires(Extents::rank() == 1) struct mapping {
    using extents_type = Extents;
    using size_type = typename extents_type::size_type;
    using index_type = typename extents_type::index_type;

    constexpr mapping(std::dextents<std::size_t, 2> parent_shape,
                      std::vector<geometry::Cell> vertices, geometry::Cell const& p, double theta)
        : parent_shape_{std::move(parent_shape)},
          polygon_{[&] {
            for (auto& v : vertices) {
              auto x = static_cast<int>(v.x * std::cos(theta) - v.y * std::sin(theta) + p.x);
              auto y = static_cast<int>(v.x * std::sin(theta) + v.y * std::cos(theta) + p.y);
              v.x = x;
              v.y = y;
            }
            // create filled footprint
            return geometry::convexFill(vertices,
                                        {parent_shape_.extent(0), parent_shape_.extent(1)});
          }()},
          extents_{polygon_.size()},
          span_size_{[this]() {
            // Compute span size by the maximum index created by the vertices
            index_type max_index = 0;
            for (const auto& coord : polygon_) {
              auto const index = static_cast<index_type>(coord.x) +
                                 static_cast<index_type>(coord.y) * parent_shape_.extent(1);
              if (index > max_index) {
                max_index = index;
              }
            }
            return max_index + 1;
          }()} {}

    constexpr extents_type const& extents() const noexcept { return extents_; }

    constexpr size_type operator()(size_type i) const noexcept {
      auto const& coord = polygon_[i];
      auto const x = static_cast<size_type>(coord.x);
      auto const y = static_cast<size_type>(coord.y);

      auto const offset = x + y * parent_shape_.extent(1);
      return offset;
    }

    constexpr size_type required_span_size() const noexcept { return span_size_; }

    static constexpr bool is_always_unique() noexcept { return true; }
    static constexpr bool is_always_exhaustive() noexcept { return false; }
    static constexpr bool is_always_strided() noexcept { return false; }

    constexpr bool is_unique() const noexcept { return true; }
    constexpr bool is_exhaustive() const noexcept { return false; }
    constexpr bool is_strided() const noexcept { return false; }

   private:
    std::dextents<std::size_t, 2> parent_shape_;
    std::vector<geometry::Cell> polygon_;
    extents_type extents_;
    size_type span_size_;
  };
};

struct occupancy_grid_t {
  occupancy_grid_t(std::dextents<std::size_t, 2> shape, unsigned char value = 127)
      : data_(shape.extent(0) * shape.extent(1), value), grid_{data_.data(), shape} {}

  occupancy_grid_t(std::dextents<std::size_t, 2> shape, std::vector<unsigned char> values)
      : data_{std::move(values)}, grid_{data_.data(), shape} {}

  std::mdspan<unsigned char, std::dextents<std::size_t, 2>, std::layout_stride>
  window_from_layout_right(std::dextents<std::size_t, 2> shape, coordinate_t upper_left) {
    auto const height = grid_.extent(0);
    auto const width = grid_.extent(1);
    // Constrain the window to the grid map
    upper_left.x = std::clamp(upper_left.x, std::size_t{0}, width);
    upper_left.y = std::clamp(upper_left.y, std::size_t{0}, height);

    // Shrink window if it goes out of bounds
    shape = std::dextents<std::size_t, 2>{
        std::clamp(shape.extent(0), std::size_t{0}, height - upper_left.y),
        std::clamp(shape.extent(1), std::size_t{0}, width - upper_left.x)};
    // spacing between elements, spacing between rows
    // 0 means all of the elements in a row are on top of each other
    // spacing between rows needs to match the width of the big map
    auto const row = upper_left.y * width;
    auto const col = upper_left.x;
    auto first = data_.data();
    std::advance(first, row + col);
    return {first, std::layout_right::mapping{shape}};
  }

  /**
   * @brief Produces a window that will read from and write to the occupancy
   * grid
   * @param size is the height and width of the window
   * @param upper_left corner of the window in the occupancy grid
   * @note
   * ┌─────────────────┐
   * │           w     │
   * │       ◄───────► │
   * │     ▲ ┌─┬─┬─┬─┐ │
   * │     │ │c│ │ │ │ │
   * │     │ ├─├─┼─┼─┤ │
   * │    h│ │ │ │ │ │ │
   * │     │ ├─┼─┼─┼─┤ │
   * │     │ │ │ │ │ │ │
   * │     ▼ └─┴─┴─┴─┘ │
   * │                 │
   * │                 │
   * └─────────────────┘
   * shows the window parameters in the context of the larger occupancy grid
   *
   * A strided window can be understood as the following:
   *
   * window<2, 4> = {{0, 1, 2, 3}, ▲
   *                 {4, 5, 6, 7}} │ m
   *                  ◄─────────►  ▼
   *                       n
   * m x n window
   * m is the number of rows in the window
   * n is the number of elements in a row
   *
   *
   * auto const layout = std::layout_stride::mapping{{n, m}, {s0, s1}};
   *
   *
   *     s0    s0                    s0
   * ┌──┬──┬──┬──┬──┬───┬──┬───┬────┬──┬────┬───┬───┐
   * │x1├─►│x2├─►│x3│...│xn│...│xn+1├─►│xn+2│...│xnm│
   * └──┴──┴──┴──┴──┴───┴──┴───┴────┴──┴────┴───┴───┘
   *    ──────────────────────►
   *               s1
   *
   * 2x4 window mapped into a 4x8 parent grid
   * with strides of:
   *
   *  s0 = 1, s1 = 8          s0 = 2, s1 = 8
   * ┌─┬─┬─┬─┬─┬─┬─┬─┐       ┌─┬─┬─┬─┬─┬─┬─┬─┐
   * │0│1│2│3│ │ │ │ │       │0│ │1│ │2│ │3│ │
   * ├─┼─┼─┼─┼─┼─┼─┼─┤       ├─┼─┼─┼─┼─┼─┼─┼─┤
   * │4│5│6│7│ │ │ │ │       │4│ │5│ │6│ │7│ │
   * ├─┼─┼─┼─┼─┼─┼─┼─┤       ├─┼─┼─┼─┼─┼─┼─┼─┤
   * │ │ │ │ │ │ │ │ │       │ │ │ │ │ │ │ │ │
   * ├─┼─┼─┼─┼─┼─┼─┼─┤       ├─┼─┼─┼─┼─┼─┼─┼─┤
   * │ │ │ │ │ │ │ │ │       │ │ │ │ │ │ │ │ │
   * └─┴─┴─┴─┴─┴─┴─┴─┘       └─┴─┴─┴─┴─┴─┴─┴─┘
   *
   *  s0 = 1, s1 = 16         s0 = 1, s1 = 6
   * ┌─┬─┬─┬─┬─┬─┬─┬─┐       ┌─┬─┬─┬─┬─┬─┬─┬─┐
   * │0│1│2│3│ │ │ │ │       │0│1│2│3│ │ │4│5│
   * ├─┼─┼─┼─┼─┼─┼─┼─┤       ├─┼─┼─┼─┼─┼─┼─┼─┤
   * │ │ │ │ │ │ │ │ │       │6│7│ │ │ │ │ │ │
   * ├─┼─┼─┼─┼─┼─┼─┼─┤       ├─┼─┼─┼─┼─┼─┼─┼─┤
   * │4│5│6│7│ │ │ │ │       │ │ │ │ │ │ │ │ │
   * ├─┼─┼─┼─┼─┼─┼─┼─┤       ├─┼─┼─┼─┼─┼─┼─┼─┤
   * │ │ │ │ │ │ │ │ │       │ │ │ │ │ │ │ │ │
   * └─┴─┴─┴─┴─┴─┴─┴─┘       └─┴─┴─┴─┴─┴─┴─┴─┘
   *
   * @returns requested window. If window would go out of bounds, a smaller
   * inbounds window is returned.
   */

  std::mdspan<unsigned char, std::dextents<std::size_t, 2>, std::layout_stride>
  window_from_layout_stride(std::dextents<std::size_t, 2> shape, coordinate_t upper_left) {
    auto const height = grid_.extent(0);
    auto const width = grid_.extent(1);
    // Constrain the window to the grid map
    upper_left.x = std::clamp(upper_left.x, std::size_t{0}, width);
    upper_left.y = std::clamp(upper_left.y, std::size_t{0}, height);

    // Shrink window if it goes out of bounds
    shape = std::dextents<std::size_t, 2>{
        std::clamp(shape.extent(0), std::size_t{0}, height - upper_left.y),
        std::clamp(shape.extent(1), std::size_t{0}, width - upper_left.x)};

    auto const strides = std::array<std::size_t, 2>{1, width};
    auto const layout = std::layout_stride::mapping{shape, strides};

    // Get pointer to starting element
    auto const row = upper_left.y * width;
    auto const col = upper_left.x;
    auto first = data_.data();
    std::advance(first, row + col);
    return {first, layout};
  }

  std::mdspan<unsigned char, std::dextents<std::size_t, 2>, std::layout_stride> window_submdspan(
      std::dextents<std::size_t, 2> shape, coordinate_t upper_left) {
    auto const height = grid_.extent(0);
    auto const width = grid_.extent(1);
    // Constrain the window to the grid map
    upper_left.x = std::clamp(upper_left.x, std::size_t{0}, width);
    upper_left.y = std::clamp(upper_left.y, std::size_t{0}, height);

    // Shrink window if it goes out of bounds
    shape = std::dextents<std::size_t, 2>{
        std::clamp(shape.extent(0), std::size_t{0}, height - upper_left.y),
        std::clamp(shape.extent(1), std::size_t{0}, width - upper_left.x)};
    // the primy layout can be used for a transformed mdspan... i think
    auto const rows = std::tuple{upper_left.y, upper_left.y + shape.extent(1)};
    auto const cols = std::tuple{upper_left.x, upper_left.x + shape.extent(0)};
    return stdex::submdspan(grid_, rows, cols);
  }

  std::mdspan<unsigned char, std::dextents<std::size_t, 2>, layout_rotatable> window_rotatable(
      std::dextents<std::size_t, 2> shape, coordinate_t upper_left, double theta) {
    layout_rotatable::mapping layout_window{grid_.extents(), shape, upper_left, theta};
    return {data_.data(), layout_window};
  }
  std::mdspan<unsigned char, std::dextents<std::size_t, 1>, layout_polygonal> window_polygonal(
      std::vector<geometry::Cell> const& vertices, geometry::Cell const& p, double theta) {
    layout_polygonal::mapping<std::dextents<std::size_t, 1>> footprint{grid_.extents(), vertices, p,
                                                                       theta};
    return {data_.data(), footprint};
  }

  std::mdspan<unsigned char, std::dextents<std::size_t, 1>, layout_vertices> window_vertices(
      std::vector<geometry::Cell> const& endpoints, geometry::Cell const& p, double theta) {
    layout_vertices::mapping<std::dextents<std::size_t, 1>> scan{grid_.extents(), endpoints, p,
                                                                 theta};
    return {data_.data(), scan};
  }

  std::mdspan<unsigned char, std::dextents<std::size_t, 1>, layout_raytrace> window_raytrace(
      std::vector<geometry::Cell> const& endpoints, geometry::Cell const& p, double theta) {
    layout_raytrace::mapping<std::dextents<std::size_t, 1>> scan{grid_.extents(), endpoints, p,
                                                                 theta};
    return {data_.data(), scan};
  }

  unsigned char* data() { return data_.data(); }

  std::dextents<std::size_t, 2> extents() { return grid_.extents(); }

 private:
  friend std::ostream& operator<<(std::ostream& os, occupancy_grid_t const& occ) {
    for (auto i = 0u; i != occ.grid_.extent(0); i++) {
      for (auto j = 0u; j != occ.grid_.extent(1); j++) {
        auto const value = occ.grid_(i, j);
        // Map value from 0-255 to grayscale range 232-255
        int const grayscale_value = 232 + (value * 24 / 256);
        std::cout << "\033[48;5;" << grayscale_value << "m  \033[0m";  // Print a grayscale block
      }
      os << "\n";
    }
    return os;
  }
  std::vector<unsigned char> data_;
  std::mdspan<unsigned char, std::dextents<std::size_t, 2>> grid_;
};

template <typename Container>
requires(Container::extents_type::rank() == 1) bool is_occupied(Container const& footprint) {
  for (auto i = 0u; i != footprint.extent(0); i++) {
    if (footprint(i) != 0) {
      return true;
    }
  }
  return false;
}

template <typename Container>
requires(Container::extents_type::rank() == 2) bool is_occupied(Container const& footprint) {
  for (auto i = 0u; i != footprint.extent(0); i++) {
    for (auto j = 0u; j != footprint.extent(1); j++) {
      if (footprint(i, j) != 0) {
        return true;
      }
    }
  }
  return false;
}

template <typename Container>
requires(Container::extents_type::rank() ==
         1) void set(Container footprint, typename Container::value_type const& value) {
  for (auto i = 0u; i != footprint.extent(0); i++) {
    footprint(i) = value;
  }
}

template <typename Container>
requires(Container::extents_type::rank() == 2)  //
    void set(Container footprint, typename Container::value_type const& value) {
  for (auto i = 0u; i != footprint.extent(0); i++) {
    for (auto j = 0u; j != footprint.extent(1); j++) {
      footprint(i, j) = value;
    }
  }
}

void set_scan(occupancy_grid_t& grid, std::vector<geometry::Cell> const& scan,
              geometry::Cell const& center, double orientation) {
  unsigned char const clear = 255;
  unsigned char const occlusion = 0;
  set(grid.window_raytrace(scan, center, orientation), clear);
  set(grid.window_vertices(scan, center, orientation), occlusion);
}

void save_occupancy_grid(std::filesystem::path const& filename, occupancy_grid_t& grid) {
  auto const shape = grid.extents();
  // Create a 8-bit unsigned single-channel grayscale cv::Mat
  cv::Mat const img(static_cast<int>(shape.extent(0)), static_cast<int>(shape.extent(1)), CV_8UC1,
                    grid.data());

  cv::imwrite(filename, img);
}
void save_occupancy_grid(
    std::filesystem::path const& filename,
    std::mdspan<unsigned char, std::dextents<std::size_t, 2>, layout_rotatable> grid) {
  auto const shape = grid.extents();
  // Create a 8-bit unsigned single-channel grayscale cv::Mat
  cv::Mat img(static_cast<int>(shape.extent(0)), static_cast<int>(shape.extent(1)), CV_8UC1);
  for (auto i = 0u; i != shape.extent(0); ++i) {
    for (auto j = 0u; j != shape.extent(1); ++j) {
      img.at<uchar>(static_cast<int>(i), static_cast<int>(j)) = grid(i, j);
    }
  }
  cv::imwrite(filename, img);
}

std::ostream& operator<<(
    std::ostream& os,
    std::mdspan<unsigned char, std::dextents<std::size_t, 2>, layout_rotatable> const& occ) {
  for (auto i = 0u; i != occ.extent(0); i++) {
    for (auto j = 0u; j != occ.extent(1); j++) {
      auto const value = occ(i, j);
      // Map value from 0-255 to grayscale range 232-255
      int const grayscale_value = 232 + (value * 24 / 256);
      std::cout << "\033[48;5;" << grayscale_value << "m  \033[0m";  // Print a grayscale block
    }
    os << "\n";
  }
  return os;
}
[[nodiscard]] tl::expected<occupancy_grid_t, std::string> load_occupancy_grid(
    std::filesystem::path const& filename) {
  if (!std::filesystem::exists(filename)) {
    return tl::unexpected{std::string{"Could not find file: "} + filename.string()};
  }
  cv::Mat image = cv::imread(filename, CV_8UC1);
  if (image.empty()) {
    return tl::unexpected{std::string{"Failed to load occupancy grid from file: "} +
                          filename.string()};
  }
  return occupancy_grid_t{
      std::dextents<std::size_t, 2>{image.rows, image.cols},
      std::vector<unsigned char>{image.begin<unsigned char>(), image.end<unsigned char>()}};
}

int main(int /* argc */, char** /* argv[] */) {
  // auto global_map = occupancy_grid_t{std::dextents<std::size_t, 1>{20, 30}, 0};
  // auto global_map = occupancy_grid_t{std::dextents<std::size_t, 2>{80, 120}, 127};
  auto global_map = occupancy_grid_t{std::dextents<std::size_t, 2>{40, 60}, 127};
  // auto global_map = occupancy_grid_t{std::dextents<std::size_t, 2>{768, 1024}, 127};
  auto map_maybe = load_occupancy_grid("/home/griswald/ws/occupancy_grid.png");
  if (!map_maybe.has_value()) {
    std::cout << map_maybe.error() << "\n";
  } else {
    global_map = map_maybe.value();
  }
  // std::cout << global_map << "\n";
  // This one doesn't work. Show this as the negative example (both layout_right
  // and layout_left)
  auto window_bad =
      global_map.window_from_layout_right(std::dextents<std::size_t, 2>{10, 10}, {10, 15});
  set(window_bad, 255);
  set(window_bad, 127);
  // std::cout << global_map << std::endl;
  // save_occupancy_grid("layout_right.png", global_map);

  // This works and uses layout_stride.
  // Show how this can also do multistride
  // Note the slide from Bryce Lelbach and the data[i * M + j] vs data[i * X + j
  // * Y] This doesn't feel quite right and need to explain why layout_left !=
  // layout_stride for this simple case
  auto window_good =
      global_map.window_from_layout_stride(std::dextents<std::size_t, 2>{10, 10}, {10, 15});
  set(window_good, 255);
  set(window_bad, 127);
  // std::cout << global_map << "\n";
  // save_occupancy_grid("layout_stride.png", global_map);
  // But for the local costmap example, using submdspan is even easier and
  // simpler to grok not needing to mess with pointer offsets
  auto local_map = global_map.window_submdspan(std::dextents<std::size_t, 2>{10, 10}, {10, 15});
  set(local_map, 255);
  set(local_map, 127);
  // std::cout << global_map << "\n";
  // save_occupancy_grid("layout_submdspan.png", global_map);

  // auto rotatable = global_map.window_rotatable(std::dextents<std::size_t, 2>{50, 50}, {2, 44},
  // geometry::degrees_to_radians(-45)); set(rotatable, 0); set(rotatable, 127); std::cout <<
  // rotatable << "\n"; std::cout << global_map << "\n"; save_occupancy_grid("layout_rotated.png",
  // rotatable); save_occupancy_grid("global_map.png", global_map);

  // Character to display that changes every loop
  // create footprint
  // rotates around upper left corner
  // std::vector<geometry::Cell> const vertices = {{0, 0}, {3, 0}, {3, 3}, {0, 3}};
  // rotates somewhere around center
  int x = 0;
  std::vector<geometry::Cell> vertices = {{-8, -8}, {8, -8}, {8, 0}, {2, 10}, {-2, 10}, {-8, 0}};
  for (auto const& angle : {0, 12, 22, 30, 45, 58, 67, 79, 90}) {
    double const theta = geometry::degrees_to_radians(angle);
    geometry::Cell const p = {30 + ++x, 20 - ++x};
    auto footprint = global_map.window_polygonal(vertices, p, theta);
    // std::cout << "footprint is_occupied = " << is_occupied(footprint) << "\n";
    set(footprint, 0);
    // std::cout << "footprint is_occupied = " << is_occupied(footprint) << "\n";

    // What's interesting is that if you try to use
    // std::fill(window.data_handle(), window.data_handle() + offset, 1);
    // then instead of it filling the submap, it just fills the linear range.
    // I guess that's because in this case, the data handle is just an int*
    // and not an iterator. So if you specified/created a type for data
    // data_handle that behaved more like an iterator, this would work. Though,
    // you would probably have to do a begin and end, and not add an offset.
    //
    // print map
    std::cout << global_map << std::endl;
    save_occupancy_grid(std::to_string(x) + ".png", global_map);
    // Clear footprint
    set(footprint, 127);
    // std::cin.get();
  }

  std::vector<geometry::Cell> laser_scan;
  for (auto const& degree : std::views::iota(-135, 135)) {
    // for (auto const& degree : std::views::iota(-10, 10)) {
    double const theta = geometry::degrees_to_radians(degree);
    double const range = 30.;
    // double const range = 300.;
    laser_scan.emplace_back(static_cast<int>(range * std::cos(theta)),
                            static_cast<int>(range * std::sin(theta)));
  }

  auto const center = geometry::Cell{30, 10};
  // auto const center = geometry::Cell{500, 300};
  double const orientation = 2.;
  set_scan(global_map, laser_scan, center, orientation);
  // std::cout << global_map << std::endl;
  // save_occupancy_grid("occupancy_grid_lidar.png", global_map);
  return 0;
}
