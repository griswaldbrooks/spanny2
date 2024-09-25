#include <tuple>
#include <ranges>

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


std::vector<Cell> convexFill(std::vector<Cell> const& polygon) {
  // we need a minimum polygon of a triangle
  if (polygon.size() < 3) return polygon;

  auto outline = polygonOutline(polygon, std::numeric_limits<int>::max());
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

// TODO: window_t being size_t and Cell being int means that the bounds can be specified as too large, causing int wrap around.
//       it also means that lower bound can't be negative, though that is already a limit of the type.
//       The bounding feature is needed by the maps but size_t::max() causes a bug
std::vector<Cell> convexFill(std::vector<Cell> const& polygon, window_t const& bounds) {
  auto cells = convexFill(polygon);
  // Clamp each pixel to the bounds
  std::ranges::for_each(cells, [&](auto& pixel) {
    pixel.x = std::clamp(pixel.x, 0, static_cast<int>(bounds.width));
    pixel.y = std::clamp(pixel.y, 0, static_cast<int>(bounds.height));
  });
  return cells;
}

double degrees_to_radians(double degrees) { return degrees * std::numbers::pi / 180.; }

}  // namespace geometry
