#pragma once
#include <vector>
#include <memory>

namespace tooly {
/**
 * @brief Iterates through all of the adjacent elements in an collection
 * @tparam T type held in collection
 * @tparam U type help in the resulting collection
 * @tparam F type of binary operator
 * @param begin
 * @param end
 * @param op binary operator that receives two T and returns U
 * I don't want to mutate the input range, but op should probably be allowed
 * to be stateful? or maybe it shouldn't but should we have a collector functor that is?
 */
template <typename T, typename U, typename F, size_t window_size>
auto transform_window(T const * begin, T const * end, F op) {
  // Check that there are at least window_size elements in the collection
  if constexpr (window_size > std::distance(begin, end)) {
    // don't compile?
  } 
  // Should we take a single span instead of two pointers?
  // How will U be deduced?
  // Should F be constrained using a concept or std::function?
  // Is an output parameter more appropriate here?
  // adjacent_difference?
  std::vector<U> output;
  while (begin != std::prev(end)) {
    // requires a function that can return a range of elements from
    // current to window_size only known at compile time
    // output.push_back(std::apply(op, std::range_n(begin, window_size)));
    // begin = std::next(begin);
  } 
  return output;
}

/**
 * @brief Iterates through all of the adjacent elements in an collection
 * @tparam T type held in collection
 * @tparam U type help in the resulting collection
 * @tparam F type of binary operator
 * @param begin
 * @param end
 * @param op binary operator that receives two T and returns U 
 */
template <typename T, typename U, typename F>
auto transform_adjacent(T const * begin, T const * end, F op) {
  // No references means no forwarding?
  return transform_window<2>(begin, end, op);
}


} // namespace tooly
