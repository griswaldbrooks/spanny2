## What is a layout
- maps a multidimensional index to a storage location
- may be:
    - non-contiguous
    - map multiple indices to the same storage location
    - do arbitrary compute
    - have state

rank = the number of extents
extent = the length of a dimension

- show how to implement local map/window
    - doesn't work with layout_right (and why)
    - layout_string
    - submdspan
- computers don't store things in multidimensional arrays, so every multidimensional indexing
  is a mapping into a 1d-array
- mention matrix/slide/stride vs iterator
- open source cad
- implement matrix with mdspan
- use ranges

## Accessor
- make sure that the output of the layout::mapping matches the input of the accessor::operator()
- mdspan::T type must match the accessor::element_type but the first element of the ctor
  of the mdspan must match accessor::data_handle_type


template <typename Arbiter>
using bin_view_t = std::mdspan<typename bin_checker_t<Arbiter>::element_type,
                               // We're treating our 6 bins as a 2x3 matrix
                               std::extents<std::size_t, 2, 3>,
                               // Our layout should do bounds-checking
                               bounds_checked_layout,
                               // Tell the robot to synchronously access the bin
                               bin_checker_t<Arbiter>>;
### This works
```cpp
auto arbiter = arbiter_single{std::make_unique<mock_hardware>(left_arm)};
auto bin_checker = bin_checker_t{&arbiter};
auto mapping = bounds_checked_layout::mapping<std::extents<std::size_t, 2, 3>>(std::extents<std::size_t, 2, 3>{});
auto bins = bin_view_t(bin_positions.data(), mapping, bin_checker);
```
### But so does this
```cpp
auto arbiter = arbiter_single{std::make_unique<mock_hardware>(left_arm)};
auto bin_checker = bin_checker_t{&arbiter};
auto bins = bin_view_t(bin_positions.data(), {}, bin_checker);
```
