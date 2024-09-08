[![ci](https://github.com/griswaldbrooks/spanny2/actions/workflows/ci.yml/badge.svg)](https://github.com/griswaldbrooks/spanny2/actions/workflows/ci.yml)
# spanny2
Robot arm project for CppCon 2024 presentation.

# Development Container
Build a new development image
```shell
mkdir -p ~/.spanny2/ccache
export UID=$(id -u) export GID=$(id -g); docker compose -f compose.dev.yml build
```
Start an interactive development container
```shell
docker compose -f compose.dev.yml run development
```
Build the repository in the container
```shell
cmake -S src/spanny2/ -B build
cmake --build build
```

# Run
```shell
username@spanny-dev:~/ws$ ./build/mappy
```

# Orphans
```shell
docker compose -f compose.dev.yml down --remove-orphans
```

# Title
spanny 2: rise of std::mdspan
# Abstract
C++23 introduced std::mdspan, a multi-dimensional view type that expanded on the capabilities of its predecessors, std::span and std::string_view, by eliminating the restriction to view into contiguous blocks of memory. Multidimensional array-type data structures are commonly used in robotics applications such as RGB and depth images, as well as occupancy grids for navigation. While these alone present a compelling alternative to hand-rolled data structures, this talk will illustrate the flexibility of the layout and accessor policy customization points included in std::mdspan, demonstrating its potential through various examples, including helping a robot arm inventory beer bottles.
This talk will review conventional use cases of std::mdspan in robotics, such as occupancy grids and submaps for determining obstacles and collision checking with polygonal footprints, as well as handling camera images retrieved after deep learning inference. Additionally, we will discuss commanding single and dual robot arms for inventory inspection using synchronous and asynchronous strategies via the std::mdspan accessor policy and std::future.

While these toy examples may have speculative applications, they highlight the flexibility and extensibility of this new data structure for novel use cases.

# Outline
## Introduction
- Brief overview of the presentation
- Introduction to std::mdspan in C++23

## Motivations for std::mdspan
- Historical background: limitations of previous non-owning types (std::span, std::string_view)
- The need for more flexible view types in C++

## Features of std::mdspan
- Multi-dimensional view type without memory layout restrictions
- Customization points: layout and accessor policies

## Conventional Use Cases in Robotics
- Occupancy grids and cost-maps
- Submaps for navigation and localization
- Polygonal views and arbitrary layouts for collision checking
- Accessor policies interfacing with GPUs (e.g., CUDA calls)

## Beer Inspection Use Cases
- Synchronous command of a single robot arm
- Synchronous command of dual robot arms
- Asynchronous command of dual robot arms for optimized path planning

## Conclusion
- Summary of mdspan features and benefits
- Discussion of use cases in fleet management and warehouse inventory control
- Using std::mdspan to access cloud resources asynchronously
- Potential impact on robotics and other fields
- Speculation on future applications and developments

## References
mdspan proposal
https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2022/p0009r18.html

Timur Doumler talk
https://youtu.be/eD-ceG-oByA?si=FYk_cG3dNPIHiF2h

Bryce Lelback talk
https://youtu.be/aFCLmQEkPUw?si=zWybvW7JAOO33yVV

Three skew rotation
https://youtu.be/1LCEiVDHJmc?si=kjbLFjp8XldHnzO1
