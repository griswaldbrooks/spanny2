# What am I trying to convey?
I want people to learn how to write their own layouts and accesors
I want them to know that in the context of a robotics application 
I want them to learn that they can write stateful layouts and async accessors
While the typical use case would be to write pure functional layouts and stack/heap based
accessors (think something like std::func always applied to the data before being accessed)
that they are not limited and can leverage the full capabilities of C++

# Outline
## Introduction
- Brief overview of the presentation
- Introduction to std::mdspan in C++23

## Motivations for std::mdspan
- Historical background: limitations of previous non-owning types (std::span, std::string_view)
- The need for more flexible view types in C++
- Reference previous talks by Bryce and Timur 
- Reference Daisy

## Features of std::mdspan
- Multi-dimensional view type without memory layout restrictions
- Customization points: layout and accessor policies

## Layout policies and how to use them 
- Explain what a layout is 
    - index to memory location mapping 
    - explain with ascii art 
    - explain layout requirements being a templated mapping type with template Extents parameter
        extents_type const& extents() const
        size_type operator()(auto indices...) // you choose how many!
        size_type required_span_size() const
    - static constexpr bool is_always_unique()
    - bool is_unique()
        - use timur's example of when this is false
    static constexpr bool is_always_exhaustive()
    bool is_exhaustive()
        - compare layout
    static constexpr bool is_always_strided()
    bool is_strided()
    https://eel.is/c++draft/mdspan.layout#reqmts
    - no requirement on default constructability
    - briefly explain out of the box layouts, right, left, stride
        - i really like the downsampling example for a 2d stride
- Occupancy grids
    - briefly explain occupancy grid and the trinary scheme
    - show how to implement "global" occupancy grid with std::vector
    - this is a convention use case
    - make a relatively coarse map so you can still make out the pixels
      but more realistic walls, clear/unk/occupied areas
- Submaps for navigation
    - explain submap/subgrid
        - I think a good visual here would be for use to take our global map 
          and cut out a chunk in gimp, and zoom in on it as the area to explore
    - try to implement with layout_right, show what it does instead of what you think
        - would be great to highlight the pieces layout_right assigned to vs what we wanted
    - show layout_stride and how to implement submap
    - show how submdspan makes this even easier
- Rotated Submaps
    - submaps as defined above are always axis aligned with the global map 
    - this makes it less intuitive for viewing things from the perspective of the robot 
    - this also means that depending on the orientation of the robot within the frame 
      the number of occupancies in front of the robot are not fixed
    - if the submap could follow the orientation of the robot this wouldn't be a problem
    - this means we get to define our own layout!
    - which ends up being an exercise in indice mapping 
    - skew rotation algorithm mapping
    - show layout_rotated from it's point of view
- Polygonal views and arbitrary layouts 
    - The above example shows the power of writing functional non-square layout, we can go further
    - We can generate a set of indices, much like you might do with the std::cartesian example 
    - And here we are maintaining state within the layout 
    - while this doesn't rise to the level of statefulness with hysterisis for example 
    - we are storing a new set of indicies
    - unfortunately for this example we destroy the ability to reason about the elements 2d 
      and resort to 1d iterators 
    - we can do layout_polygonal, layout_raytrace, layout_vertices
    - this allows me to add obstacles to my map 
    - collision check my robot footprint
    - clear and mark lidar points
- conclusion to this section?
- lead in to the next section?

## Accessor policies and how to use them
- Explain what accessor policies are (reprise intro on customization points briefly)
- Explain how the default accessor works ie return p[offset]
- Talk about the type requirements of an accessor, ie using element_type, reference, data_handle_type
- Note there are no template requirements like the layout policy has
- Note that they are not limited to this pattern
- In fact... you can call arbitrary functions in the accessor
- Show the simple std function accessor example and how the mdspan::T must match accessor::element_type 
  but the first ctor element must match accessor::data_handle_type
  - note that accessor::reference does not need to be a & or * or even related to accessor::element_type
- accessor has no requirements on default constructability
- Daisy Hollman said... Accessor policies interfacing with GPUs (e.g., CUDA calls)
- But let's see what we can do 
- I don't work on GPUs, but I do work on robots
- Lets envision a 2x3 array allocated not on the stack or the heap, but on the table
- Describe the perennial problem of roboticists not having enough beer
    - show the willow garage beer run 
    - mit csail ariel anders beer run 
    - other
- Synchronous command of a single robot arm
    - in that spirit, I thought that having a robot to check on my beer supply would be appropriate
    - explain the problem 
    - explain the serial port communication 
    - explain that there is a planner but that is left as an exercise to the reader
        - also here's the github
    - what's different from the default accessor policy
        - first of all, calling a blocking function in an accessor
        - show the initial version of this did not require a reference to stack/heap memory and
          was purely functional
        - this version only uses the pointed to array from the json to give the metric locations
        - but let's not the anatomy of the accessor type declaration and ctor 
        - note how those two parameters, that normally would be realted, need not be 
          and are actually unrelated and depend on the accessor types
        - this allows the element type to be a bin_state, but the data_handle_type to be the robot arbiter
        - so now we can access the elements, which causes us to command the robot arm to check the appropriate bin
- Synchronous command of dual robot arms
    - problem: this seems slow and inefficient... but what if we added another robot arm 
    - in fact, we don't even have to modify the mdspan to do so!
    - demo
    - hmmm, that seems inefficient as well, seeing as the robot arms block each other...
    - what if they didn't!
- Asynchronous command of dual robot arms for optimized path planning
    - reiterating that the accessor::reference type can be anything, meaning it can be a std::future
    - show the arbiter using std::jthread and std::packaged_task with a thread_safe_queue
    - now, the accessor is calling functions that create std::futures!
    - this isn't really anything special about accessors as they are just types with a few requirements

## Conclusion
- Summary of mdspan features and benefits
- Discussion of use cases in fleet management and warehouse inventory control
- Using std::mdspan to access cloud resources asynchronously
- Potential impact on robotics and other fields
- Speculation on future applications and developments

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
