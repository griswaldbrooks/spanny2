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
    - strategy pattern
- make this really short as we will go more in depth later but in short layout describe how to translate a multidimensional index to a storage location, and accessors describe how to retrive data from the storage location
- we will learn how layout and accessor policies work, what the type requirements of them are, how to implement your own custom policies, and how they leverage the full flexibility of C++

## Layout policies and how to use them 
- Explain what a layout is 
    - maps a multidimensional index to a storage location
    - computers don't store things in multidimensional arrays, so every multidimensional indexing ultimately is a mapping into a 1d-array
        - struct of arrays or array of structs
    - may be:
        - non-contiguous
        - map multiple indices to the same storage location
        - do arbitrary compute
        - have state
    - rank = the number of extents
    - extent = the length of a dimension
    - explain with ascii art 
    - briefly explain out of the box layouts, right, left, stride
        - i really like the downsampling example for a 2d stride
    - no requirement on default constructability
    - explain layout requirements being a templated mapping type with template Extents parameter
        extents_type const& extents() const
        - extents represent the dimensionality of the object ie number of dimensions (rank) and how long 
          they are
        size_type operator()(auto indices...) // you choose how many!
        size_type required_span_size() const
        - the number of contiguous elements in the referred data structure needed to contain the spanned elements
    - static constexpr bool is_always_unique()
    - bool is_unique()
        - show how a symmetric matrix index mapping and diagram as the code for a layout this early would be premature
    static constexpr bool is_always_exhaustive()
    bool is_exhaustive()
        - show how layout_strided is not exhaustive
    static constexpr bool is_always_strided()
    bool is_strided()
        - has consistent constant offsets, all strided layouts are non-exhaustive but we will show 
          examples of layouts that are not exhaustive and have no consistent stride
    https://eel.is/c++draft/mdspan.layout#reqmts
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
    - non-defaul ctor
    - skew rotation algorithm mapping
    - show layout_rotated from it's point of view
    - show layout_rotatable and all of the indices which won't map in the square to the range 
      and therefore is not exhaustive nor strided 
    - make sure the example here has a nice sharp corner near the top
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
- conclusion to this section
    - mention that we learned what a layout is and it's requirements 
    - what the out of the box layouts do 
    - how we can make our own layouts with non-default constructable
    - have state 
    - dimensionality reduction
- lead in to the next section
    - these examples still referred to elements on the stack/heap 
    - but what if our data lives somewhere else?

## Accessor policies and how to use them
- Explain what accessor policies are (reprise intro on customization points briefly)
- they take the offset from the layout and return data from a storage location
- Explain how the default accessor works ie return p[offset]
- Talk about the type requirements of an accessor, ie using element_type, reference, data_handle_type
- accessor has no requirements on default constructability
- Note there are no template requirements like the layout policy has
- Note that they are not limited to this pattern
- In fact... you can call arbitrary functions in the accessor
- Show the simple std function accessor example and how the mdspan::T must match accessor::element_type 
  but the first ctor element must match accessor::data_handle_type
  - show a hilbert matrix
  - note that accessor::reference does not need to be a & or * or even related to accessor::element_type
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
        - i mean the default one is blocking but this one takes seconds to access an element
        - show the initial version of this did not require a reference to stack/heap memory and
          was purely functional
        - this version only uses the pointed to array from the json to give the metric locations
        - but let's now talk about the anatomy of the accessor type declaration and ctor 
        - note how those two parameters, that normally would be realted, need not be 
          and are actually unrelated and depend on the accessor types and mentioned previously
        - this allows the element type to be a bin_state, but the data_handle_type to be the robot arbiter
          as with our lambda example from earlier
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
- to summarize
    - we learned what accessor policies do and how the default one works  
    - we learned the requirements of the accessor, and what aren't requirements 
    - we learned how to implement our own accessor from a very simple lambda calculation
      to something very complicated
    - we learned that accessors can return async resources because they can leverage the full flexibility of C++

## Conclusion
- in conclusion we discussed:
- what mdspan is and the motivation for the type
- we discussed the customization points of the layout and accessor policies 
    - how layouts map indices to storage locations 
    - how accessors map storage locations to storage values
- we showed the built in options for each 
    - layout_right, layout_left, layout_stride 
    - submdspan 
    - default_accessor
- we showed how you can implement your own policies 
    - reviewed policy requirements 
    - policies to not have to be default constructable 
    - policies can have state 
    - layouts can normally increase dimensionality but can do dimensionality reduction
    - layouts do not have to map contiguously and can map multiple indices to the same storage location
    - mdspan does not have to refer to continuous memory... or any memory
    - accessor::data_handle_type is not related to accessor::reference nor mdspan::element_type (accessor::element_type)
        though... I'm not sure that mdspan::element_type == accessor::element_type is really relevant
    - accessors can return futures, allowing storage access to be asynchronous

# Final slide
- code and slides on github 
- cad files linked on onshape, but I have to publish the parts list, boards, and put the parts on printables
