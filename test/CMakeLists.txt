list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)
set_target_properties(gtest PROPERTIES COMPILE_OPTIONS "" EXPORT_COMPILE_COMMANDS OFF)
set_target_properties(gtest_main PROPERTIES COMPILE_OPTIONS "" EXPORT_COMPILE_COMMANDS OFF)
set_target_properties(gmock PROPERTIES COMPILE_OPTIONS "" EXPORT_COMPILE_COMMANDS OFF)
set_target_properties(gmock_main PROPERTIES EXPORT_COMPILE_COMMANDS OFF)

enable_testing()

add_executable(test_spanny
  test_tooly.cpp
)
target_link_libraries(test_spanny PRIVATE
  GTest::gtest_main
  GTest::gmock_main
  spanny::tooly
)

include(GoogleTest)
gtest_discover_tests(test_spanny)
