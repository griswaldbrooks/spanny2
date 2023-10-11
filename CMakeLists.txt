cmake_minimum_required(VERSION 3.16)
project(spanny CXX)

add_subdirectory(third_party SYSTEM)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Werror -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion)
endif()

add_library(tooly INTERFACE)
add_library(spanny::tooly ALIAS tooly)
target_include_directories(tooly INTERFACE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
target_compile_features(tooly INTERFACE cxx_std_20)

add_executable(spanny
  src/main.cpp
)
target_link_libraries(spanny PRIVATE serial mdspan expected)
target_include_directories(spanny INTERFACE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
target_compile_features(spanny INTERFACE cxx_std_20)

if(NOT PROJECT_IS_TOP_LEVEL)
  return()
endif()

include(CTest)
if(BUILD_TESTING)
  add_subdirectory(test)
endif()
