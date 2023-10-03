cmake_minimum_required(VERSION 3.16)
project(expected CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Werror -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion)
endif()

add_library(expected INTERFACE)
add_library(expected::expected ALIAS expected)
target_include_directories(expected INTERFACE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>)
target_compile_features(expected INTERFACE cxx_std_20)
