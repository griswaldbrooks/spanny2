cmake_minimum_required(VERSION 3.16)
project(mdspan CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Werror -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion)
endif()

add_library(mdspan INTERFACE)
add_library(mdspan::mdspan ALIAS mdspan)
target_include_directories(mdspan INTERFACE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>)
target_compile_features(mdspan INTERFACE cxx_std_20)
