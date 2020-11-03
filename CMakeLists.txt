cmake_minimum_required(VERSION 3.10)

option(CUDA "Build with CUDA" FALSE)
if(CUDA)
    project(cut CXX CUDA)
else()
    project(cut CXX)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CUDA_STANDARD 14)

add_library(cut src/cut_common.cpp src/cut_memory.cpp src/extrema.cpp src/extrema.cu src/sort_util.cu src/timing.cpp)
target_include_directories(cut PRIVATE ${PROJECT_SOURCE_DIR}/include/)
set_target_properties(cut PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

include(cmake/ClangTidy.cmake)
include(cmake/CompilerWarnings.cmake)
set_project_warnings(cut)