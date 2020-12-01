cmake_minimum_required(VERSION 3.17)

add_library(cut "")

option(ENABLE_CUDA "Build with CUDA" OFF)
if(ENABLE_CUDA)
    project(cut CXX CUDA)
    target_compile_definitions(cut PUBLIC CUDA_SUPPORT)
    target_compile_features(cut PUBLIC cxx_std_14 cuda_std_14)
else()
    project(cut CXX)
    target_compile_features(cut PUBLIC cxx_std_14)
endif()

option(CUDA_ARCH "CUDA architecture(s)" 61)
if (NOT DEFINED ${CUDA_ARCH} )
    set_property(TARGET cut PROPERTY CMAKE_CUDA_ARCHITECTURE ${CUDA_ARCH})
else()
    set_property(TARGET cut PROPERTY CMAKE_CUDA_ARCHITECTURE 50 61 75)
endif()

if(ENABLE_CUDA)
    target_sources(cut PRIVATE src/extrema.cu src/sort_util.cu)
endif()
target_sources(cut PRIVATE src/cut_common.cpp src/cut_memory.cpp src/extrema.cpp src/timing.cpp)

target_include_directories(cut PUBLIC ${PROJECT_SOURCE_DIR}/include/)

set_target_properties(cut PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

include(cmake/ClangTidy.cmake)
include(cmake/CompilerWarnings.cmake)
set_project_warnings(cut)
