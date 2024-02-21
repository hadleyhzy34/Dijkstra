cmake_minimum_required(VERSION 3.0.0)
project(
  Dijkstra
  VERSION 0.1.0
  LANGUAGES C CXX)

include(GNUInstallDirs)
set(CMAKE_CXX_STANDARD 14)
set(PACKAGE_NAME matplotlib_cpp)
set(INSTALL_CONFIGDIR${CMAKE_INSTALL_LIBDIR}/${PACKAGE_NAME}/cmake)
set(Python3_FIND_STRATEGY LOCATION)
set(Python3_ROOT_DIR "/home/hadley/anaconda3/envs/new")

# debug mode
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
# set_property(TARGET tgt PROPERTY CXX_STANDARD 17)
message('${PROJECT_SOURCE_DIR}')

# Library target
add_library(matplotlib INTERFACE)
target_include_directories(
  matplotlib INTERFACE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
target_compile_features(matplotlib INTERFACE cxx_std_17)

find_package(
  Python3
  COMPONENTS Interpreter Development
  REQUIRED)
target_link_libraries(matplotlib INTERFACE Python3::Python Python3::Module)
find_package(Python3 COMPONENTS NumPy)

if(Python3_NumPy_FOUND)
  target_link_libraries(matplotlib INTERFACE Python3::NumPy)
else()
  target_compile_definitions(matplotlib INTERFACE WITHOUT_NUMPY)
endif()

include(CTest)
enable_testing()

add_library(plan src/Dijsktra.h src/Dijsktra.cpp)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} plan matplotlib)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
# set_target_properties(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY
# "${CMAKE_BINARY_DIR}/bin")
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
