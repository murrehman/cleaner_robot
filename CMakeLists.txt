cmake_minimum_required(VERSION 3.16)
project(robot_cleaner VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

# Find necessary packages
find_package(Boost REQUIRED)
find_package(Qt6 COMPONENTS Widgets Xml REQUIRED)

# Fetch content dependencies
include(cmake/FetchDependencies.cmake)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${Boost_INCLUDE_DIRS})

# Source files
set(SOURCES
    include/visualization/viewer.hpp
    src/main.cpp
    src/transforms.cpp
    src/curvature.cpp
    src/path_metrics.cpp
    src/cleaning_area.cpp
    src/json_loader.cpp
    src/viewer.cpp
)

# Core library
add_library(robot_cleaner_core
    include/visualization/viewer.hpp
    src/transforms.cpp
    src/curvature.cpp
    src/path_metrics.cpp
    src/cleaning_area.cpp
    src/json_loader.cpp
    src/viewer.cpp
)

target_link_libraries(robot_cleaner_core PUBLIC nlohmann_json::nlohmann_json Qt6::Widgets)

# Executable
add_executable(robot_cleaner src/main.cpp)
target_link_libraries(robot_cleaner PRIVATE robot_cleaner_core)

# Tests
enable_testing()
add_subdirectory(tests)
