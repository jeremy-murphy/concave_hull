cmake_minimum_required(VERSION 3.0)

project(concave_hull)

set(CMAKE_CXX_STANDARD 98)

find_package(Boost REQUIRED COMPONENTS unit_test_framework)

include_directories(${Boost_INCLUDE_DIRS}) # Doing it the easy way for now.

enable_testing()

# link_directories(${BOOST_LIBRARY_DIRS})

add_executable(test_concave_hull test_concave_hull.cpp)
target_link_libraries(test_concave_hull ${Boost_LIBRARIES})
add_test(concave_hull test_concave_hull)

add_executable(test_hemisphere test_hemisphere.cpp)
target_link_libraries(test_hemisphere ${Boost_LIBRARIES})
add_test(hemisphere test_hemisphere)
