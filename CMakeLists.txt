cmake_minimum_required(VERSION 3.0)

project(concave_hull)

set(CMAKE_CXX_STANDARD 98)

find_package(Boost REQUIRED)
find_package(GTest REQUIRED)

include_directories(${Boost_INCLUDE_DIRS}) # Doing it the easy way for now.

# add_executable(concave_hull main.cpp)
# install(TARGETS concave_hull RUNTIME DESTINATION bin)

enable_testing()

include_directories($GTEST_INCLUDE_DIRS)

add_executable(test_concave_hull test.cpp)
target_link_libraries(test_concave_hull ${GTEST_BOTH_LIBRARIES})
add_test(concave_hull test_concave_hull)
