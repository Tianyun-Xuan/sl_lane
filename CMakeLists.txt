cmake_minimum_required(VERSION 3.12 FATAL_ERROR)
project(lane)

add_compile_options(-fPIC)
set(CMAKE_CXX_STANDARD 17)
set(CXX_FLAGS "-std=c++17 -lstdc++fs")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O1 -Wall -g -pthread -fsanitize=address -fsanitize-recover=all  -fsanitize=leak")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fopenmp -Wall -g -pthread")
set(CMAKE_BUILD_TYPE "Debug")
set(PACKAGE_VERSION 1.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3)
find_package(yaml-cpp)
find_package(OpenCV REQUIRED)
# find_package(PythonLibs REQUIRED)

# include directories
include_directories(../include)
include_directories(
	include
  # ${PYTHON_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

###########
## Build ##
###########

file (GLOB_RECURSE SRC_FILES src/geometry_util.cpp
                              src/lane.cpp
                              src/loss.cpp 
                              src/sg_filter.cpp)

add_library(lanefitting STATIC ${SRC_FILES})
target_link_libraries(lanefitting
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  stdc++fs
)

add_executable(${PROJECT_NAME} ./src/main.cpp)
target_link_libraries(${PROJECT_NAME} lanefitting)


## Test ##
###########
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
enable_testing()

# geometry unit-test
add_executable(test_loss ./src/test/test_loss.cpp)
target_link_libraries(test_loss GTest::gtest_main lanefitting)