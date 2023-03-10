cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(lane)

add_compile_options(-fPIC)
set(CMAKE_CXX_STANDARD 17)
set(CXX_FLAGS "-std=c++17 -lstdc++fs")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O1 -Wall -g -pthread -fsanitize=address -fsanitize-recover=all  -fsanitize=leak")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -Wall -g -pthread")
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

file (GLOB_RECURSE SRC_FILES src/*.cpp)

add_executable(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME}
  # ${PYTHON_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  stdc++fs
  # tcmalloc
  # profiler
)
# target_link_libraries(${PROJECT_NAME} tcmalloc profiler) 
