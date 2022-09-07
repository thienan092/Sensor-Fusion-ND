cmake_minimum_required( VERSION 3.0 )
if( 0 )
  set( CMAKE_TOOLCHAIN_FILE "C:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake" )
  set( OpenCV_DIR )
  if( NOT CMAKE_CL_64 )
    set( OpenCV_DIR "C:/dev/vcpkg/installed/x86-windows/share/opencv" )
  else()
    set( OpenCV_DIR "C:/dev/vcpkg/installed/x64-windows/share/opencv" )
  endif()

endif()

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(camera_fusion)

find_package(OpenCV 4.1 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# Executable for create matrix exercise
add_executable (3D_object_tracking src/camFusion_Student.cpp src/FinalProject_Camera.cpp src/lidarData.cpp src/matching2D_Student.cpp src/objectDetection2D.cpp)
target_link_libraries (3D_object_tracking ${OpenCV_LIBRARIES})