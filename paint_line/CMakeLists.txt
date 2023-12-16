cmake_minimum_required(VERSION 3.10)

set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../vcpkg/scripts/buildsystems/vcpkg.cmake"
  CACHE STRING "Vcpkg toolchain file")

project(paint_line)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(OpenGL REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIR} )

find_package(GLUT REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories( ${Eigen3_INCLUDE_DIRS} )

# find_package(matplot++ REQUIRED)

add_executable(paint_line
    CG_framework.cpp
    )

target_link_libraries(paint_line PRIVATE
    ${OPENGL_LIBRARIES} 
    GLUT::GLUT
    # Matplot++::matplot 
    Eigen3::Eigen
    ${OpenCV_LIBRARIES} 
    )

