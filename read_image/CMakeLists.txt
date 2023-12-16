cmake_minimum_required(VERSION 3.10)


# set(OpenCV_DIR "${VCPKG_INSTALLED_DIR}/x64-windows/share/opencv4")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# find_package(OpenCV REQUIRED)
# include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(OpenGL REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIR} )

find_package(GLUT REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories( ${Eigen3_INCLUDE_DIRS} )

# find_package(matplot++ REQUIRED)

add_executable(read_image
    CG_framework.cpp
    )

target_link_libraries(read_image PRIVATE
    Eigen3::Eigen
    ${OPENGL_LIBRARIES} 
    GLUT::GLUT
    ${OpenCV_LIBS}
    )
    # Matplot++::matplot 

