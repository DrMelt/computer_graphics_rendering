cmake_minimum_required(VERSION 3.10)

# find_package(OpenCV REQUIRED)
# include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(OpenGL REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIR} )

find_package(GLUT REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories( ${Eigen3_INCLUDE_DIRS} )

find_package(matplot++ REQUIRED)

add_executable(hello_world
    CG_framework.cpp
    Parameters.cpp
    )

target_link_libraries(hello_world 
    # ${OpenCV_LIBRARIES} 
    ${OPENGL_LIBRARIES} 
    Matplot++::matplot 
    Eigen3::Eigen
    GLUT::GLUT
    )

