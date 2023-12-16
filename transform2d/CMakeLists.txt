add_executable(transform2d CG_framework.cpp)

target_link_libraries(transform2d PRIVATE
    Eigen3::Eigen
    ${OPENGL_LIBRARIES} 
    GLUT::GLUT
    ${OpenCV_LIBS}
    )

