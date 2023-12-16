add_executable(transform2dHW Transformation_2D.cpp)

target_link_libraries(transform2dHW PRIVATE
    Eigen3::Eigen
    ${OPENGL_LIBRARIES} 
    GLUT::GLUT
    # ${OpenCV_LIBS}
    )

