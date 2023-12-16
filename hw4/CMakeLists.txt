add_executable(hw4 Transformation_3D.cpp)

target_link_libraries(hw4 PRIVATE
    Eigen3::Eigen
    ${OPENGL_LIBRARIES} 
    GLUT::GLUT
    # ${OpenCV_LIBS}
    )

