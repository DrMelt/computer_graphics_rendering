add_executable(transform3d Transformation_3D.cpp)

target_link_libraries(transform3d PRIVATE
    Eigen3::Eigen
    ${OPENGL_LIBRARIES} 
    GLUT::GLUT
    # ${OpenCV_LIBS}
    )

