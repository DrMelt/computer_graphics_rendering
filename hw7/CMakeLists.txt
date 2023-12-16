add_executable(hw7 
    Transformation_3D.cpp
)


target_link_libraries(hw7 PRIVATE
    Eigen3::Eigen
    ${OPENGL_LIBRARIES} 
    GLUT::GLUT
    # ${OpenCV_LIBS}
    )

