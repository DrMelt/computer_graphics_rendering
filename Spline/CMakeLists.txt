add_executable(Spline 
    Transformation_3D.cpp
)


target_link_libraries(Spline PRIVATE
    Eigen3::Eigen
    ${OPENGL_LIBRARIES} 
    GLUT::GLUT
    # ${OpenCV_LIBS}
    )

