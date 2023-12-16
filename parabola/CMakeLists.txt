add_executable(parabola
    CG_framework.cpp
    )

target_link_libraries(parabola PRIVATE
    Eigen3::Eigen
    ${OPENGL_LIBRARIES} 
    GLUT::GLUT
    ${OpenCV_LIBS}
    )
    # Matplot++::matplot 

