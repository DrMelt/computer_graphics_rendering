add_executable(paint_ellipse
    CG_framework.cpp
    )

target_link_libraries(paint_ellipse PRIVATE
    Eigen3::Eigen
    ${OPENGL_LIBRARIES} 
    GLUT::GLUT
    ${OpenCV_LIBS}
    )
    # Matplot++::matplot 

