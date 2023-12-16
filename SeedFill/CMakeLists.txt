add_executable(SeedFill  Transformation_2D.cpp)

target_link_libraries(SeedFill PRIVATE
    Eigen3::Eigen
    ${OPENGL_LIBRARIES} 
    GLUT::GLUT
    # ${OpenCV_LIBS}
    )

