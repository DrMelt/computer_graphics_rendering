add_executable(clipBack clipBack.cpp)

target_link_libraries(clipBack PRIVATE
    Eigen3::Eigen
    ${OPENGL_LIBRARIES} 
    GLUT::GLUT
    # ${OpenCV_LIBS}
    )

