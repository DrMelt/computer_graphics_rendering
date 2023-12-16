add_executable(WeilerAthertonLineClip WeilerAthertonLineClip.cpp)

target_link_libraries(WeilerAthertonLineClip PRIVATE
    Eigen3::Eigen
    ${OPENGL_LIBRARIES} 
    GLUT::GLUT
    # ${OpenCV_LIBS}
    )

