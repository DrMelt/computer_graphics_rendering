add_executable(hw10 
    "hw10.cpp"
)


target_link_libraries(hw10 PRIVATE
    Eigen3::Eigen
    ${OPENGL_LIBRARIES} 
    GLUT::GLUT
    ${OpenCV_LIBS}
    )

