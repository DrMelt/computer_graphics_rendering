add_executable(hw11 
    "hw11.cpp"
)


target_link_libraries(hw11 PRIVATE
    Eigen3::Eigen
    ${OPENGL_LIBRARIES} 
    GLUT::GLUT
    ${OpenCV_LIBS}
    ${OIDN_LIBS}
    )

