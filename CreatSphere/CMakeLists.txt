add_executable(CreatSphere 
    Transformation_3D.cpp
)


target_link_libraries(CreatSphere PRIVATE
    Eigen3::Eigen
    ${OPENGL_LIBRARIES} 
    GLUT::GLUT
    # ${OpenCV_LIBS}
    )

