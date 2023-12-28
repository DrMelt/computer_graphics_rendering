add_executable(TextureMap 
    "TextureMap.cpp"
)


target_link_libraries(TextureMap PRIVATE
    Eigen3::Eigen
    ${OPENGL_LIBRARIES} 
    GLUT::GLUT
    ${OpenCV_LIBS}
    ${OIDN_LIBS}
    )

