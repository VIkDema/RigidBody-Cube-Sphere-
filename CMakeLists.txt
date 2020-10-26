cmake_minimum_required(VERSION 3.6)
project(Cube)

set(CMAKE_CXX_STANDARD 11)
set(SOURCE_FILES1 main.cpp graphics.cpp graphics.h mathRB.cpp mathRB.h)
set(SOURCE_FILES2 Sphere/main.cpp Sphere/comput.cpp Sphere/comput.h Sphere/drawing.cpp Sphere/drawing.h)

add_executable(${PROJECT_NAME} ${SOURCE_FILES1})
add_executable(Sphere ${SOURCE_FILES2})

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
target_link_libraries(Sphere ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})