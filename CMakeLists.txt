cmake_minimum_required(VERSION 3.6)
project(mathlabs)

set(CMAKE_CXX_STANDARD 11)
set(SOURCE_FILES main.cpp graphics.cpp graphics.h mathRB.cpp mathRB.h)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})