//
// Created by viktor on 25.09.2020.
//

#ifndef MATHLABS_GRAPHICS_H
#define MATHLABS_GRAPHICS_H
#include <GL/glut.h>
#include <eigen3/Eigen/Dense>
#include "mathRB.h"
using namespace Eigen;

void reshape(GLsizei width, GLsizei height);
void display();
void initGL();


#endif //MATHLABS_GRAPHICS_H
