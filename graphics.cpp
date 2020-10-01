//
// Created by viktor on 25.09.2020.
//

#include "graphics.h"

int refreshMills = 15;
static double *y = new double[STATE_SIZE];
static double* m = new double[16];


void angleComp(double *yend, Matrix3d R0, Matrix3d Rt) {
    double cosp = 0;
    for (int i = 0; i < 3; ++i) {
        cosp = (double )(R0(0, i) * Rt(0, i) + R0(1, i) * Rt(1, i) + R0(2, i) * Rt(2, i)) /
               (sqrt(R0(0, i) * R0(0, i) + R0(1, i) * R0(1, i) + R0(2, i) * R0(2, i)) *
                sqrt(Rt(0, i) * Rt(0, i) + Rt(1, i) * Rt(1, i) + Rt(2, i) * Rt(2, i)));
        yend[i]=acos(cosp)*180/M_PI;
    }
}

void reshape(GLsizei width, GLsizei height) {  // GLsizei for non-negative integer
    // Compute aspect ratio of the new window
    if (height == 0) height = 1;                // To prevent divide by 0
    GLfloat aspect = (GLfloat) width / (GLfloat) height;

    // Set the viewport to cover the new window
    glViewport(0, 0, width, height);

    // Set the aspect ratio of the clipping volume to match the viewport
    glMatrixMode(GL_PROJECTION);  // To operate on the Projection matrix
    glLoadIdentity();             // Reset
    // Enable perspective projection with fovy, aspect, zNear and zFar
    gluPerspective(45.0f, aspect, 0.1f, 100.0f);
}

void initGL() {
    for (int i = 0; i < STATE_SIZE; ++i) {
        y[i] = 0;
    }
    for (int i = 0; i < 16; ++i) {
        m[i]=0;
    }
    m[0]=1;
    m[5]=1;
    m[10]=1;
    m[15]=1;
    InitRigidBody(pRigidBody);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Set background color to black and opaque
    glClearDepth(1.0f);// Set background depth to farthest
    glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
    glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
    glShadeModel(GL_SMOOTH);   // Enable smooth shading
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);  // Nice perspective corrections
}

void timer(int value) {
    glutPostRedisplay();      // Post re-paint request to activate display()
    glutTimerFunc(refreshMills, timer, 0); // next timer call milliseconds later
}

void display() {

    RunSimulation(pRigidBody, y);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear color and depth buffers
    glMatrixMode(GL_MODELVIEW);     // To operate on model-view matrix

    // Render a color-cube consisting of 6 quads with different colors
    glLoadIdentity();                 // Reset the model-view matrix
    for (int i = 0; i < 3; ++i) {
            m[0+i*4]=y[3+i];
            m[1+i*4]=y[6+i];
            m[2+i*4]=y[9+i];
    }


    glTranslated(y[0], y[1], -7.0 + y[2]);  // Move right and into the screen
    glMultMatrixd(m);
    //glLoadMatrixd(m);


    //поворот
    //double *angles = new double[3];
    //angleComp(angles, R0, pRigidBody->R);
    //R0=pRigidBody->R;
    //glRotated(angles[0], 0, 0, 1);

    glEnable(GL_CULL_FACE);

    glBegin(GL_QUADS);                // Begin drawing the color cube with 6 quads
    // Top face (y = 1.0f)
    // Define vertices in counter-clockwise (CCW) order with normal pointing out
    glColor3f(0.0f, 1.0f, 0.0f);     // Green
    glVertex3f(1.0f, 0.5f, -1.0f);
    glVertex3f(-1.0f, 0.5f, -1.0f);
    glVertex3f(-1.0f, 0.5f, 1.0f);
    glVertex3f(1.0f, 0.5f, 1.0f);

    // Bottom face (y = -1.0f)
    glColor3f(1.0f, 0.5f, 0.0f);     // Orange
    glVertex3f(1.0f, -0.5f, 1.0f);
    glVertex3f(-1.0f, -0.5f, 1.0f);
    glVertex3f(-1.0f, -0.5f, -1.0f);
    glVertex3f(1.0f, -0.5f, -1.0f);

    // Front face  (z = 1.0f)
    glColor3f(1.0f, 0.0f, 0.0f);     // Red
    glVertex3f(1.0f, 0.5f, 1.0f);
    glVertex3f(-1.0f, 0.5f, 1.0f);
    glVertex3f(-1.0f, -0.5f, 1.0f);
    glVertex3f(1.0f, -0.5f, 1.0f);

    // Back face (z = -1.0f)
    glColor3f(1.0f, 1.0f, 0.0f);     // Yellow
    glVertex3f(1.0f, -0.5f, -1.0f);
    glVertex3f(-1.0f, -0.5f, -1.0f);
    glVertex3f(-1.0f, 0.5f, -1.0f);
    glVertex3f(1.0f, 0.5f, -1.0f);

    // Left face (x = -1.0f)
    glColor3f(0.0f, 0.0f, 1.0f);     // Blue
    glVertex3f(-1.0f, 0.5f, 1.0f);
    glVertex3f(-1.0f, 0.5f, -1.0f);
    glVertex3f(-1.0f, -0.5f, -1.0f);
    glVertex3f(-1.0f, -0.5f, 1.0f);

    // Right face (x = 1.0f)
    glColor3f(1.0f, 0.0f, 1.0f);     // Magenta
    glVertex3f(1.0f, 0.5f, -1.0f);
    glVertex3f(1.0f, 0.5f, 1.0f);
    glVertex3f(1.0f, -0.5f, 1.0f);
    glVertex3f(1.0f, -0.5f, -1.0f);
    glEnd();

    // Render a pyramid consists of 4 triangles
    glutSwapBuffers();  // Swap the front and back frame buffers (double buffering)
    //angleCube -= 1;
    glutTimerFunc(0, timer, 0);
}