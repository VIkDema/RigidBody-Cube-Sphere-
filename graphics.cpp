//
// Created by viktor on 25.09.2020.
//

#include "graphics.h"

int refreshMills = 15;
static double *y = new double[STATE_SIZE];
static double* m = new double[16];


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
    //gluPerspective(45.0f, aspect, 0.1f, 100.0f);
    gluPerspective(30.0f, aspect, 1.0f, 1000.0f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0,100,0,0,0,0,0,0,1);
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
   // glutTimerFunc(refreshMills, timer, 0); // next timer call milliseconds later
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
    glTranslated(y[0], y[1], -14 + y[2]);  // Move right and into the screen
    glMultMatrixd(m);
    //glLoadMatrixd(m);


    glEnable(GL_CULL_FACE);

    glBegin(GL_QUADS);                // Begin drawing the color cube with 6 quads
    // Top face (y = 1.0f)
    // Define vertices in counter-clockwise (CCW) order with normal pointing out
    glColor3d(0.0, 1.0, 0.0);     // Green
    glVertex3d(pRigidBody->x0, pRigidBody->y0, -pRigidBody->z0);
    glVertex3d(-pRigidBody->x0, pRigidBody->y0, -pRigidBody->z0);
    glVertex3d(-pRigidBody->x0, pRigidBody->y0, pRigidBody->z0);
    glVertex3d(pRigidBody->x0, pRigidBody->y0, pRigidBody->z0);

    // Bottom face (y = -1.0f)
    glColor3d(1.0, 0.5, 0.0);     // Orange
    glVertex3d(pRigidBody->x0, -pRigidBody->y0, pRigidBody->z0);
    glVertex3d(-pRigidBody->x0, -pRigidBody->y0, pRigidBody->z0);
    glVertex3d(-pRigidBody->x0, -pRigidBody->y0, -pRigidBody->z0);
    glVertex3d(pRigidBody->x0, -pRigidBody->y0, -pRigidBody->z0);

    // Front face  (z = 1.0f)
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(pRigidBody->x0, pRigidBody->y0, pRigidBody->z0);
    glVertex3d(-pRigidBody->x0, pRigidBody->y0, pRigidBody->z0);
    glVertex3d(-pRigidBody->x0, -pRigidBody->y0, pRigidBody->z0);
    glVertex3d(pRigidBody->x0, -pRigidBody->y0, pRigidBody->z0);



    // Back face (z = -1.0f)
    glColor3d(1.0, 1.0, 0.0);     // Yellow
    glVertex3d(pRigidBody->x0, -pRigidBody->y0, -pRigidBody->z0);
    glVertex3d(-pRigidBody->x0, -pRigidBody->y0, -pRigidBody->z0);
    glVertex3d(-pRigidBody->x0, pRigidBody->y0, -pRigidBody->z0);
    glVertex3d(pRigidBody->x0, pRigidBody->y0, -pRigidBody->z0);

    // Left face (x = -1.0f)
    glColor3d(0.0, 0.0, 1.0);     // Blue
    glVertex3d(-pRigidBody->x0, pRigidBody->y0, pRigidBody->z0);
    glVertex3d(-pRigidBody->x0, pRigidBody->y0, -pRigidBody->z0);
    glVertex3d(-pRigidBody->x0, -pRigidBody->y0, -pRigidBody->z0);
    glVertex3d(-pRigidBody->x0, -pRigidBody->y0, pRigidBody->z0);


    // Right face (x = 1.0f)
    glColor3d(-1.0, 0.0, 1.0);     // Magenta
    glVertex3d(pRigidBody->x0, pRigidBody->y0, -pRigidBody->z0);
    glVertex3d(pRigidBody->x0, pRigidBody->y0, pRigidBody->z0);
    glVertex3d(pRigidBody->x0, -pRigidBody->y0, pRigidBody->z0);
    glVertex3d(pRigidBody->x0, -pRigidBody->y0, -pRigidBody->z0);
    glEnd();

    // Render a pyramid consists of 4 triangles
    glutSwapBuffers();  // Swap the front and back frame buffers (double buffering)
    //angleCube -= 1;
    glutTimerFunc(refreshMills, timer, 0);
}