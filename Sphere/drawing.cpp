#include "drawing.h"

int msecs = 10;//in milliseconds
static double *y = new double[STATE_SIZE];
static double *m = new double[16];

void reshape(int width, int height){

    if (height == 0) 
        height = 1;  

    //устанавливаем область просмотра
    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
   
    gluPerspective(30.0f, (double)width/(double) height, 1.0f, 1000.0f);
    glMatrixMode(GL_MODELVIEW);//режим матрицы преобразования сцены
    glLoadIdentity();
    gluLookAt(0,100,0,0,0,0,0,0,1);//задаём положение камеры, куда смотрим, и вектор камеры, который смотрит вверх
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
    InitRigidBody(my_RigidBody);//инициализируем твёрдое тело

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClearDepth(1.0f);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glShadeModel(GL_SMOOTH);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); 
}

void timer(int value) {
    glutPostRedisplay();//перерисовывает текущее окно
    //glutTimerFunc(msecs, timer, 0);
   //RunSimulation(my_RigidBody, y);
}

void display() {

    RunSimulation(my_RigidBody, y);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);

    glLoadIdentity();//сбрасывает матрицу в состояние по умолчанию
    
    for (int i = 0; i < 3; ++i) {
            m[0+i*4]=y[3+i];
            m[1+i*4]=y[6+i];
            m[2+i*4]=y[9+i];
    }
    glTranslated(y[0], y[1], -14.0 + y[2]);//переводит вектор в матрицу
    glMultMatrixd(m);

    glEnable(GL_CULL_FACE);

    glColor3d(0.5, 1, 0.83);
   
    glutWireSphere(my_RigidBody->x0,30,36);

    glutSwapBuffers(); //меняет передний и задний буффер(в заднем обрабатывается следующий кадр)

    glutTimerFunc(msecs, timer, 0);//таймер для вызова функции
}
