#include "drawing.h"

int main(int argc, char **argv) {

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE);
    glutInitWindowSize(1000, 900);
    glutInitWindowPosition(50, 50);
    glutCreateWindow("Window1");
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    initGL();//инициализируем экран
    glutMainLoop();//продолжает вызывать и вызывать функции отображения, а также сохраняет окно фактически открытым

    return 0;
}
