//
// Created by viktor on 25.09.2020.
//

#include <iostream>
#include "mathRB.h"


void Array_to_State(RigidBody *rb, double *y) {
    rb->x[0] = *y++;
    rb->x[1] = *y++;
    rb->x[2] = *y++;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            rb->R(i, j) = *y++;
    rb->P[0] = *y++;
    rb->P[1] = *y++;
    rb->P[2] = *y++;
    rb->L[0] = *y++;
    rb->L[1] = *y++;
    rb->L[2] = *y++;
/* Compute auxiliary variables... */
/* v(t) = P(t)/M */
    rb->v = rb->P * (1 / rb->mass);
/* I−1(t) = R(t)I−1bodyR(t)T*/
    rb->Iinv = rb->R * rb->Ibodyinv * rb->R.transpose();
/* ω(t) = I−1(t)L(t) */
    rb->omega = rb->Iinv * rb->L;

}

void State_to_Array(RigidBody *rb, double *y) {
    *y++ = rb->x[0]; /* x component of position */
    *y++ = rb->x[1]; /* etc. */
    *y++ = rb->x[2];
    for (int i = 0; i < 3; i++) /* copy rotation matrix */
        for (int j = 0; j < 3; j++)
            *y++ = rb->R(i, j);
    *y++ = rb->P[0];
    *y++ = rb->P[1];
    *y++ = rb->P[2];
    *y++ = rb->L[0];
    *y++ = rb->L[1];
    *y++ = rb->L[2];
}

//вычисляет силу и крутящий момент
void Compute_Force_and_Torque(double t, RigidBody *rb) {
    rb->force = {0, 0, 0};
    Vector3d r = {0, 0, 0};

    rb->torque = rb->force.cross(r);
}


Matrix3d Star(Vector3d a) {
    Matrix3d matrix_star;
    matrix_star << 0, -a[2], a[1],
            a[2], 0, -a[0],
            -a[1], a[0], 0;
    return matrix_star;
}


//заносит производную в y
void ddt_State_to_Array(RigidBody *rb, double *ydot) {
/* copy d
dt x(t) = v(t) into ydot */
    *ydot++ = rb->v[0];
    *ydot++ = rb->v[1];
    *ydot++ = rb->v[2];
/* Compute R˙(t) = ω(t)∗R(t) */
    Matrix3d Rdot = Star(rb->omega) * rb->R;
/* copy R˙(t) into array */
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            *ydot++ = Rdot(i, j);
    *ydot++ = rb->force[0]; /* d
dt P(t) = F(t) */
    *ydot++ = rb->force[1];
    *ydot++ = rb->force[2];
    *ydot++ = rb->torque[0]; /* d
dt L(t) = τ(t) */
    *ydot++ = rb->torque[1];
    *ydot++ = rb->torque[2];
}

//вычисляет производную в момент времени t
void dydt(double t, double y[], double ydot[], RigidBody *rb) {
/* put data in y[ ] into Bodies[ ] */

    Compute_Force_and_Torque(t, rb);
    ddt_State_to_Array(rb, ydot);
}

//функция
void ode(double y0[], double yend[], int len, double t0,
         double t1, dydt_func dydt, RigidBody *rb) {
    State_to_Array(rb, y0);
    double Y1[len];
    double Y1withy0[len];
    double Y2[len];
    double Y2withy0[len];
    double Y3[len];
    double Y3withy0[len];
    double Y4[len];
    double Y4withy0[len];
    dydt(t0, y0, Y1, rb);
    for (int i = 0; i < len; ++i) {
        Y1[i] = (t1 - t0) * Y1[i];
        Y1withy0[i] = y0[i] + Y1[i] / 2.0;
    }
    State_to_Array(rb, Y1withy0);
    dydt(t0 + (t1 - t0) / 2, Y1withy0, Y2, rb);
    for (int i = 0; i < len; ++i) {
        Y2[i] = (t1 - t0) * Y2[i];
        Y2withy0[i] = y0[i] + Y2[i] / 2.0;
    }
    State_to_Array(rb, Y2withy0);
    dydt(t0 + (t1 - t0) / 2, Y2withy0, Y3, rb);
    for (int i = 0; i < len; ++i) {
        Y3[i] = (t1 - t0) * Y3[i];
        Y3withy0[i] = y0[i] + Y3[i];
    }
    State_to_Array(rb, Y3withy0);
    dydt(t0 + (t1 - t0) / 2, Y3withy0, Y4, rb);
    for (int i = 0; i < len; ++i) {
        Y4[i] = (t1 - t0) * Y4[i];
    }

    for (int i = 0; i < len; ++i) {
        yend[i] = y0[i] + (Y1[i] + 2 * Y2[i] + 2 * Y3[i] + Y4[i]) / 6.0;
    }
}

void printInvariant(RigidBody *rb) {
    std::cout << (0.5) * rb->mass * rb->v.transpose() * rb->v + 0.5 * rb->omega.transpose() * rb->L << "(time: "
              << timeRB << ")" << "\n";
}


void RunSimulation(RigidBody *rb, double y[]) {
    timeRB = timeRB + 0.015;
    printInvariant(rb);
    double y0[STATE_SIZE],
            yfinal[STATE_SIZE];
    for (int i = 0; i < STATE_SIZE; ++i) {
        yfinal[i] = 0;
        y0[i] = 0;
    }
    State_to_Array(rb, y0);


/* copy yfinal back to y0 */
    ode(y0, yfinal, STATE_SIZE, timeRB, timeRB + 0.015, dydt, rb);


    Array_to_State(rb, yfinal);
    //ортоганализация матрицы вращения
    Vector3d v1 = rb->R.col(0);
    Vector3d v2 = rb->R.col(1);
    Vector3d v3 = rb->R.col(2);
    v1.normalize();
    v2 = v2 - (v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]) * v1;
    v2.normalize();
    v3 = v3 - (v1[0] * v3[0] + v1[1] * v3[1] + v1[2] * v3[2]) * v1 -
         (v3[0] * v2[0] + v3[1] * v2[1] + v3[2] * v2[2]) * v2;
    v3.normalize();
    rb->R.col(0) = v1;
    rb->R.col(1) = v2;
    rb->R.col(2) = v3;

    State_to_Array(rb, y);
}

void InitRigidBody(RigidBody *rb,int mode) {
    //размер
    double y[STATE_SIZE];
    if (mode == 1) {
        std::cout << "Enter the dimensions of the block (preferably up to two): \n";
        double x0 = 1, y0 = 2, z0 = 1;
        cin >> x0 >> y0 >> z0;
        rb->x0 = x0;
        rb->y0 = y0;
        rb->z0 = z0;
        std::cout << "Enter the mass of the bar:\n";
        cin >> rb->mass;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                rb->R(i, j) = 0;
                if (i == j) {
                    rb->R(i, j) = 1;
                }
            }
        }
        //cubeIbody
        rb->Ibody << y0 * y0 + z0 * z0, 0, 0,
                0, x0 * x0 + z0 * z0, 0,
                0, 0, x0 * x0 + y0 * y0;

        rb->Ibody = rb->Ibody * (rb->mass / 12);
        rb->Ibodyinv = rb->Ibody.reverse();

        rb->x = {0, 0, 0};
        rb->P = {0, 0, 0};
        rb->L = {0.5, 1, 1};
        for (double &j : y) {
            j = 0;
        }
    }else{
        std::cout << "Enter the radius of the sphere:\n";
        double x0 = 1;
        cin >> x0;
        rb->x0 = x0;
        rb->y0 = x0;
        rb->z0 = x0;
        std::cout << "Enter the mass of the sphere:\n";
        cin >> rb->mass;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                rb->R(i, j) = 0;
                if (i == j) {
                    rb->R(i, j) = 1;
                }
            }
        }

        double k=rb->mass/5*2;

        rb->Ibody << x0, 0, 0,
                0, x0*x0, 0,
                0, 0, x0*x0;

        rb->Ibodyinv = rb->Ibody.reverse();

        rb->x= {0, 0, 0};
        rb->P= {0, 0, 0};
        rb->L= {1, 0, 1};
        for (double &j : y) {
            j = 0;
        }
    }
    State_to_Array(rb, y);
    //посчиталось то что нужно
    Array_to_State(rb, y);
}