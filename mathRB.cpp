//
// Created by viktor on 25.09.2020.
//

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
    Vector3d r={0,0,0};

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
    /*
    double ydydt[len];
    for (int i = 0; i < len; ++i) {
        ydydt[i] = 0;
    }
    dydt(t0, y0, ydydt, rb);

    for (int i = 0; i < len; ++i) {
        yend[i] = y0[i] + (t1 - t0) * ydydt[i];
    }
*/
    double ydydt1[len];
    for (int i = 0; i < len; ++i) {
        ydydt1[i] = 0;
    }
    dydt(t0,y0,ydydt1,rb);
    double ydydt2[len];
    for (int i = 0; i < len; ++i) {
        ydydt2[i] = 0;
    }
    double y1[len];
    for (int i = 0; i < len; ++i) {
        y1[i] = y0[i]+(t1 - t0)/2*ydydt1[i];
    }

    dydt(t0+(t1 - t0)/2,y1,ydydt2,rb);
    for (int i = 0; i < len; ++i) {
        yend[i] = y0[i]+(t1 - t0)*ydydt2[i];
    }

}

void RunSimulation(RigidBody *rb, double y[]) {
    timeRB = timeRB + 0.03;
    double y0[STATE_SIZE],
            yfinal[STATE_SIZE];
    for (int i = 0; i < STATE_SIZE; ++i) {
        yfinal[i] = 0;
        y0[i] = 0;
    }
    State_to_Array(rb, y0);


/* copy yfinal back to y0 */
    ode(y0, yfinal, STATE_SIZE, timeRB, timeRB + 0.03, dydt, rb);


    Array_to_State(rb, yfinal);
    //ортоганализация матрицы вращения
    Vector3d x_v = rb->R.col(0);
    Vector3d y_v = rb->R.col(1);
    Vector3d z_v = rb->R.col(2);
    x_v.normalize();
    y_v.normalize();
    z_v.normalize();

    y_v = y_v - (x_v[0] * y_v[0] + x_v[1] * y_v[1] + x_v[2] * y_v[2])  * x_v;
    z_v=z_v-(x_v[0] * z_v[0] + x_v[1] * z_v[1] + x_v[2] * z_v[2]) * x_v-(z_v[0] * y_v[0] + z_v[1] * y_v[1] + z_v[2] * y_v[2]) * y_v;
    rb->R.col(0) = x_v;
    rb->R.col(1) = y_v;
    rb->R.col(2) = z_v;

    State_to_Array(rb, y);
}

void InitRigidBody(RigidBody *rb) {
    //размер
    double x0 = 1, y0 = 1, z0 = 1;
    rb->x0 = x0;
    rb->y0 = y0;
    rb->z0 = z0;
    rb->mass = 10;
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
    rb->P = {0, 1, 0};
    rb->L = {0, 0, 10};
    double y[STATE_SIZE];
    for (double &j : y) {
        j = 0;
    }
    State_to_Array(rb, y);
    //посчиталось то что нужно
    Array_to_State(rb, y);
}