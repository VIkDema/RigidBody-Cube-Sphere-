//
// Created by viktor on 25.09.2020.
//

#ifndef MATHLABS_MATHRB_H
#define MATHLABS_MATHRB_H

#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;


#define STATE_SIZE 18

static double  timeRB=0;


struct RigidBody {
/* Constant quantities */
    double mass; /* mass M */
    Matrix3d Ibody, /* Ibody */
    Ibodyinv; /* I−1
body (inverse of Ibody) */
/* State variables */
    Vector3d x;   /* x(t) */
    Matrix3d R;   /* R(t) */
    Vector3d P,   /* P(t) */
    L;  /* L(t) */
/* Derived quantities (auxiliary variables) */
    Matrix3d Iinv;    /* I−1(t) */
    Vector3d v,   /* v(t) */
    omega;      /* ω(t) */
/* Computed quantities */
    Vector3d force,    /* F(t) */
    torque;         /* τ(t) */
    double x0,y0,z0;
};

static struct RigidBody *pRigidBody = new struct RigidBody();

typedef void (*dydt_func)(double t, double y[], double ydot[], RigidBody *rb);

void Array_to_State(RigidBody *rb, double *y);

void State_to_Array(RigidBody *rb, double *y);

void Compute_Force_and_Torque(double t, RigidBody *rb);

Matrix3d Star(Vector3d a);

void ddt_State_to_Array(RigidBody *rb, double *ydot);

void dydt(double t, double y[], double ydot[], RigidBody *rb);

void ode(double y0[], double yend[], int len, double t0,
         double t1, dydt_func dydt,RigidBody* rb);

void RunSimulation(RigidBody *rb,double y[]);
void InitRigidBody(RigidBody* rb);


#endif //MATHLABS_MATHRB_H
