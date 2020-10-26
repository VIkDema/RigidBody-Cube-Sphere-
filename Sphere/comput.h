#pragma once
//используем данную библиотеку для удобной работы с матрицами и тройками
#include </usr/include/eigen3/Eigen/Dense>
using namespace Eigen;
#include <iostream> 

using namespace std;

#define STATE_SIZE 18

static double  timeRB=0;

struct RigidBody {
/* Constant quantities */
    double mass; /* mass M */

    Matrix3d Ibody; /* Ibody */
    Matrix3d Ibodyinv; /* I−1 body (inverse of Ibody) */
/* State variables */
    Vector3d x;   /* x(t) */
    Matrix3d R;   /* R(t) */
    Vector3d P;  /* P(t) */
    Vector3d L;  /* L(t) */
/* Derived quantities (auxiliary variables) */
    Matrix3d Iinv;    /* I−1(t) */
    Vector3d v;   /* v(t) */
    Vector3d omega;      /* ω(t) */
/* Computed quantities */
    Vector3d force;    /* F(t) */
    Vector3d torque;         /* τ(t) */


    double x0,y0,z0;
};

static RigidBody *my_RigidBody = new RigidBody();


typedef void (*dydt_func)(double , double [], double ydot[], RigidBody *);

Matrix3d Star(Vector3d a);


void InitRigidBody(RigidBody* );

void StateToArray(RigidBody *, double *);
void ArrayToState(RigidBody *, double *);

Matrix3d Star(Vector3d );

void DdtStateToArray(RigidBody *, double *);
void dydt(double , double [], double [], RigidBody *);

void ode(double [], double [], int , double , double , dydt_func ,RigidBody* );
void ComputeForceandTorque(double , RigidBody *);

void RunSimulation(RigidBody *,double []);
void printInvariant(RigidBody* rb);
