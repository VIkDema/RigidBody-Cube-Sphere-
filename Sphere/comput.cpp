#include "comput.h"

void InitRigidBody(RigidBody *rb) {
   
    double x0 = 1, y0 = 1, z0 = 1;
    rb->x0 = x0;
    rb->y0 = y0;
    rb->z0 = z0;
    rb->mass = 4;
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
    double y[STATE_SIZE];
    for (double &j : y) {
        j = 0;
    }
    StateToArray(rb, y);
    ArrayToState(rb, y);
}

void StateToArray(RigidBody *rb, double *y) {
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

void ArrayToState(RigidBody *rb, double *y) {
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
//вычисляет крутящий момент и силу
void ComputeForceandTorque(double t, RigidBody *rb){

    rb->torque={0, 0, 0};
    rb->force={0, 0, 0};
}

//приводим вектор к матричному виду
Matrix3d Star(Vector3d a) {
    Matrix3d matrix_star;
    matrix_star<< 0, -a[2], a[1],
                a[2], 0, -a[0],
                -a[1], a[0], 0;
    return matrix_star;
}

void dydt(double t, double y[], double ydot[], RigidBody *rb) {

    ComputeForceandTorque(t, rb);

    //заносим производную в момент времени т в массив
    DdtStateToArray(rb, ydot);
}

void DdtStateToArray(RigidBody *rb, double *ydot) {
/* copy ddt x(t) = v(t) into ydot */
    *ydot++ = rb->v[0];
    *ydot++ = rb->v[1];
    *ydot++ = rb->v[2];
/* Compute R˙(t) = ω(t)∗R(t) */
    Matrix3d Rdot = Star(rb->omega) * rb->R;
/* copy R˙(t) into array */
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            *ydot++ = Rdot(i, j);

    *ydot++ = rb->force[0]; /* ddt P(t) = F(t) */
    *ydot++ = rb->force[1];
    *ydot++ = rb->force[2];

    *ydot++ = rb->torque[0]; /* ddt L(t) = τ(t) */
    *ydot++ = rb->torque[1];
    *ydot++ = rb->torque[2];
}

void mul_y(double *y, double numb){
    for(int i = 0; i<STATE_SIZE; i++){
        y[i]*=numb;
    }
}

void sum_y(double *y, double numb){
    for(int i = 0; i<STATE_SIZE; i++){
        y[i]+=numb;
    }
}

void sum_y_y(double *y1, double *y2){
    for(int i = 0; i<STATE_SIZE; i++){
        y1[i]+=y2[i];
    }
}


void ode(double y0[], double yFinal[], int len, double t0, double t1, dydt_func dydt, RigidBody *rb){

    double h=t1-t0;

    
    double k_1[len];
    
    dydt(t0, y0, k_1, rb);    
   
    mul_y(k_1, h);
    double temp[len];
    for(int i = 0; i<len; i++){
        temp[i] = k_1[i];
    }
    
   
    mul_y(temp, 1.0/2.0);
    sum_y_y(temp, y0);
    double k_2[len];
  
    dydt(t0,temp,k_2, rb);
    mul_y(k_2, h);
    
 
    for(int i = 0; i<len; i++){
        temp[i] = k_2[i];
    }
    mul_y(temp, 1.0/2.0);
    sum_y_y(temp, y0);
    double k_3[len];
 
    dydt(t0, temp, k_3, rb);
    mul_y(k_3, h);
    
  
    for(int i = 0; i<len; i++){
        temp[i] = k_3[i];
    }
    sum_y_y(temp, y0);
    double k_4[len];
   
    dydt(t0, temp, k_4, rb);
    mul_y(k_4, h);
    
   
    mul_y(k_1, 1.0/6.0);
    mul_y(k_2, 1.0/3.0);
    mul_y(k_3, 1.0/3.0);
    mul_y(k_4, 1.0/6.0);
    
   
    for(int i = 0; i<len; i++){
        yFinal[i] = y0[i];
    }
   
    sum_y_y(yFinal, k_1);
    sum_y_y(yFinal, k_2);
    sum_y_y(yFinal, k_3);
    sum_y_y(yFinal, k_4);
}
    
    


void printInvariant(RigidBody* rb){
   /*std::cout<<0.5*rb->mass*rb->v.transpose()*rb->v<<"\n";
   std::cout<<.5*rb->omega.transpose()*rb->L<<"\n";*/
   std::cout<<"time: "<<timeRB<<", energy: "<<(double)(0.5*rb->mass*rb->v.transpose()*rb->v)+(double)(0.5*rb->omega.transpose()*rb->L)<<"\n";
}

void RunSimulation(RigidBody *rb, double y[]) {
    timeRB = timeRB + 0.025;
     printInvariant(rb);
    double y0[STATE_SIZE], yFinal[STATE_SIZE];
    for (int i = 0; i < STATE_SIZE; ++i) {
        yFinal[i] = 0;
        y0[i] = 0;
    }
    StateToArray(rb, y0);

    ode(y0, yFinal, STATE_SIZE, timeRB-0.025, timeRB, dydt, rb);

    ArrayToState(rb, yFinal);

    //ортоганализируем матрицу вращения, чтобы она не искажалась
    //извлекаем векторы из матрицы
    Vector3d vector_x;
    Vector3d vector_y;
    Vector3d vector_z;

    for (int i=0;i<3;i++){
        vector_x[i]=rb->R(i,0);
        vector_y[i]=rb->R(i,1);
        vector_z[i]=rb->R(i,2);
    }

    vector_x.normalize();
   
    vector_y=vector_y-(vector_y[0]*vector_x[0]+vector_y[1]*vector_x[1]+ vector_y[2]*vector_x[2])*vector_x;
    vector_y.normalize();
    
    vector_z=vector_z-(vector_z[0]*vector_x[0]+ vector_z[1]*vector_x[1] + vector_z[2]*vector_x[2])*vector_x-
    (vector_y[0]*vector_z[0] + vector_y[1]*vector_z[1] + vector_y[2]*vector_z[2])*vector_y;
    vector_z.normalize();
    

    for (int i=0;i<3;i++){
        rb->R(i,0)=vector_x[i];
        rb->R(i,1)=vector_y[i];
        rb->R(i,2)=vector_z[i];
    }




    StateToArray(rb, y);
}
