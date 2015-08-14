#include "kalman.h"
#include "arm_math.h"

#define arm_mat_instance(val) { \
                                  .numRows = num_vars, \
                                  .numCols = num_vars, \
                                  .pData = val \
                              }

#define arm_mat_instance_col(val) { \
                                  .numRows = num_vars, \
                                  .numCols = 1, \
                                  .pData = val \
                              }

typedef arm_matrix_instance_f32 arm_matrix;

kalman::kalman()
{

    /* we need to transpose F and store it in F_T for future use */
    arm_matrix mat_F = arm_mat_instance(F);

    arm_matrix mat_F_T = arm_mat_instance(F_T);

    arm_mat_trans_f32(&mat_F, &mat_F_T);
    /* F_T now holds the transposed F */
}

kalman::~kalman()
{

}

void kalman::predict()
{

    float new_X[num_vars];
    float new_P[num_vars * num_vars];

        /*** initialize matrix references ***/

    arm_matrix mat_new_X = arm_mat_instance_col(new_X);
    arm_matrix mat_current_X = arm_mat_instance_col(X);
    arm_matrix mat_F = arm_mat_instance(F);

    arm_matrix mat_new_P = arm_mat_instance(new_P);
    arm_matrix mat_current_P = arm_mat_instance(P);
    arm_matrix mat_F_T = arm_mat_instance(F_T);
    arm_matrix mat_Q = arm_mat_instance(Q);

                /*** x = F * x ***/

    //new_X = F * current_X
    arm_mat_mult_f32(&mat_F, &mat_current_X, &mat_new_X);

    // X = new_X;
    memcpy(X, new_X, sizeof(float) * num_vars);


            /*** P = F * P * (F^T) + Q ***/

    //new_P = F * current_P
    arm_mat_mult_f32(&mat_F, &mat_current_P, &mat_new_P);

    //current_P = new_P * F^T
    arm_mat_mult_f32(&mat_new_P, &mat_F_T, &mat_current_P);

    //new_P = current_P + Q
    arm_mat_add_f32(&mat_current_P, &mat_Q, &mat_new_P);

    // P = new_P;
    memcpy(P, new_P, sizeof(float) * num_vars * num_vars);


}

void kalman::update(float measured)
{

}