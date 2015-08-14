#include "kalman.h"
#include "mbed.h"

kalman::kalman()
{

}

kalman::~kalman()
{

}

void kalman::predict()
{
	//x = Fx
	//P = FP(F^T) + Q
	arm_matrix_instance_f32 test;
}

void kalman::update(float measured)
{

}