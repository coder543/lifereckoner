#ifndef KALMAN_H
#define KALMAN_H

const float dt   = 0.000625;
const float dt_2 = 0.000000390625;
const int num_vars = 3;

class kalman
{
public:
    /*** current state ***/
    float X[num_vars] = {
                    0,  //position
                    0,  //velocity
                    0   //acceleration
                 };

private:
    /*** values for prediciton step ***/
    float P[num_vars * num_vars] = { //covariance matrix
                        0,   0,   0,
                        0, 100,   0,
                        0,   0, 500
                    };

    float F[num_vars * num_vars] = { //state transition function
                        1,  dt, 0.5 * dt_2,
                        0,   1,     dt    ,
                        0,   0,     1     
                    }; 

    float F_T[num_vars * num_vars]; //state transition function, transposed

    float Q[num_vars * num_vars] = { //system noise
                        0,   0,   0,
                        0,   0,   0,
                        0,   0,   0
                    };


    /*** values for update step ***/
    float H[num_vars] = {0, 0, 16384}; //state space to measurement space conversion
    float R = 0.01; //sensor measurement noise
    float S[num_vars * num_vars]; //system uncertainty
    float K[num_vars * num_vars]; //kalman gain


public:
    kalman();
    ~kalman();

    void predict();
    void update(float measured);

};

#endif