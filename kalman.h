#ifndef KALMAN_H
#define KALMAN_H

const float dt   = 0.000625;
const float dt_2 = 0.000000390625;

class kalman
{
public:
    /*** current state ***/
    float X[3] = {
                    0,  //position
                    0,  //velocity
                    0   //acceleration
                 };

private:
    /*** values for prediciton step ***/
    float P[3][3] = { //covariance matrix
                        {0,   0,   0},
                        {0, 100,   0},
                        {0,   0, 500}
                    };

    float F[3][3] = { //state transition function
                        {1,  dt, 0.5 * dt_2},
                        {0,   1,     dt    },
                        {0,   0,     1     }
                    }; 
    float Q[3][3] = { //system noise
                        {0,   0,   0},
                        {0,   0,   0},
                        {0,   0,   0}
                    };


    /*** values for update step ***/
    float H[3] = {0, 0, 1}; //state space to measurement space conversion
    float R = 0.01; //sensor measurement noise
    float S[3][3]; //system uncertainty
    float K[3][3]; //kalman gain


public:
    kalman();
    ~kalman();

    void predict();
    void update(float measured);

};

#endif