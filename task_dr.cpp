#include "LSM9DS0.h"
#include "tasks.h"
#include "ao_quaternion.h"
#include "kalman.h"

Ticker accTick;
Ticker gyroTick;

struct Triple_F
{
    float x;
    float y;
    float z;
};

ao_quaternion Qstate; //quaternion state
Triple_F Rpos = {.x = 0.0, .y = 0.0, .z = 0.0}; //Real position
Triple_F Rvel = {.x = 0.0, .y = 0.0, .z = 0.0}; //Real velocity


/*
 *  Movement on each axis is completely unrelated to movement 
 *  on any other axis, so each axis can have a separate Kalman
 *  filter, and it should not suffer for it.
 *  
 *  In other words, covariance between axes should be zero.
 */

kalman x, y, z;


vector<Triple> accvals;
vector<Triple> gyrovals;
Triple magvals;

const float ACC_PERIOD    = 0.000625;            //seconds between each accelerometer sample
const float GYRO_PERIOD   = 0.0010416667;        //seconds between each gyro sample
const float MAG_PERIOD    = 0.01;                //seconds between each magnetometer sample

const float ACC_PERIOD_2  = 0.000000390625;      //seconds between each accelerometer sample
const float GYRO_PERIOD_2 = 0.0000010850694;     //seconds between each gyro sample
const float MAG_PERIOD_2  = 0.0001;              //seconds between each magnetometer sample


inline void update_vals()
{
    accvals = lsm9.readAccel();
    gyrovals = lsm9.readGyro();
    magvals = lsm9.readMagneto();
}

inline void update_Qstate(Triple gyroVal)
{
    //convert from degrees to radians? possibly?
    volatile Triple_F gyro = {
        .x = raw2dps(gyroVal.x, 250) * GYRO_PERIOD,
        .y = raw2dps(gyroVal.y, 250) * GYRO_PERIOD,
        .z = raw2dps(gyroVal.z, 250) * GYRO_PERIOD
    };
    //pc.printf("%.5f, %.5f, %.5f\r\n", gyro.x, gyro.y, gyro.z);
    //Qtmp = qyaw * qpitch * qroll
    //qyaw
    ao_quaternion qz = {.r = cosf(gyro.z / 2), .x = 0, .y = 0, .z = sinf(gyro.z / 2)};
    //qpitch
    ao_quaternion qy = {.r = cosf(gyro.y / 2), .x = 0, .y = sinf(gyro.y / 2), .z = 0};
    //qroll
    ao_quaternion qx = {.r = cosf(gyro.x / 2), .x = sinf(gyro.x / 2), .y = 0, .z = 0};
    
    //temporary variables
    ao_quaternion r1, Qtmp;
    
    
    ao_quaternion_multiply(&r1, &qz, &qy);
    ao_quaternion_multiply(&Qtmp, &r1, &qx);
    
    
    //multiply Qstate by Qtmp to get new Qstate
    ao_quaternion_multiply(&Qstate, &Qstate, &Qtmp);
    pc.printf("%.5f, %.5f, %.5f , %.5f, %.5f, %.5f, %.5f\r\n", gyro.x, gyro.y, gyro.z, Qtmp.r, Qtmp.x, Qtmp.y, Qtmp.z);
    
    // static int counter = 0;
    // if (counter++ % 75 == 0)
    // {
    //     counter = 0;
    //     pc.printf("%.5f, %.5f, %.5f, %.5f\r\n", Qstate.r, Qstate.x, Qstate.y, Qstate.z);
    // }
}

inline void update_Rpos(Triple acc)
{
    static int counter = 0;

    //pure quaternion form of acc after conversion to gravities
    ao_quaternion acc_tmp = {.r = 0, .x = raw2gravities(acc.x, 2), .y = raw2gravities(acc.y, 2), .z = raw2gravities(acc.z, 2)};
    
    //rotate acc by Qstate
    ao_quaternion_rotate(&acc_tmp, &acc_tmp, &Qstate);
    
    //acceleration, after rotation into the inertial reference frame
    Triple_F acc_rotated = {.x = acc_tmp.x, .y = acc_tmp.y, .z = acc_tmp.z}; 

    x.predict();
    x.update(acc_rotated.x);
    //Rpos.x = x.X[0];

    y.predict();
    y.update(acc_rotated.y);
    //Rpos.y = y.X[0];

    z.predict();
    z.update(acc_rotated.z);
    //Rpos.z = z.X[0];
    
    // //update current velocity
    Rvel.x = acc_rotated.x * ACC_PERIOD;
    Rvel.y = acc_rotated.y * ACC_PERIOD;
    Rvel.z = acc_rotated.z * ACC_PERIOD;
    
    // //update absolute position
    Rpos.x += Rvel.x * ACC_PERIOD + acc_rotated.x * ACC_PERIOD_2;
    Rpos.y += Rvel.y * ACC_PERIOD + acc_rotated.y * ACC_PERIOD_2;
    Rpos.z += Rvel.z * ACC_PERIOD + acc_rotated.z * ACC_PERIOD_2;
    
    counter += 1;
    if (counter % 75 == 0) {
        counter = 0;
        //pc.printf("%.5f, %.5f, %.5f , %.5f, %.5f, %.5f\r\n", Rpos.x, Rpos.y, Rpos.z, Rvel.x, Rvel.y, Rvel.z);
        //pc.printf("%.5f, %.5f, %.5f, %.5f\r\n", Qstate.r, Qstate.x, Qstate.y, Qstate.z);
        //pc.printf("%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\r\n", raw2gravities(acc.x, 2), raw2gravities(acc.y, 2), raw2gravities(acc.z, 2), acc_tmp.x, acc_tmp.y, acc_tmp.z, acc_tmp.r);
    }
}

void newAcc()
{
    accvals = lsm9.readAccel();
    //! pc.printf("x: %i\r\n", accvals.size());
    //should only be one value, but be thorough
    for (unsigned int i = 0; i < accvals.size(); i++)
        update_Rpos(accvals[i]);
}

void newGyro()
{
    gyrovals = lsm9.readGyro();
    //! pc.printf("g: %i\r\n", gyrovals.size());
    //should only be one value, but be thorough
    for (unsigned int i = 0; i < gyrovals.size(); i++)
        update_Qstate(gyrovals[i]);
}

void task_dr()
{
    update_vals(); //clear whatever buffer is built-up in the LSM9DS0
    
    pc.printf("Starting...\r\n");
    
    accvals = lsm9.readAccel(); //read the first real accel val,
    accTick.attach(&newAcc, ACC_PERIOD); //which, incidentally, might sync this timer up
    
    gyrovals = lsm9.readGyro(); //repeat for gyro vals
    gyroTick.attach(&newGyro, GYRO_PERIOD); //and sync this timer as much as possible
    
    
__disable_irq();
    //consider popping accvals[0] off stack
    ao_quaternion gravity = {.r = 0, .x = raw2gravities(accvals[0].x, 2), .y = raw2gravities(accvals[0].y, 2), .z = raw2gravities(accvals[0].z, 2)};
    ao_quaternion_normalize(&gravity, &gravity);
    
    ao_quaternion v = {.r = 0, .x = 0, .y = 0, .z = 1};
    
    ao_quaternion_vectors_to_rotation(&Qstate, &gravity, &v);

    pc.printf("%.5f,%.5f,%.5f,%.5f\r\n\r\n", Qstate.r, Qstate.x, Qstate.y, Qstate.z);
__enable_irq();
    while (1); //busy loop forever
    
}
