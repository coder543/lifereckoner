#include "LSM9DS0.h"
#include "tasks.h"
#include "ao_quaternion.h"

Ticker accTick;
Ticker gyroTick;

int counter = 0;

struct Triple_F
{
    float x;
    float y;
    float z;
};

ao_quaternion Qstate; //quaternion state
Triple_F Rpos = {.x = 0.0, .y = 0.0, .z = 0.0}; //Real position
Triple_F Rvel = {.x = 0.0, .y = 0.0, .z = 0.0}; //Real velocity

vector<Triple> accvals;
vector<Triple> gyrovals;
Triple magvals;

#define ACC_PERIOD    0.000625            //seconds between each accelerometer sample
#define GYRO_PERIOD   0.0010416667        //seconds between each gyro sample
#define MAG_PERIOD    0.01                //seconds between each magnetometer sample

#define ACC_PERIOD_2  0.000000390625      //seconds between each accelerometer sample
#define GYRO_PERIOD_2 0.0000010850694     //seconds between each gyro sample
#define MAG_PERIOD_2  0.0001              //seconds between each magnetometer sample


inline void update_vals()
{
    accvals = lsm9.readAccel();
    gyrovals = lsm9.readGyro();
    magvals = lsm9.readMagneto();
}

inline void update_Qstate(Triple gyro)
{
    
    //Qtmp = qyaw * qpitch * qroll
    
    //qyaw
    ao_quaternion qz = {.r = cosf(gyro.z/2), .x = 0, .y = 0, .z = sinf(gyro.z/2)};
    //qpitch
    ao_quaternion qy = {.r = cosf(gyro.y/2), .x = 0, .y = sinf(gyro.y/2), .z = 0};
    //qroll
    ao_quaternion qx = {.r = cosf(gyro.x/2), .x = sinf(gyro.x/2), .y = 0, .z = 0};
    
    //temporary variables
    ao_quaternion r1, Qtmp;
    
    
    ao_quaternion_multiply(&r1, &qz, &qy);
    ao_quaternion_multiply(&Qtmp, &r1, &qx);
    
    
    //multiply Qstate by Qtmp to get new Qstate
    ao_quaternion_multiply(&Qstate, &Qstate, &Qtmp);
    
}

inline void update_Rpos(Triple acc)
{
    //pure quaternion form of acc after conversion to gravities
    ao_quaternion acc_tmp = {.r = 0, .x = raw2gravities(acc.x, 2), .y = raw2gravities(acc.y, 2), .z = raw2gravities(acc.z, 2)};
    
    //rotate acc by Qstate
    ao_quaternion_rotate(&acc_tmp, &acc_tmp, &Qstate);
    
    //acceleration, after rotation into the inertial reference frame
    Triple_F acc_rotated = {.x = acc_tmp.x, .y = acc_tmp.y, .z = acc_tmp.z}; 
    
    //update current velocity
    Rvel.x += acc_rotated.x * ACC_PERIOD;
    Rvel.y += acc_rotated.y * ACC_PERIOD;
    Rvel.z += acc_rotated.z * ACC_PERIOD;
    
    //update absolute position
    Rpos.x += Rvel.x * ACC_PERIOD + acc_rotated.x * ACC_PERIOD_2;
    Rpos.y += Rvel.y * ACC_PERIOD + acc_rotated.y * ACC_PERIOD_2;
    Rpos.z += Rvel.z * ACC_PERIOD + acc_rotated.z * ACC_PERIOD_2;
    
    counter += 1;
    if (counter % 75 == 0) {
        counter = 0;
        pc.printf("%.2f, %.2f, %.2f\r\n", Rpos.x, Rpos.y, Rpos.z);
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
    accTick.attach(&newAcc, ACC_PERIOD); //which incidentally syncs this timer up
    
    gyrovals = lsm9.readGyro(); //repeat for gyro vals
    gyroTick.attach(&newGyro, GYRO_PERIOD); //and sync this timer as much as possible
    
    
    //consider popping accvals[0] off stack
    ao_quaternion gravity = {.r = 0, .x = raw2gravities(accvals[0].x, 2), .y = raw2gravities(accvals[0].y, 2), .z = raw2gravities(accvals[0].z, 2)};
    ao_quaternion_normalize(&gravity, &gravity);
    
    ao_quaternion v = {.r = 0, .x = 0, .y = 0, .z = 1};
    
    ao_quaternion_vectors_to_rotation(&Qstate, &gravity, &v);
   
    while (1); //busy loop forever
    
}