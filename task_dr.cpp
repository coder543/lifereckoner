#include "LSM9DS0.h"
#include "tasks.h"
#include "ao_quaternion.h"

ao_quaternion Qstate; //quaternion state
vector<Triple> accvals = lsm9.readAccel();
vector<Triple> gyrovals = lsm9.readGyro();
Triple magvals = lsm9.readMagneto();

// void print_vals(vector<Triple> &accvals, vector<Triple> &gyrovals, Triple &magvals)
// {
//     
//     //print the collected data
//     for(std::vector<Triple>::iterator it = accvals.begin(); it != accvals.end(); ++it) {
//         pc.printf("ax: %f Gs\r\n", raw2gravities((*it).x, 2));
//         pc.printf("ay: %f Gs\r\n", raw2gravities((*it).y, 2));
//         pc.printf("az: %f Gs\r\n", raw2gravities((*it).z, 2));
//         pc.printf("\r\n");
//     }
//     
//     pc.printf("mx: %f Gauss\r\n", raw2gauss(magvals.x, 2));
//     pc.printf("my: %f Gauss\r\n", raw2gauss(magvals.y, 2));
//     pc.printf("mz: %f Gauss\r\n", raw2gauss(magvals.z, 2));
//     pc.printf("\r\n");
//     
//     for(std::vector<Triple>::iterator it = gyrovals.begin(); it != gyrovals.end(); ++it) {
//         pc.printf("gx: %f dps\r\n", raw2dps((*it).x, 250));
//         pc.printf("gy: %f dps\r\n", raw2dps((*it).y, 250));
//         pc.printf("gz: %f dps\r\n", raw2dps((*it).z, 250));
//         pc.printf("\r\n");
//     }
// }

inline void update_vals()
{
    accvals = lsm9.readAccel();
    gyrovals = lsm9.readGyro();
    magvals = lsm9.readMagneto();
}

inline void update_Qstate()
{
    
}

void task_dr()
{
    update_vals();
    
    //consider popping accvals[0] off stack
    ao_quaternion gravity = {.r = 0, .x = raw2gravities(accvals[0].x, 2), .y = raw2gravities(accvals[0].y, 2), .z = raw2gravities(accvals[0].z, 2)};
    ao_quaternion_normalize(&gravity, &gravity);
    
    ao_quaternion v = {.r = 0, .x = 0, .y = 0, .z = 1};
    
    ao_quaternion_vectors_to_rotation(&Qstate, &gravity, &v);
    
    while (1) {
        update_vals();
        update_Qstate();        
    }
    
}