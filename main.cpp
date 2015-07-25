#include "mbed.h"
#include "LSM9DS0.h"
#include "ao_quaternion.h"
#include "tasks.h"
#include "global_state.h"

//these are the actual instantiations of the variables declared in global_state.h
Serial pc(SERIAL_TX, SERIAL_RX); //allows communication over serial
LSM9DS0 lsm9(D9, D7, D12, D11, D13); //sets up the LSM9DS0 driver

int main()
{
    //this baud rate is fast enough to move a significant
    //chunk of data in real time, but not fast enough to
    //move all of the data in real time at this point.
    pc.baud(115200);
    
    //start the dead reckoning task
    task_dr();
    
    return 0;
}
