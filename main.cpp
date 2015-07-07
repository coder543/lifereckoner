#include "mbed.h"
#include "LSM9DS0.h"
#include "ao_quaternion.h"
#include "tasks.h"

Serial pc(SERIAL_TX, SERIAL_RX);
LSM9DS0 lsm9(D9, D7, D12, D11, D13);

int main()
{
    pc.baud(115200);
    task_dr();
    return 0;
}
