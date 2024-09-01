#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <hal.h>
#include <UM7.h>

int main(int argc, char *argv[]){
    HAL hal(argc, argv);
    usleep(100000); 

    UM7 imu("/dev/ttyUSB0");
    usleep(100000);

    imu.set_sensor_baud_rate(115200);
    usleep(100000); 
    
    imu.set_euler_rate(255);
    usleep(100000); 

    imu.set_all_processed_rate(255);
    usleep(100000); 

    imu.zero_gyros();
    usleep(100000);

    imu.calibrate_accelerometers();
    usleep(100000);

    return 0;
}

