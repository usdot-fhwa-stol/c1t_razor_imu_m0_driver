#include "c1t_razor_imu_m0_driver/c1t_razor_imu_m0_driver.h"

int main(int argc, char** argv) {
    RazorImuM0DriverWrapper wrapper(argc, argv);

    return wrapper.run();
}
