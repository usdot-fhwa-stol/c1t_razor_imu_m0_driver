#!/bin/bash

# CARMA dependencies
vcs import --input src/carma-msgs.repos src/
vcs import --input src/carma-utils.repos src/

# Razor IMU M0 dependencies
vcs import --input src/razor_imu_m0_driver.repos src/
