/**
 * Copyright 2021 U.S. Department of Transportation, Federal Highway Administration
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "c1t_razor_imu_m0_driver/c1t_razor_imu_m0_driver.h"

#include <sensor_msgs/Imu.h>

RazorImuM0DriverWrapper::RazorImuM0DriverWrapper(int argc, char** argv, const std::string& name)
  : DriverWrapper(argc, argv, name), imu_timeout_(0.5)
{
}

void RazorImuM0DriverWrapper::initialize()
{
  status_.imu = true;

  imu_sub_ = nh_->subscribe<sensor_msgs::Imu>("imu/data_raw", 1, [this](const sensor_msgs::Imu::ConstPtr& msg){
    last_update_time_ = ros::Time::now();
    status_.status = cav_msgs::DriverStatus::OPERATIONAL;
  });

  private_nh_->param<double>("imu_timeout", imu_timeout_, 0.5);
}

void RazorImuM0DriverWrapper::pre_spin()
{
  this->checkImuTimeout();
}

void RazorImuM0DriverWrapper::post_spin()
{
}

void RazorImuM0DriverWrapper::shutdown()
{
}

void RazorImuM0DriverWrapper::checkImuTimeout()
{
  if (last_update_time_.isZero() || ros::Time::now() - last_update_time_ > ros::Duration(imu_timeout_))
  {
    status_.status = cav_msgs::DriverStatus::OFF;
  }
}
