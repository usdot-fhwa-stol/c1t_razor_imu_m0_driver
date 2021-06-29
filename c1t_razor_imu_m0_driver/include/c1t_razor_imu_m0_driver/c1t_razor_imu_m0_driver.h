#ifndef C1T_RAZOR_IMU_M0_DRIVER_H
#define C1T_RAZOR_IMU_M0_DRIVER_H

#include <sensor_msgs/Imu.h>

#include <cav_driver_utils/driver_wrapper/driver_wrapper.h>

/**
 * Razor IMU M0 ROS package wrapper that confirms to the CARMA IMU
 * hardware interface.
 * 
 * Subscribers:
 *   system_alert: CARMA system_alert topic
 *   imu/data_raw: IMU data published from razor_imu_m0_driver
 * 
 * Publishers:
 *   driver_discovery: driver information
 */ 
class RazorImuM0DriverWrapper : public cav::DriverWrapper
{
public:
  /**
   * @brief RazorImuM0DriverWrapper constructor
   * 
   * @param argc commandline argument count
   * @param argv array of commandline argument values
   * @param name ROS node name (defaults to "c1t_razor_imu_m0_driver")
   */
  RazorImuM0DriverWrapper(int argc, char** argv, const std::string& name = "c1t_razor_imu_m0_driver");

  /**
   * @brief RazorImuM0DriverWrapper destructor
   */
  virtual ~RazorImuM0DriverWrapper() = default;

private:
  ros::Subscriber imu_sub_;
  ros::Time last_update_time_;
  double imu_timeout_;

  /**
   * @brief Initializes the ROS node
   * 
   * This function is called before the ROS node starts running.
   */
  void initialize() override;

  /**
   * @brief Process stuff before spinning
   * 
   * This function is called before the spinOnce() function.
   */
  void pre_spin() override;

  /**
   * @brief Process stuff after spinning
   * 
   * This function is called after the spinOnce() function.
   */
  void post_spin() override;

  /**
   * @brief Prepare ROS node for shutting down
   * 
   * This function is called before the node is shut down.
   */
  void shutdown() override;

  /**
   * @brief Check if the IMU has timed out
   * 
   * Check if the time between IMU messages exceeds the timeout threshoold.
   */
  void checkImuTimeout();
};

#endif  // C1T_RAZOR_IMU_M0_DRIVER_H
