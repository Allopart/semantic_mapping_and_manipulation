/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the PAL Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Luca Marchionni
 * Author: Bence Magyar
 * Author: Enrique Fernández
 * Author: Paul Mathieu
 */

#include <omni_drive_controller/odometry.h>

#include <boost/bind.hpp>

namespace omni_drive_controller
{
  namespace bacc = boost::accumulators;

  Odometry::Odometry(size_t velocity_rolling_window_size)
  : timestamp_(0.0)
  , x_(0.0)
  , y_(0.0)
  , heading_(0.0)
  , linear_x_(0.0)
  , linear_y_(0.0)
  , angular_(0.0)
  , wheel_separation_(0.0)
  , wheel_radius_(0.0)
  , wheel_0_old_pos_(0.0)
  , wheel_1_old_pos_(0.0)
  , wheel_2_old_pos_(0.0)
  , velocity_rolling_window_size_(velocity_rolling_window_size)
  , linear_x_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , linear_y_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , angular_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  {
  }

  void Odometry::init(const ros::Time& time)
  {
    // Reset accumulators and timestamp:
    resetAccumulators();
    timestamp_ = time;
  }

  bool Odometry::update(double pos_0, double pos_1, double pos_2, const ros::Time &time)
  {
    /// Get current wheel joint positions:
    const double wheel_0_cur_pos  = pos_0  * wheel_radius_;
    const double wheel_1_cur_pos  = pos_1  * wheel_radius_;
    const double wheel_2_cur_pos  = pos_2  * wheel_radius_;

    /// Estimate velocity of wheels using old and current position:
    const double wheel_0_est_vel  = wheel_0_cur_pos  - wheel_0_old_pos_;
    const double wheel_1_est_vel  = wheel_1_cur_pos  - wheel_1_old_pos_;
    const double wheel_2_est_vel  = wheel_2_cur_pos  - wheel_2_old_pos_;

    /// Update old position with current:
    wheel_0_old_pos_  = wheel_0_cur_pos;
    wheel_1_old_pos_  = wheel_1_cur_pos;
    wheel_2_old_pos_  = wheel_2_cur_pos;



    /// Compute linear and angular omni:
    Eigen::Matrix3f j1f = Eigen::Matrix3f::Zero();
    j1f << std::sin(M_PI/3.0), -std::cos(M_PI/3.0), -wheel_separation_,
        std::sin(-M_PI/3.0), -std::cos(-M_PI/3.0), -wheel_separation_,
        std::sin(M_PI), -std::cos(M_PI), -wheel_separation_;

    Eigen::Matrix3f j1f_inverse = j1f.inverse();
    Eigen::Matrix3f j2 = Eigen::Matrix3f::Identity();

    Eigen::Vector3f wheel_est_vel;
    wheel_est_vel << wheel_0_est_vel, wheel_1_est_vel, wheel_2_est_vel;

    Eigen::Vector3f est_vel = j1f_inverse * j2 * wheel_est_vel;
    const double linear_x = est_vel(0);
    const double linear_y = est_vel(1);
    const double angular = est_vel(2);

    Eigen::Matrix3f R_theta_inv;
    R_theta_inv << std::cos(heading_), -std::sin(heading_), 0,
                std::sin(heading_), std::cos(heading_), 0,
                0, 0, 1;


    Eigen::Vector3f est_pos = R_theta_inv * j1f_inverse * j2 * wheel_est_vel;
    x_ += est_pos(0);
    y_ += est_pos(1);
    heading_ += est_pos(2);

    /// We cannot estimate the speed with very small time intervals:
    const double dt = (time - timestamp_).toSec();
    if (dt < 0.0001)
      return false; // Interval too small to integrate with

    timestamp_ = time;

    /// Estimate speeds using a rolling mean to filter them out:
    linear_x_acc_(linear_x/dt);
    linear_y_acc_(linear_y/dt);
    angular_acc_(angular/dt);

    linear_x_ = bacc::rolling_mean(linear_x_acc_);
    linear_y_ = bacc::rolling_mean(linear_y_acc_);
    angular_ = bacc::rolling_mean(angular_acc_);

    return true;
  }

  void Odometry::setWheelParams(double wheel_separation, double wheel_radius)
  {
    wheel_separation_ = wheel_separation;
    wheel_radius_     = wheel_radius;
  }

  void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
  {
    velocity_rolling_window_size_ = velocity_rolling_window_size;

    resetAccumulators();
  }


  void Odometry::resetAccumulators()
  {
    linear_x_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    linear_y_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    angular_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
  }

} // namespace omni_drive_controller
