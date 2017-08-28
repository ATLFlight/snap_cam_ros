/****************************************************************************
 *   Copyright (c) 2017 Michael Shomin. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ATLFlight nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * In addition Supplemental Terms apply.  See the SUPPLEMENTAL file. 
 ****************************************************************************/

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <boost/thread.hpp>

#include "snap_cam_ros/snap_cam.hpp"

namespace snap_cam {
  class CameraNodelet : public nodelet::Nodelet {
  public:
    CameraNodelet() : running_(false) {}
    ~CameraNodelet();

  private:
    virtual void onInit();
    virtual void devicePoll();
    volatile bool running_;
    boost::mutex mutex;
    boost::shared_ptr<SnapCamDriver> driver_;
    boost::shared_ptr<boost::thread> deviceThread_;
  };

  CameraNodelet::~CameraNodelet() {
    if (running_) {
      mutex.lock();
      driver_->Stop();
      mutex.unlock();
    }
  }

  void CameraNodelet::onInit() {
    ros::NodeHandle nh(getNodeHandle());
    ros::NodeHandle pnh(getPrivateNodeHandle());

    std::string camera_name;
    pnh.param<std::string>("camera_name", camera_name, "camera");

    driver_.reset(new SnapCamDriver(nh, pnh, nh, camera_name));

    if (driver_->Start()) {
      running_ = true;
      deviceThread_ = boost::shared_ptr< boost::thread >
	(new boost::thread(boost::bind(&CameraNodelet::devicePoll, this)));
    } else {
      NODELET_ERROR("Unable to open camera.");
      driver_.reset();
    }
  }

  void CameraNodelet::devicePoll() {
    while(running_){
      mutex.lock();
      driver_->PublishLatestFrame();
      mutex.unlock();
    }
  }


};

// Register this plugin with pluginlib.
//
// parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(snap_cam, driver,snap_cam::CameraNodelet, nodelet::Nodelet);
