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
#ifndef SNAP_CAM_HPP_
#define SNAP_CAM_HPP_

#include "SnapdragonCameraManager.hpp"
#include "SnapdragonCameraTypes.hpp"
#include "SnapdragonCameraUtil.hpp"

#include <iostream>
#include <chrono>
#include <thread>
#include <time.h>

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h> 
#include <snap_msgs/ExposureTimes.h>
#include <std_msgs/Header.h>

#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <camera_info_manager/camera_info_manager.h>

#include <dynamic_reconfigure/server.h>
#include <snap_cam_ros/SnapCamConfig.h>

class SnapCamDriver {
public:
  /**
   * Constructor.
   * @param nh
   *   nodehandle to initialize the node.
   * @param pnh
   *   private namespace nodehandle for this node
   * @param cinfo_nh
   *   private namespace specifically for the camera info
   * @param camera_name
   *   ROS name of the camera
   */
  SnapCamDriver(ros::NodeHandle nh, ros::NodeHandle pnh, 
		ros::NodeHandle cinfo_nh, 
		std::string camera_name);
  /**
   * Destructor.
   */
  ~SnapCamDriver();

  /**
   * Start the Camera Manager and Driver
   */
  bool Start();

  /**
   * Stop the Camera Manager and Driver
   */
  virtual void Stop();
  
  /**
   * Block for the latest camera frame and publish (image and info)
   */
  void PublishLatestFrame();

  /**
   * Helper/Debug Function to print the camera frame number (id) of 
   * the newest frame in the queue
   */
  void PrintNewestFrameId();

  /**
   * Advertise ros topics (redefined by stereo driver)
   */
  virtual void advertiseTopics();

  Snapdragon::CameraParameters snap_cam_param_;
  Snapdragon::CameraManager * snap_cam_man_;

protected:
  /**
   * Accept a reconfigure request from a client
   * @param config
   *   dynamic reconfigure paramter class of params
   * @param level
   *   level/priority of reconfigure request
   */
  void ReconCallback(snap_cam_ros::SnapCamConfig &config, uint32_t level);

  /**
   * Compute difference between monotonic and realtime clock
   */
  void GetMonotonicClockOffset();

  //TODO: recon params that require driver restart
  //void OpenCamera(snap_cam::SnapCamConfig &new_config);
  //void CloseCamera();
  //boost::recursive_mutex mutex_;
  //bool config_changed_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  
  dynamic_reconfigure::Server<snap_cam_ros::SnapCamConfig> recon_server_;
  snap_cam_ros::SnapCamConfig config_;
  
  bool running_;
  
  image_transport::ImageTransport im_trans_;
  image_transport::CameraPublisher cam_pub_;
  camera_info_manager::CameraInfoManager cinfo_manager_;
  ros::Publisher expos_pub_;

  uint8_t* image_buffer;
  ros::Duration monotonic_offset;

  int yuv_map_;
  enum YUV_MAPPING{
    MONO,
    YUV422,
    RGB8
  };
};

class SnapCamStereoDriver : public SnapCamDriver {
public:
  /**
   * Constructor.
   * @param nh
   *   nodehandle to intialize the node.
   * @param pnh
   *   private namespace nodehandle for this node
   * @param camera_name
   *   ROS name of the left camera
   * @param camera_name_right
   *   ROS name of the right camera
   */  
  SnapCamStereoDriver(ros::NodeHandle nh, ros::NodeHandle pnh, 
		      std::string camera_name, std::string camera_name_right);

  /**
   * Advertise ROS topics, redefined from base class
   */  
  void advertiseTopics();

  /**
   * Block and publish latest stereo pair from driver
   */  
  void PublishLatestStereoFrame();

  /**
   * Start the Stereo driver and manager
   */  
  bool StartStereo();

  /**
   * Stop the Stereo driver and manager
   */  
  void Stop();

protected:
  image_transport::ImageTransport im_trans_right_;
  image_transport::CameraPublisher cam_pub_right_;
  camera_info_manager::CameraInfoManager cinfo_manager_right_;
  ros::Publisher expos_pub_right_;
  uint8_t* image_buffer_right;
};

#endif // SNAP_CAM_HPP_
