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

#include "snap_cam_ros/snap_cam.hpp"

SnapCamDriver::SnapCamDriver(ros::NodeHandle nh, ros::NodeHandle pnh,
			     ros::NodeHandle cinfo_nh, std::string camera_name)
  : nh_(nh),
    pnh_(pnh),
    recon_server_(pnh_),
    cinfo_manager_(cinfo_nh, camera_name),
    im_trans_(nh_){
  running_=false;
}

SnapCamStereoDriver::SnapCamStereoDriver(ros::NodeHandle nh, ros::NodeHandle pnh,
					 std::string left_camera_name,
					 std::string right_camera_name)
  : SnapCamDriver(nh,pnh,ros::NodeHandle(nh,"left"), left_camera_name),
    cinfo_manager_right_(ros::NodeHandle(nh,"right"), right_camera_name),
    im_trans_right_(nh){
  running_=false;
}

SnapCamDriver::~SnapCamDriver(){
}

void SnapCamDriver::advertiseTopics(){
  cam_pub_ = im_trans_.advertiseCamera("image_raw",1,false);
  expos_pub_ = nh_.advertise<snap_msgs::ExposureTimes>("exposure_times",1);
}

void SnapCamStereoDriver::advertiseTopics(){
  cam_pub_ = im_trans_.advertiseCamera("left/image_raw",1,false);
  cam_pub_right_ = im_trans_.advertiseCamera("right/image_raw",1,false);
  expos_pub_ = nh_.advertise<snap_msgs::ExposureTimes>("left/exposure_times",1);
  expos_pub_right_ = nh_.advertise<snap_msgs::ExposureTimes>("right/exposure_times",1);
}

void SnapCamDriver::ReconCallback(snap_cam_ros::SnapCamConfig &config, uint32_t level){
  config_ = config;
  if(running_){
    //TODO: Add frame_id, frame_id_right
    //TODO: Add reconfigurable params for width, height, framerate
    snap_cam_man_->SetManualExposure( config_.exposure );
    snap_cam_man_->SetManualGain( config_.gain );
  }
}

void SnapCamDriver::GetMonotonicClockOffset()
{
  // Camera Frames are timestamped with the monotonic clock
  // ROS timestamps use the realtime clock.  Here we compute the difference
  // and apply to messages
  struct timespec time_monotonic;
  struct timespec time_realtime;
  clock_gettime(CLOCK_REALTIME, &time_realtime);
  clock_gettime(CLOCK_MONOTONIC, &time_monotonic);

  ros::Time realtime(time_realtime.tv_sec, time_realtime.tv_nsec);
  ros::Time monotonic(time_monotonic.tv_sec, time_monotonic.tv_nsec);

  monotonic_offset = realtime - monotonic;

  ROS_INFO_STREAM("Monotonic offset: " << monotonic_offset);
}


bool SnapCamDriver::Start(){
  ROS_INFO("SnapCamDriver Starting");

  GetMonotonicClockOffset();
  advertiseTopics();

  recon_server_.setCallback(boost::bind(&SnapCamDriver::ReconCallback, this, _1, _2));

  int format, cam_type, width, height, frame_rate;
  bool is_cam_master;
  pnh_.param<int>("format", format, 1);//default to YUV
  pnh_.param<int>("camera_number", cam_type,1);
  pnh_.param<int>("width", width, 640);
  pnh_.param<int>("height", height, 480);
  pnh_.param<int>("frame_rate", frame_rate, 30);
  pnh_.param<bool>("is_cam_master", is_cam_master, true);

  snap_cam_param_.camera_config.cam_format = (Snapdragon::CameraFormat)format;
  snap_cam_param_.camera_config.cam_type = (Snapdragon::CameraType)cam_type;
  snap_cam_param_.camera_config.pixel_width = width;
  snap_cam_param_.camera_config.pixel_height = height;
  snap_cam_param_.camera_config.memory_stride = width;
  snap_cam_param_.camera_config.fps = frame_rate;
  snap_cam_param_.camera_config.is_cam_master = is_cam_master;

  snap_cam_man_ = new Snapdragon::CameraManager(&snap_cam_param_);

  if( snap_cam_man_->Initialize() != 0 ) {
    ROS_ERROR("CameraManager::Initialize() failed");
    return false;
  }

  if( snap_cam_man_->Start() != 0 ) {
    ROS_ERROR("CameraManager::Start() failed");
    return false;
  }

  std::string yuv_remap;
  pnh_.param("yuv_remap", yuv_remap, std::string("mono"));
  if(yuv_remap == "mono")
  {
    yuv_map_ = MONO;
  }
  else if(yuv_remap == "yuv422")
  {
    yuv_map_ = YUV422;
  }
  else if(yuv_remap == "rgb8")
  {
    yuv_map_ = RGB8;
  }
  else if(snap_cam_param_.camera_config.cam_format == Snapdragon::CameraFormat::YUV_FORMAT)
  {
    ROS_ERROR("Using YUV format, but specified an unknown mapping to ROS type");
    return false;
  }

  //Alocating extra buffer in case we incorrectly choose the image type (YUV vs RAW)
  uint32_t image_size = snap_cam_man_->GetImageSize()*3.0;
  image_buffer = new uint8_t[ image_size ];

  running_=true;
  snap_cam_man_->SetManualExposure( config_.exposure );
  snap_cam_man_->SetManualGain( config_.gain );

  return true;
}

bool SnapCamStereoDriver::StartStereo(){
  if(!Start())
    return false;

  uint32_t image_size = snap_cam_man_->GetImageSize()*3.0;
  image_buffer_right = new uint8_t[ image_size ];
  return true;
}

void SnapCamDriver::Stop(){
  running_=false;
  delete[] image_buffer;
}

void SnapCamStereoDriver::Stop(){
  running_=false;
  delete[] image_buffer;
  delete[] image_buffer_right;
}

void SnapCamDriver::PrintNewestFrameId(){
  int64_t frame_id;
  frame_id = snap_cam_man_->GetNewestFrameId();
  ROS_INFO_STREAM("Frame id: " << frame_id);
}

void SnapCamDriver::PublishLatestFrame(){
  int64_t newest_id = snap_cam_man_->GetNewestFrameId();
  if (newest_id < 0) {
    ros::Duration(0.005).sleep();
    return;
  }

  uint32_t image_size = snap_cam_man_->GetImageSize();
  int32_t rc;
  static int64_t prev_frame_id = -1;
  static uint64_t prev_ts_ns = 0;
  int64_t frame_id = prev_frame_id + 1 + config_.skip_n_frames;
  uint64_t timestamp_ns;
  uint32_t used = 0;

  if (newest_id > frame_id) {
    frame_id = newest_id;
  }
  //ROS_INFO_STREAM("Frame id: " << frame_id);
  //ROS_INFO_STREAM("Newest id: " << newest_id);
  rc = snap_cam_man_->GetNextImageData( frame_id, &timestamp_ns, image_size, &used, image_buffer );

  if( rc != 0 ) {
    ROS_ERROR_STREAM("GetImageData() Unable to get Frame data for id: " << frame_id << " rc: " << rc);
    return;
  }

  if( used != image_size ) {
    ROS_WARN_STREAM("GetImageData() : Buffer size returned do not match: size: "<<image_size<<" got: "<<used);
    return;
  }

  prev_frame_id = frame_id;
  prev_ts_ns = timestamp_ns;

  sensor_msgs::Image::Ptr image(new sensor_msgs::Image());
  image->width = snap_cam_param_.camera_config.pixel_width;
  image->height = snap_cam_param_.camera_config.pixel_height;

  if(snap_cam_param_.camera_config.cam_format == Snapdragon::CameraFormat::YUV_FORMAT){
    unsigned int image_size_pix = snap_cam_param_.camera_config.pixel_height *
      snap_cam_param_.camera_config.pixel_width;
    unsigned int u_offset = image_size_pix;
    unsigned int w = snap_cam_param_.camera_config.pixel_width;
    unsigned int h = snap_cam_param_.camera_config.pixel_height;

    if(yuv_map_ == MONO)
    {
      sensor_msgs::fillImage(*image,sensor_msgs::image_encodings::MONO8,
			     snap_cam_param_.camera_config.pixel_height,
			     snap_cam_param_.camera_config.pixel_width,
			     snap_cam_param_.camera_config.memory_stride,
			     image_buffer);
      image->encoding = std::string("mono8");
      image->step = snap_cam_param_.camera_config.pixel_width;
    }
    else if(yuv_map_ == YUV422)
    {
      image->data.resize(image_size_pix * 2);
      for(unsigned int i=0; i*2 < image_size_pix; i++){
	unsigned int row = (i*2)/w; // full res row
	unsigned int col = (i*2)%w; // full res row
	unsigned int rc_ind = ((row/2)*(w/2)) + (col/2); // half res ind
	unsigned int u_ind = u_offset + (rc_ind*2) + 1;
	unsigned int v_ind = u_offset + (rc_ind*2);

	//UYVY
	image->data[(i*4)+1] = image_buffer[i*2]; // Y1
	image->data[(i*4)+3] = image_buffer[(i*2) + 1]; // Y2
	image->data[(i*4)]   = image_buffer[u_ind];
	image->data[(i*4)+2] = image_buffer[v_ind];
      }

      image->encoding = std::string("yuv422");
      image->step = snap_cam_param_.camera_config.pixel_width*2;
    }
    else if(yuv_map_ == RGB8)
    {
      image->data.resize(image_size_pix * 3);
      for(unsigned int i=0; i < image_size_pix; ++i){
	unsigned int row = (i)/w; // full res row
	unsigned int col = (i)%w; // full res row
	unsigned int rc_ind = ((row/2)*(w/2)) + (col/2); // half res ind
	unsigned int u_ind = u_offset + (rc_ind*2) + 1;
	unsigned int v_ind = u_offset + (rc_ind*2);

	image->data[(i*3)  ] = std::min(std::max((int)(image_buffer[i] +
						       (1.370705*(image_buffer[v_ind]-128))),0),255);
	image->data[(i*3)+1] = std::min(std::max((int)(image_buffer[i] +
						       (-0.337633*(image_buffer[u_ind]-128)) +
						       (-0.698001*(image_buffer[v_ind]-128))),0),255);
	image->data[(i*3)+2] = std::min(std::max((int)(image_buffer[i] +
						       (1.732446*(image_buffer[u_ind]-128))),0),255);
      }
      image->encoding = std::string("rgb8");
      image->step = snap_cam_param_.camera_config.pixel_width*3;
    }
  }
  else if(snap_cam_param_.camera_config.cam_format == Snapdragon::CameraFormat::RAW_FORMAT){
    unsigned int image_size_bytes = snap_cam_param_.camera_config.pixel_height *
      snap_cam_param_.camera_config.memory_stride;
    image->data.resize(image_size_bytes);
    for(unsigned int i=0; i*4 < image_size_bytes; ++i){
      memcpy(&(image->data[i*4]),
	     reinterpret_cast<uint8_t*>(image_buffer)+i*5, 4);
    }
    image->encoding = std::string("mono8");
    image->step = snap_cam_param_.camera_config.pixel_width;
  }
  else{
    ROS_ERROR("Unknown image format");
    return;
  }

  ros::Time timestamp(timestamp_ns / 1000000000, timestamp_ns % 1000000000);
  timestamp += monotonic_offset;

  sensor_msgs::CameraInfo::Ptr cinfo(
    new sensor_msgs::CameraInfo(cinfo_manager_.getCameraInfo()));

  image->header.frame_id = config_.frame_id;
  image->header.stamp = timestamp;
  image->is_bigendian = 0;

  cinfo->header.frame_id = config_.frame_id;
  cinfo->header.stamp = timestamp;

  cinfo->width = snap_cam_param_.camera_config.pixel_width;
  cinfo->height = snap_cam_param_.camera_config.pixel_height;

  cam_pub_.publish(image, cinfo);

  // publish exposure times also
  uint64_t exposure_time_ns = 0;
  int32_t erc = snap_cam_man_->GetExposureTime(frame_id, &exposure_time_ns);
  if ( erc != 0 ) {
    ROS_ERROR_STREAM("Could not find exposure time for frame_id " << frame_id);
  }
  ros::Duration exposure_time(exposure_time_ns / 1000000000, exposure_time_ns % 1000000000);

  uint64_t timestamp_coe_ns = 0;
  int32_t trc = snap_cam_man_->GetExposureCenterTimestamp(frame_id, &timestamp_coe_ns);
  if ( trc != 0 ) {
    ROS_ERROR_STREAM("Could not find center of exposure timestamp for frame_id " << frame_id);
  }
  ros::Time timestamp_coe(timestamp_coe_ns / 1000000000, timestamp_coe_ns % 1000000000);
  timestamp_coe += monotonic_offset;

  snap_msgs::ExposureTimes expos_times;
  expos_times.header.stamp = timestamp;
  expos_times.header.frame_id = config_.frame_id;
  expos_times.exposure_time = exposure_time;
  expos_times.center_of_exposure = timestamp_coe;
  expos_pub_.publish(expos_times);

  /*
  // TODO: If any parameters have changed or the driver is
  // restarted, notify dynamic reconfigure
  if (config_changed_) {
    recon_server_.updateConfig(config_);
    recon_changed_ = false;
  */
}


void SnapCamStereoDriver::PublishLatestStereoFrame(){
  int64_t newest_id = snap_cam_man_->GetNewestFrameId();
  if (newest_id < 0) {
    ros::Duration(0.005).sleep();
    return;
  }

  uint32_t image_size = snap_cam_man_->GetImageSize();
  int32_t rc;
  static int64_t prev_frame_id = -1;
  static uint64_t prev_ts_ns = 0;
  int64_t frame_id = prev_frame_id + 1 + config_.skip_n_frames;
  uint64_t timestamp_ns;
  uint32_t used = 0;

  if (newest_id > frame_id) {
    frame_id = newest_id;
  }
  //ROS_INFO_STREAM("Frame id: " << frame_id);
  rc = snap_cam_man_->GetNextStereoImageData( frame_id, &timestamp_ns, image_size, &used, image_buffer, image_buffer_right );

  if( rc != 0 ) {
    ROS_ERROR_STREAM("GetImageData() Unable to get Frame data for id: " << frame_id << " rc: " << rc);
    return;
  }

  if( used != image_size ) {
    ROS_WARN_STREAM("GetImageData() : Buffer size returned do not match: size: "<<image_size<<" got: "<<used);
    return;
  }

  prev_frame_id = frame_id;
  prev_ts_ns = timestamp_ns;

  sensor_msgs::Image::Ptr image(new sensor_msgs::Image());
  sensor_msgs::Image::Ptr image_right(new sensor_msgs::Image());

  sensor_msgs::fillImage(*image,sensor_msgs::image_encodings::MONO8,
			 snap_cam_param_.camera_config.pixel_height,
			 snap_cam_param_.camera_config.pixel_width,
			 snap_cam_param_.camera_config.memory_stride,
			 image_buffer);

  sensor_msgs::fillImage(*image_right,sensor_msgs::image_encodings::MONO8,
			 snap_cam_param_.camera_config.pixel_height,
			 snap_cam_param_.camera_config.pixel_width,
			 snap_cam_param_.camera_config.memory_stride,
			 image_buffer_right);

  ros::Time timestamp(timestamp_ns / 1000000000, timestamp_ns % 1000000000);
  timestamp += monotonic_offset;

  sensor_msgs::CameraInfo::Ptr cinfo(
    new sensor_msgs::CameraInfo(cinfo_manager_.getCameraInfo()));

  image->header.frame_id = config_.frame_id;
  image->header.stamp = timestamp;

  image->is_bigendian = 0;
  image->encoding = std::string("mono8");

  image->width = snap_cam_param_.camera_config.pixel_width;
  image->height = snap_cam_param_.camera_config.pixel_height;
  image->step = snap_cam_param_.camera_config.pixel_width;

  // scale the stereo calibration (left) if using different resolution
  if (cinfo->width != snap_cam_param_.camera_config.pixel_width || cinfo->height != snap_cam_param_.camera_config.pixel_height) {
    float width_scale = (float)snap_cam_param_.camera_config.pixel_width / (float)cinfo->width;
    float height_scale = (float)snap_cam_param_.camera_config.pixel_height / (float)cinfo->height;
    if (width_scale == height_scale) {
      for (int i=0; i<7; i++) {
        cinfo->K[i] = cinfo->K[i]*width_scale;
      }
      for (int i=0; i<8; i++) {
        cinfo->P[i] = cinfo->P[i]*width_scale;
      }
    } else {
      ROS_ERROR_STREAM("PublishLatestStereoFrame() : Calibration may not match requested resolution.");
    }
  }

  cinfo->header.frame_id = config_.frame_id;
  cinfo->header.stamp = timestamp;

  cinfo->width = snap_cam_param_.camera_config.pixel_width;
  cinfo->height = snap_cam_param_.camera_config.pixel_height;

  cam_pub_.publish(image, cinfo);


  sensor_msgs::CameraInfo::Ptr cinfo_right(
    new sensor_msgs::CameraInfo(cinfo_manager_right_.getCameraInfo()));

  image_right->header.frame_id = config_.frame_id;
  image_right->header.stamp = timestamp;

  image_right->is_bigendian = 0;
  image_right->encoding = std::string("mono8");

  image_right->width = snap_cam_param_.camera_config.pixel_width;
  image_right->height = snap_cam_param_.camera_config.pixel_height;
  image_right->step = snap_cam_param_.camera_config.pixel_width;

  // scale the stereo calibration (right) if using different resolution
  if (cinfo_right->width != snap_cam_param_.camera_config.pixel_width || cinfo_right->height != snap_cam_param_.camera_config.pixel_height) {
    float width_scale = (float)snap_cam_param_.camera_config.pixel_width / (float)cinfo_right->width;
    float height_scale = (float)snap_cam_param_.camera_config.pixel_height / (float)cinfo_right->height;
    if (width_scale == height_scale) {
      for (int i=0; i<7; i++) {
        cinfo_right->K[i] = cinfo_right->K[i]*width_scale;
      }
      for (int i=0; i<8; i++) {
        cinfo_right->P[i] = cinfo_right->P[i]*width_scale;
      }
    } else {
      ROS_ERROR_STREAM("PublishLatestStereoFrame() : Calibration may not match requested resolution.");
    }
  }

  cinfo_right->header.frame_id = config_.frame_id_right;
  cinfo_right->header.stamp = timestamp;

  cinfo_right->width = snap_cam_param_.camera_config.pixel_width;
  cinfo_right->height = snap_cam_param_.camera_config.pixel_height;

  cam_pub_right_.publish(image_right, cinfo_right);

  // publish exposure times also
  uint64_t exposure_time_ns = 0;
  int32_t erc = snap_cam_man_->GetExposureTime(frame_id, &exposure_time_ns);
  if ( erc != 0 ) {
    ROS_ERROR_STREAM("Could not find exposure time for frame_id " << frame_id);
  }
  ros::Duration exposure_time(exposure_time_ns / 1000000000, exposure_time_ns % 1000000000);

  uint64_t timestamp_coe_ns = 0;
  int32_t trc = snap_cam_man_->GetExposureCenterTimestamp(frame_id, &timestamp_coe_ns);
  if ( trc != 0 ) {
    ROS_ERROR_STREAM("Could not find center of exposure timestamp for frame_id " << frame_id);
  }
  ros::Time timestamp_coe(timestamp_coe_ns / 1000000000, timestamp_coe_ns % 1000000000);
  timestamp_coe += monotonic_offset;

  snap_msgs::ExposureTimes expos_times;
  expos_times.header.stamp = timestamp;
  expos_times.header.frame_id = config_.frame_id;
  expos_times.exposure_time = exposure_time;
  expos_times.center_of_exposure = timestamp_coe;

  expos_pub_.publish(expos_times);

  expos_times.header.frame_id = config_.frame_id_right;
  expos_pub_right_.publish(expos_times);

  /*
  // TODO: If any parameters have changed or the driver is
  // restarted, notify dynamic reconfigure
  if (config_changed) {
    recon_server_.updateConfig(config_);
    recon_changed_ = false;
  */
}

