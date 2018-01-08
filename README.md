# Snapdragon Flight<sup>TM</sup> ROS Camera Node/Nodelets

This ROS-based example demonstrates how to interact with libcamera on the Snapdragon Flight<sup>TM</sup> board.

## Table of Contents

1. [High-level block diagram](#high-level-block-diagram)
1. [Pre-requisites](#pre-requisites)
   * [Hardware](#hardware)
   * [Software](#software)
1. [Clone and build example code](#clone-and-build-example-code)
1. [Run example code](#run-example-code)
1. [Verification](#verification)

## Pre-requisites

### Hardware
This example requires the following hardware:

* [Qualcomm Snapdragon Flight Kit](https://shop.intrinsyc.com/collections/product-development-kits/products/qualcomm-snapdragon-flight-sbc)

### Software
To run the example, the following are needed:

* [Plaform Image from Intrynsic 3.1.3.1](https://support.intrinsyc.com/attachments/download/1597/Flight_3.1.3.1_JFlash.zip)
* [ROS on target](https://github.com/ATLFlight/ATLFlightDocs/blob/master/SnapdragonROSInstallation.md)

## Clone and build example code

### Setup ROS workspace on target

```
adb shell
source /home/linaro/.bashrc
mkdir -p /home/linaro/ros_ws/src
cd /home/linaro/ros_ws/src
catkin_init_workspace
cd ../
catkin_make
echo "source /home/linaro/ros_ws/devel/setup.bash" >> /home/linaro/.bashrc
```

This ensures that the ROS workspace is setup correctly.

### Clone the sample code
* The repo may be cloned from [here](https://github.com/ATLFlight/snap_cam_ros.git) directly on the target, or cloned on the host computer and then pushed to the target using ADB. The recommended method is to clone directly on the target.  You will also need the snap_msgs repository for image metadata messages.

```
adb shell
source /home/linaro/.bashrc
roscd
cd ../src
git clone https://github.com/ATLFlight/snap_cam_ros.git
git clone https://github.com/ATLFlight/snap_msgs.git
```

You will also need the snap_cam_manager submodule:

```
cd snap_cam_ros
git submodule init
git submodule update
```

### Install ROS dependencies

This package depends on other ROS packages to run.  To install them, we'll use [rosdep](http://wiki.ros.org/rosdep)

```bash
cd /home/linaro/ros_ws
rosdep install --from-paths src --skip-keys snap_msgs
```
This requires an internet connection as it will install the rosdeps using aptitude.

If you get an apt error while trying to install the dependencies like this:

```bash
trying to overwrite '/usr/share/glib-2.0/schemas/gschema.dtd', which is also in package libglib-2.0-0 1:2.38.2-r0. this is while trying to install the dependency libglib2.0-dev_2.40.0-2_armhf.deb
```
then you will need to force dpkg to overwrite platform files with files from aptitude.  To do that, run this command:

```bash
sudo apt-get -o Dpkg::Options::="--force-overwrite" –f install
```
Now re-run the rosdep install:

```bash
rosdep install --from-paths src --skip-keys snap_msgs
```

### Build the code

```bash
cd /home/linaro/ros_ws
catkin_make
```

This will compile snap_cam nodes and nodelets for publishing camera images.

### Optional - Install image-transport-plugins

If you would like to stream image data over wifi, it is very beneficial to have ros-indigo-image-transport-plugins.  This will allow you to use compressed image transport to save bandwith for streaming image data.  Unfortunately, the Intrinsyc platform 3.1.3.1 has some some library versions that make apt-get unable to resolve a depency tree.  To solve this, run the included script to get particular versions of some dependencies and install image transport plugins.

```bash
roscd snap_cam_ros
chmod +x fix_transport_plugin_deps.sh
./fix_transport_plugin_deps.sh
```

## Run example code

This example assumes that the user is familiar with ROS framework.  If you are new to ROS, refer to [ROS Start Guide](http://wiki.ros.org/ROS/StartGuide) first to get started.
This also assumes that the ROS build command is successful.

snap_cam_ros nodes and nodelets need to be run as root.  To run one of the example launch files in the launch directory:

```bash
sudo su
source /home/linaro/ros_ws/devel/setup.bash #/path/to/ros_ws/devel/setup.bash
export ROS_IP=192.168.1.1 #your_ip
roslaunch snap_cam_ros hires.launch
```
Take a look at the [launch file](launch/hires.launch) to see what params are available.

## Verification

Images can be viewed on your workstation using rqt_image_view

On your workstation, assuming you have installed ROS, set up you environment variable to point to your target.  These values assume you are in softap mode and your target has IP=192.168.1.1
```bash
export ROS_MASTER_URI=http://192.168.1.1:11311
export ROS_IP=192.168.1.77 #your workstation ip
```

Replace the value of ROS_IP with your real IP address of your workstation.  After that, you can check that data is coming in.
First, check out what topics are being published:

```bash
rostopic list
```

Use `rostopic echo` to verify that data is streaming. You should see camera_info

```bash
rostopic echo /hires/camera_info
```

This should show a stream of live data. If they do not, your ROS_IP
env variable may need to be set on target.  If that is working, try to view the images:

```bash
rosrun rqt_image_vew rqt_image_view
```

Select `/hires/image_raw` from the dropdown box, and you should see streaming video.  You can also
stream compressed images if you have image-transport-plugins installed.  To do so, select the `/hires/image_raw/compressed` topic.

## Fisheye Calibration

For normal monocular calibration, refer the [ROS camera_calibration documentation](http://wiki.ros.org/camera_calibration#Camera_Calibrator).

For calibration of the fisheye camera, however, the plumb_bob model used by ROS is not sufficeint.  Furthermore, SNAV uses a fisheye model.  Because the ROS calibration tool does not support fisheye, a patch is included in this repo to add support.  Please note that this was tested for one particular version of the calibration tool on Ubuntu 14.04.  Specifically, this was tested on camera_calibration 1.12.21-0trusty-20171109-013453-0800.  To check you version, run:

```bash
dpkg -l | grep camera_calibration
```
on your workstation. (Make sure you have run "sudo apt-get install ros-indigo-camera-calibration"). Note that this patch is likely to work for other version if you allow for fuzzy line matching.  Also note that this patch makes the calibration tool only suitable for fisheye calibration, so you must change it back to its original state to calibrate non-fisheye cameras.  To patch:

Make a backup:
```bash
sudo cp /opt/ros/indigo/lib/python2.7/dist-packages/camera_calibration/calibrator.py /opt/ros/indigo/lib/python2.7/dist-packages/camera_calibration/calibrator.py_bu
```
Patch the calibrator:
```bash
sudo patch /opt/ros/indigo/lib/python2.7/dist-packages/camera_calibration/calibrator.py calibrator.patch 
```

Now, after starting snap_cam_ros on the Snapdragon Flight board (roslaunch snap_cam_ros downward.launch) as root:

```bash
rosrun camera_calibration cameracalibrator.py -c downward -p 'acircles' --size 4x11 --square 0.0816 image:=/downward/image_raw
```

You should see the message:

```bash
CUSTOM CALIBRATOR, for use with fisheye cameras!
```
displayed in the terminal.  Make sure to modify the command above for you calibration target type and size.

When you're done, replace the original file:
```bash
sudo mv /opt/ros/indigo/lib/python2.7/dist-packages/camera_calibration/calibrator.py_bu /opt/ros/indigo/lib/python2.7/dist-packages/camera_calibration/calibrator.py
```
