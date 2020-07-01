====
**New Official Version of Driver available**
====
The new official driver (an extended version of this package with some bug-fixes and new functionality) is available at https://github.com/basler/pylon-ros-camera. 


====
**ROS-Driver for Basler Cameras**
====
**developed by Magazino GmbH, using the pylon Software Camera Suite by Basler AG**


This package offers many functions of the Basler pylon API inside the ROS-Framwork.

The package supports Baslers USB 3.0, GigE as well as the DART cameras.

Images can continuously be published over *\/image\_raw* or the *\/image\_rect* topic.
The latter just in case the intrinsic calibration matrices are provided through the **camera_info_url** parameter.

The camera-characteristic parameter such as hight, width, projection matrices and camera_frame were published over the *\/camera\_info* topic.
Furthermore an action-based image grabbing with desired exposure, gain, gamma and / or brightness is provided.
Hence one can grab a sequence of images with above target settings as well as a single image.

Adapting camera's settings regarding binning (in x and y direction), exposure, gain, gamma and brightness can be done using provided 'set_*' services.
These changes effect the continuous image acquisition and hence the images provided through the image topics.

The default node operates in Software-Trigger Mode.
This means that the image acquisition is triggered with a certain rate and the camera is not running in the continuous mode.

The package opens either a predefined camera (using a given 'device_user_id' parameter) or, if no camera id is predefined the first camera device it can find.

|

******
**Installation**
******
The package has been tested for ROS-Indigo and ROS-Kinetic.

The pylon_camera-pkg requires the pylonSDK to be installed on your system. Please download and install the pylon debian package for your architecture from:

``https://www.baslerweb.com/de/support/downloads/downloads-software/``

In order to build the package, you need to configure rosdep (i.e. the ROS command-line tool for checking and installing system dependencies for ROS packages) such that
it knows how to resolve this dependency. This can be achieved by executing the following commands:

``sudo sh -c 'echo "yaml https://raw.githubusercontent.com/magazino/pylon_camera/indigo-devel/rosdep/pylon_sdk.yaml " > /etc/ros/rosdep/sources.list.d/15-plyon_camera.list'``

``rosdep update``

Then, clone the pylon_camera-pkg, and the camera_control_msgs-pkg and install the pylon SDK in your catkin_ws:

``cd ~/catkin_ws/src/ && git clone https://github.com/magazino/pylon_camera.git && git clone https://github.com/magazino/camera_control_msgs.git``

``rosdep install --from-paths . --ignore-src --rosdistro=$ROS_DISTRO -y``

Build the pylon_camera package as you would build a standard ROS-package unsing p.e.

``cd ~/catkin_ws && catkin_make``

|

******
**Parameters**
******

All parameters are listed in the default config file:  ``config/default.yaml``

**Common parameters**

- **camera_frame**
  The tf frame under which the images were published

- **device_user_id**
  The DeviceUserID of the camera. If empty, the first camera found in the device list will be used

- **camera_info_url**
  The CameraInfo URL (Uniform Resource Locator) where the optional intrinsic camera calibration parameters are stored. This URL string will be parsed from the CameraInfoManager:
  http://docs.ros.org/api/camera_info_manager/html/classcamera__info__manager_1_1CameraInfoManager.html#details

- **image_encoding**
  The encoding of the pixels -- channel meaning, ordering, size taken from the list of strings in include/sensor_msgs/image_encodings.h. The supported encodings are 'mono8', 'bgr8', 'rgb8', 'bayer_bggr8', 'bayer_gbrg8' and 'bayer_rggb8'.
  Default values are 'mono8' and 'rgb8'

- **binning_x & binning_y**
  Binning factor to get downsampled images. It refers here to any camera setting which combines rectangular neighborhoods of pixels into larger "super-pixels." It reduces the resolution of the output image to (width / binning_x) x (height / binning_y). The default values binning_x = binning_y = 0 are considered the same as binning_x = binning_y = 1 (no subsampling).

- **downsampling_factor_exposure_search**
  To speed up the exposure search, the mean brightness is not calculated on the entire image, but on a subset instead. The image is downsampled until a desired window hight is reached. The window hight is calculated out of the image height divided by the downsampling_factor_exposure search

- **frame_rate**
  The desired publisher frame rate if listening to the topics. This parameter can only be set once at start-up. Calling the GrabImages-Action can result in a higher frame rate.

- **shutter_mode**
  Set mode of camera's shutter if the value is not empty. The supported modes are 'rolling', 'global' and 'global_reset'.
  Default value is '' (empty)

**Image Intensity Settings**

The following settings do **NOT** have to be set. Each camera has default values which provide an automatic image adjustment resulting in valid images

- **exposure**
  The exposure time in microseconds to be set after opening the camera.

- **gain**
  The target gain in percent of the maximal value the camera supports. For USB-Cameras, the gain is in dB, for GigE-Cameras it is given in so called 'device specific units'.

- **gamma**
  Gamma correction of pixel intensity. Adjusts the brightness of the pixel values output by the camera's sensor to account for a non-linearity in the human perception of brightness or of the display system (such as CRT).

- **brightness**
  The average intensity value of the images. It depends the exposure time as well as the gain setting. If '**exposure**' is provided, the interface will try to reach the desired brightness by only varying the gain. (What may often fail, because the range of possible exposure values is many times higher than the gain range). If '**gain**' is provided, the interface will try to reach the desired brightness by only varying the exposure time. If '**gain**' AND '**exposure**' are given, it is not possible to reach the brightness, because both are assumed to be fix.

- **brightness_continuous**
  Only relevant, if '**brightness**' is set: The brightness_continuous flag controls the auto brightness function. If it is set to false, the brightness will only be reached once. Hence changing light conditions lead to changing brightness values. If it is set to true, the given brightness will be reached continuously, trying to adapt to changing light conditions. This is only possible for values in the possible auto range of the pylon API which is e.g. [50 - 205] for acA2500-14um and acA1920-40gm

- **exposure_auto & gain_auto**
  Only relevant, if '**brightness**' is set: If the camera should try to reach and / or keep the brightness, hence adapting to changing light conditions, at least one of the following flags must be set. If both are set, the interface will use the profile that tries to keep the gain at minimum to reduce white noise. The exposure_auto flag indicates, that the desired brightness will be reached by adapting the exposure time. The gain_auto flag indicates, that the desired brightness will be reached by adapting the gain.

**Optional and device specific parameter**

- **gige/mtu_size**
  The MTU size. Only used for GigE cameras. To prevent lost frames configure the camera has to be configured with the MTU size the network card supports. A value greater 3000 should be good (1500 for RaspberryPI)

- **gige/inter_pkg_delay**
  The inter-package delay in ticks. Only used for GigE cameras. To prevent lost frames it should be greater 0. For most of GigE-Cameras, a value of 1000 is reasonable. For GigE-Cameras used on a RaspberryPI this value should be set to 11772.


******
**Usage**
******

The pylon_camera_node can be started over the launch file which includes a config file with desired parameters as frame rate or exposure time

``roslaunch pylon_camera pylon_camera_node.launch``     or     ``rosrun pylon_camera pylon_camera_node``

Images were only published if another node connects to the image topic. The published images can be seen using the image_view node from the image_pipeline stack:

``rosrun image_view image_view image:=/pylon_camera_node/image_raw``

******
**Questions**
******

Please provide your questions via http://answers.ros.org/questions/ and tag them with **pylon_camera**
