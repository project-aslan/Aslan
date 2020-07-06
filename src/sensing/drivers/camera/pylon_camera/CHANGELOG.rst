^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pylon_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.15.0 (2019-03-14)
-------------------
* Add support or area of interest selection through service call

0.14.1 (2019-03-13)
-------------------
* If no device id is given, loop over all available devices until finding a valid one

0.14.0 (2019-02-07)
-------------------
* added diagnostics

0.13.0 (2018-12-20)
-------------------
* Added script to collect images with different flashes
* filling auto_flash_line_2\_ (3) parameter from parameter server
* Support for online change of autoflash
  Allow activating and deactivating lines as active on exposure
  A new service is provided per output line, which allows it to be
  configured as autoflash. Internally GiGe Cameras will
  default to switch all outputs on if autoflash is set to True
  whenever exposure is active.
* Removed maintainer

0.12.0 (2018-06-19)
-------------------
* Parameter for auto flash control
* Publish camera info when requested, but deactivate images grabbed when not
* Hard coded line_3 as output for gigE cameras
* possible fix for setting device_user_id if it was not set by user
* Propagating the device user id back to ros params server, if not set

0.11.8 (2018-05-16)
-------------------
* Last change from Marcel as a member of the Magazino-Team. Bye Bye :-)
* Bugfix: CameraPublisherImpl leads to crash
* Added CameraInfo to the GrabImagesResult

0.11.7 (2018-05-16)
-------------------
* Revied PR, Moved CameraPublisherImpl struct to method
* Workaround for overestimated wrong numbers of subscribers on image_raw

0.11.6 (2018-05-15)
-------------------
* Added definition in default config and readme description of missed parameter 'shutter_mode'
* fix typos.
* Fixed typo in message in method PylonCameraNode::startGrabbing
* Tried to fix LICENSE-issue

0.11.5 (2018-05-07)
-------------------
* Updated license year
* Issue found -> wrong year -> renamed back

0.11.4 (2018-05-07)
-------------------
* Again renamed the LICENSE file

0.11.3 (2018-05-07)
-------------------
* Renamed LICENSE.rst

0.11.2 (2018-03-28)
-------------------
* Bugfix for useless rectification
  Moved rectification part in the scope of publishing the result.
  This resolves #36.
  Thanks to @flajolet for your contribution

0.11.1 (2018-03-26)
-------------------
* Improved re-connection behavior
  ...by resetting the set_user_output services
  Furthermore the spin method now checks if the camera might have lost connection
  and toggles a re-init.

0.11.0 (2018-03-06)
-------------------
* Removed deprectated msg-flags
  There were deprecated flags in camera_control_msgs/GrabImagesAction,
  that have been removed, namely
  uint8 BRIGHTNESS = 1
  uint8 EXPOSURE = 2
  uint8 target_type
  float32[] target_values

0.10.14 (2018-03-05)
--------------------
* Updated outdated/confusing install instructions
  This resolves #21

0.10.13 (2018-03-05)
--------------------
* Add aarch64 architecture (`#32 <https://github.com/magazino/pylon_camera/pull/32>`_)
* Contributors: lalten

0.10.12 (2018-02-13)
--------------------
* 0.10.11
* Updated install intructions in README.rst, catkin_lint

0.10.10 (2018-02-13)
--------------------
* Installation via pylon-debian pkg
  This fixes #22, fixes #31
* Added missing camera_info_manager dependency

0.10.9 (2018-01-29)
-------------------
* Reviewd bugfix: init() is called within action -> multiple server
  Resolves: SW-6342
* Create action server and data for rectification only once, delete allocated data conditionally

0.10.8 (2018-01-04)
-------------------
* prevent double free

0.10.7 (2017-11-20)
-------------------
* Bugfix: lost this because of PylonCameraNode::grabImagesRaw()

0.10.6 (2017-10-13)
-------------------
* fix Pylon find script (`#27 <https://github.com/magazino/pylon_camera/issues/27>`_)
* Contributors: tlindbloom

0.10.5 (2017-09-28)
-------------------
* Empty action goal now leads to undefined returned image
* Updated README
* Corrected install instructions
  Basler finally provides a debian package for the pylon-sdk

0.10.4 (2017-09-11)
-------------------
* Regeneration of brightness indices after binning change
  Resolves: https://github.com/magazino/pylon_camera/issues/26

0.10.3 (2017-08-31)
-------------------
* Removed all non-ascii symbols from the changelog
  -> fixed changelog 2.0

0.10.2 (2017-08-31 12:09)
-------------------------
* Fixed CHANGELOG.rst
* Contributors: Marcel Debout

0.10.1 (2017-08-31 11:48)
-------------------------
* Bugfix: gain of 0.0 was rejected due to wrong empty check
* CMAKE_INSTALL_RPATH_USE_LINK_PATH for pylon deb
* Contributors: Marcel Debout, Markus Grimm

0.10.0 (2017-07-17)
-------------------
* Updated message dependencies
* Contributors: Magazino Version Daemon

0.9.2 (2017-06-26)
------------------
* SW-1177 No longer using node_name as frame_name but keeping frame_name from configuration file
* Contributors: Nikolas Engelhard

0.9.1 (2017-04-18 17:41)
------------------------
* removed cv bridge version
* Contributors: Ulrich Klank

0.9.0 (2017-04-18 09:25)
------------------------
* Updated message dependencies
* Contributors: Magazino Version Daemon

0.8.1 (2017-04-18 09:22)
------------------------
* removed cv bridge version
* Contributors: Ulrich Klank

0.8.0 (2017-04-12 21:03)
------------------------
* Updated message dependencies
* Contributors: Magazino Version Daemon

0.7.7 (2017-04-12 18:46)
------------------------
* Force recompile (blank line CMakeLists.txt)
* Contributors: Marcel Debout

0.7.6 (2017-04-12 11:02)
------------------------
* Tested the auto-exp-upprper-lim and adapted fail output
* Added upper-exp-limit for exp/brightness search
  For dark scenes, the exposure search (to reach a desired brightness) will
  reach high exposure values ( >1s ). This leads to instabillity and
  timeouts. Therefore it's now possible to limit the exposure and fail in
  case the brightness can not be reached.
  The own binary-exp-search takes above limits as bounds.
* Contributors: Marcel Debout

0.7.5 (2017-04-06 16:14)
------------------------
* Improved smart exp search for values < 50
  The own binary search needs an upper bound. This is generated by using
  the default PylonAutoExposure function with a value of 50.
  So an initial setting of the corresponding exposure to a brightness of
  50 will speed up the search.
* Contributors: Marcel Debout

0.7.4 (2017-04-06 09:32)
------------------------
* Updated invalid logo path
* Contributors: Marcel Debout

0.7.3 (2017-03-01)
------------------
* Fix: Installation failed du to return code 2
  udevadm control can return failure ($? != 0)
  When building docker containers, the or true does the trick
* Contributors: plieningerweb

0.7.2 (2017-02-23)
------------------
* Fix: Install udev rules
  Udev Rules usually installed with setup-usb.sh of tar.gz
  Without, camera will not be recognized in Ubuntu stock install
* Contributors: plieningerweb

0.7.1 (2017-02-14)
------------------
* Reviewing beetkeskin PR for GigE gamma
  - Formatted the code (deleted whitespaces)
  - Agreed to the fact that a non-accessible gamma is not always an error,
  so that returning true makes sense
  - Enabling gamma before checking if the NodeMap is available might solve
  the problem
* 0.7.0
* fixed type decive->device
* fix gamma handling for GigE cameras
  When connecting to a GigE camera (aca1920-50gc), the node crashes with "Error while accessing Gamma in PylonCameraImpl<CameraTraitT>": For some camera types, the Gamma settings are not available to the interface as they are handled automatically by the camera itself. This was already partly fixed for some gamma-related function calls, but not for all of them. This fix adds the missing checks. The behaviour is slightly changed: If the gamma is not set via user, gamma remains in auto mode (i.e. controlled by the camera). Once the user tries to set a gamma value, the gamma mode switches to user.
* Contributors: Magazino Version Daemon, Marcel Debout, Nikolas Engelhard, Stefan Kaiser

0.6.17 (2016-11-23 14:54)
-------------------------
* Bugfix: Wrong vector size (255 instead of 256)
* Contributors: Marcel Debout

0.6.16 (2016-11-23 13:38)
-------------------------
* Added upper brightness limit
* Contributors: Marcel Debout

0.6.15 (2016-11-23 10:04)
-------------------------
* Fixed crash in case of target brightness > 255
* Contributors: Marcel Debout

0.6.14 (2016-11-23 09:13)
-------------------------
* Reviewd parametrized timeout for ExposureSearch
  Lead to a better RaspPI support
* Refactor exposure time search to meet requ
* Add timeout param for brightness adjustment
  Add the optional parameter brightness_timeout to increase
  the time for the brightness search. Modified error massage
  to report the actual timeout.
* Contributors: Marcel Debout, Maxi Maerz

0.6.13 (2016-11-14)
-------------------
* Fixed non-working set gamma for GigE cameras
  Up to now, the setGamma() did not have an influence for GigE cameras,
  because one has to 'EnableGamma' first. Fixed that bug by moving from
  base-class to the usb and gige classes
* Contributors: Marcel Debout

0.6.12 (2016-11-08 17:45)
-------------------------
* Moved setup of exp-search before the first brightness is set
* Contributors: Marcel Debout

0.6.11 (2016-11-08 16:47)
-------------------------
* Reverted bullshit changes that broke the exp search
* Contributors: Marcel Debout

0.6.10 (2016-11-08 12:13)
-------------------------

0.6.9 (2016-11-08 09:13)
------------------------
* Fixed brightness calculation for color images
  For mono cameras, the subset calculation remains, for color images the
  brightness is calculated using all pixels and channels
* 0.6.8
* Fix for non-selectable gamma for some GigE cameras
* 0.6.7
* Updated changelog and README.rst
* Changed default behavior (no_ros_enc given)
  Non-provided encoding is indicated via empty string right-now.
  Default values are mono8 and rgb8 which are checked afterwards.
  Moved YUV422 support to 'future work'.
  Still TODO: - Update documentation
  - Fix brightness search that is evaluating various colored
  pixels for now
  See: https://github.com/magazino/pylon_camera/pull/7
  Resolves: AL-87
* First working color image version with Bayer Support
  - Moved imagePixelDepth() and the setEncoding() Method to the base
  implementation.
  - Added functionallity to detect and store the available image encodings from
  the used camera.
  - Added conversion methods to convert between ROS and GenAPI encodings
  Still TODO: - Update documentation
  - Test code with a camera that supports 'rgb8' and 'bgr8'
  - Provide 'bgr8' iamges in case the camera does not support
  'BGR8' but has 'YCbCr422_8' instead
  - Test brightness search
  See: https://github.com/magazino/pylon_camera/pull/7
  Resolves: AL-87
* Updated rectify image to support rgb8 encoding.
  Updated grabImage function to create "img_raw" variable with correct
  format based on current image encoding.
* Updated imageEncoding and imagePixelDepth function
  - Modified imageEncoding function to support RGB8 format.
  - Modified imagePixelDepth function to return correct pixel size based
  on current image encoding.
* Added function to set PixelFormat
  Baed on image_encoding\_ paramter, the function set appropriate
  PixelFormat depending on USB camera or GigE camera.
* Added image_encoding as parameter
  Added image_encoding as one of the parameters defined in yaml file. User
  can choose between "MONO8" and "RGB8".
* Contributors: Kazumi Malhan, Magazino Version Daemon, Marcel Debout

0.6.6 (2016-10-19)
------------------
* Merged in unstable/super_fast_brightness_search (pull request #2)
  Unstable/super fast brightness search
* Further micro-CleanUP
* CleanUp & Comments
* Downsampling is now working, fixed indices error
* Added brighntness exp LUT, to allow smart search
  Unstable version with lot's of debug output -> to be tested on the robot
* Continued working on the brightness speedup
  Fixed missing starting point offset in index calculation
  Added output to compare both methods
  Added imwrite to investigate the result
* Added idx vector to select subset of pixels
  Idea is that the brightness search does not have to calculate the mean
  of the entire image in every step, furthermore on a supset of pixels.
  Pixels will be selected like this:
  sampled img:   point:                                idx:
  s 0 0 0 0 0 0  a) [(e.x-s.x)*0.5, (e.y-s.y)*0.5]     a.x*a.y*0.5
  0 0 0 d 0 0 0  b) [a.x,           1.5*a.y]           b.y*a.x+b.x
  0 0 0 0 0 0 0  c) [0.5*a.x,       a.y]               c.y*a.x+c.x
  0 c 0 a 0 f 0  d) [a.x,           0.5*a.y]           d.y*a.x+d.x
  0 0 0 0 0 0 0  f) [1.5*a.x,       a.y]               f.y*a.x+f.x
  0 0 0 b 0 0 0
  0 0 0 0 0 0 e
  Resolves: TORU-1750
* Contributors: Marcel Debout

0.6.5 (2016-08-31)
------------------
* Added a script that calls the grab image action and publishes the result on on a sensor_msgs/Image topic
* Contributors: Ulrich Klank

0.6.4 (2016-08-24)
------------------
* setting image publisher queuesize to 1. If queue is to long and only single images are used (e.g. by waking up camera via set_sleeping, getting an image, setting to sleep again), old images are provided
* Contributors: Nikolas Engelhard

0.6.3 (2016-08-23)
------------------
* new script to toggle camera(s)
* Contributors: Nikolas Engelhard

0.6.2 (2016-08-16 16:12)
------------------------
* Changed new brightness request do ros_debug as it was creating a lot of output
* Contributors: Carsten Zumsande

0.6.1 (2016-08-16 15:06)
------------------------
* Changed new brightness request do ros_debug as it was creating a lot of output
* Contributors: Carsten Zumsande

0.6.0 (2016-07-28)
------------------
* Updated message dependencies
* Contributors: Magazino Version Daemon

0.5.4 (2016-07-26)
------------------
* Merged in user_output (pull request #1)
  User_output
* ros-linted the code, removed tabs
* Made set-user-output working finally! Still have problems, that USB cameras start counting with 1 and GigE-Cameras by 0, but created a workaround
* figured out, that basler enums are of type double, removed num_outputs member and replaced it with a vector containing the UserOutputselectorEnums -> Output '1' can now be set using 'vector.at(1)'
* added function that counts the number of available UserOutputs for the camera, have to test it for other devices
* starting to fix the setDigitalOutput functions for GigE cameras. Added member to the pylon_camera-class where the number of digital user outputs a camera provide will be stored. Still have to think of a way how to get this information, because they are highly dependend the used device and the used enums
* Contributors: Marcel Debout

0.5.3 (2016-06-28 07:41)
------------------------
* typo - thank God for jenkins
* Contributors: Marcel Debout

0.5.2 (2016-06-28 07:21)
------------------------
* corrected command line output in case that the default image encoding is not mono8
* Contributors: Marcel Debout

0.5.1 (2016-06-27)
------------------
* Fixed: Node claims to not have reached the desired brightness, but in fact it reached the brightness. Therefore trust in the pylon auto brightness search function and wait till it claims to be finished, instead of running into the timeout
* Contributors: Marcel Debout

0.5.0 (2016-06-23)
------------------
* Fixed a two bugs reported by andermi: Node crashes in case that the camera does not support binning. (fixed by previously checking if this feature is available) and setting the mono8 image encoding before the startGrabbing(), because afterwards it's assumend to be fix.
* Contributors: Marcel Debout

0.4.2 (2016-05-20 12:02)
------------------------
* minor fix: changed from global namespace to the one of the node
* Contributors: Marcel Debout

0.4.1 (2016-05-20 08:12)
------------------------
* Bugfix: filled empty 'encoding' field for images comming via the 'grab_images_rect'-action
* Contributors: Marcel Debout

0.4.0 (2016-05-12 15:24)
------------------------
* improved error handling for the grab_and_save action server
* Contributors: Marcel Debout

0.3.2 (2016-05-12 14:31)
------------------------
* added launch file for grab_and_save_image_as and print error instead of warning, in case no grab_image_raw as is found
* Contributors: Marcel Debout

0.3.1 (2016-05-12 14:11)
------------------------
* fixed copy-paste typo and added loginfo output
* Contributors: Marcel Debout

0.3.0 (2016-05-12 13:43)
------------------------
* Updated message dependencies
* Contributors: Magazino Version Daemon

0.2.9 (2016-05-12 13:41)
------------------------
* added action server which wraps the GrabImagesAction to be able to store the grabbed at desired location on the filesystem
* Contributors: Marcel Debout

0.2.8 (2016-05-11)
------------------
* Node dies no longer, if no device is available. Instead it keeps retrying to find a camera
* Contributors: Marcel Debout

0.2.7 (2016-05-10 18:37)
------------------------
* fixed wrong uri in rdmanifest file
* Contributors: Marcel Debout

0.2.6 (2016-05-10 17:09)
------------------------
* README.rst edited online with Bitbucket
* fixed wrong link name
* Contributors: Marcel Debout

0.2.5 (2016-05-10 15:32)
------------------------
* renamed empty tar
* Contributors: Markus Grimm

0.2.4 (2016-05-10 13:57)
------------------------
* Added required-empty.tar archive for rosdep
* Contributors: Markus Grimm

0.2.3 (2016-05-09 18:07)
------------------------
* README.rst edited online with Bitbucket
* Contributors: Marcel Debout

0.2.2 (2016-05-09 17:32)
------------------------
* Updated readme
* Contributors: Markus Grimm

0.2.1 (2016-05-09 16:17)
------------------------
* updated rosdep definitions for github
* Contributors: Markus Grimm

0.2.0 (2016-05-09 15:44)
------------------------
* Updated message dependencies
* Contributors: Magazino Version Daemon

0.1.1 (2016-05-09 15:40)
------------------------
* Updated message dependencies
* Added rdmanifest script to download pylon sdk
* Contributors: Magazino Version Daemon, Markus Grimm

0.1.0 (2016-05-09 09:08)
------------------------
* Updated message dependencies
* Contributors: Magazino Version Daemon

0.0.72 (2016-05-04)
-------------------
* basler-feedback: usage of the https:// origin for git clone to be able to use it without ssh key
* Contributors: Marcel Debout

0.0.71 (2016-05-03)
-------------------
* added loslaunch dependency to be able to check the launch files at build time
* Contributors: Marcel Debout

0.0.70 (2016-05-02 18:41)
-------------------------
* continued linting to reduce cpp-check errors
* Contributors: Marcel Debout

0.0.69 (2016-05-02 18:21)
-------------------------
* linting
* Contributors: Marcel Debout

0.0.68 (2016-04-29)
-------------------
* TORU-319: cleaned up cmake
* Contributors: Markus Grimm

0.0.67 (2016-04-26)
-------------------
* ROBEE-336: linting for result bag to action
* Contributors: zumsande

0.0.66 (2016-04-25 18:52)
-------------------------
* ROBEE-336
* Contributors: Ulrich Klank

0.0.65 (2016-04-25 16:42)
-------------------------
* Basler-Feedback: 'pylon' should be lower-case
* Contributors: Marcel Debout

0.0.64 (2016-04-19)
-------------------
* added missing camera_info_url description to the default config file
* Contributors: Marcel Debout

0.0.63 (2016-04-18)
-------------------
* README.rst edited online with Bitbucket,
  Added 'questions' section
* Contributors: Marcel Debout

0.0.62 (2016-04-14 18:01)
-------------------------
* fixed unhandled std::runtime_error in constructor: init() is now void, if something fails (no camera present) ros::shutdown() will be called. Furthermore added handling if grabImage() fails
* Contributors: Marcel Debout

0.0.61 (2016-04-14 17:01)
-------------------------
* write out namespace instead of assuming default
* Contributors: Marcel Debout

0.0.60 (2016-04-13 16:21)
-------------------------
* fixed launch file bug: tf frame should not contain '/', setting frame_id in case that the camera_info is parsed from the camera info manager
* Contributors: Marcel Debout

0.0.59 (2016-04-13 08:41)
-------------------------
* changed size of logos for the wiki.ros.org page
* Contributors: Marcel Debout

0.0.58 (2016-04-12 18:53)
-------------------------
* edited logo size for ros-wiki
* Contributors: Marcel Debout

0.0.57 (2016-04-12 18:31)
-------------------------
* added small logo for wiki.ros.org
* Contributors: Marcel Debout

0.0.56 (2016-04-12 17:31)
-------------------------
* README.rst edited online with Bitbucket
* README.rst edited online with Bitbucket
* Contributors: Marcel Debout

0.0.55 (2016-04-12 17:04)
-------------------------
* README.rst edited online with Bitbucket
* Contributors: Marcel Debout

0.0.54 (2016-04-12 16:51)
-------------------------
* README.rst edited online with Bitbucket
* Contributors: Marcel Debout

0.0.53 (2016-04-12 16:31)
-------------------------
* Added rosdep yaml
* Contributors: Markus Grimm

0.0.52 (2016-04-12 13:21)
-------------------------
* README.rst edited online with Bitbucket
* Contributors: Marcel Debout

0.0.51 (2016-04-12 12:21)
-------------------------
* magazino_id is now the device_user_id as in the pylon API
* Contributors: Marcel Debout

0.0.50 (2016-04-12 12:01)
-------------------------
* added CHANGELOG.rst, generated by catkin_generate_changelog
* Contributors: Marcel Debout

0.0.49 (2016-04-12 10:31)
-------------------------
* Updated readme
* Contributors: Markus Grimm

0.0.48 (2016-04-11 13:41)
-------------------------
* removed deprecated 'SetBrightnessSrv', 'SetExposureSrv' and 'SetSleepingSrv'. Please adapt to the new interface
* ROS_WARN instead of ROS_ERR if the desired brightness could not be reached
* Contributors: Marcel Debout

0.0.47 (2016-04-11 10:12)
-------------------------
* Code review
* Contributors: Markus Grimm

0.0.46 (2016-04-08 16:52)
-------------------------
* Changed dependencies for pylon to the new debian package
* Contributors: Markus Grimm

0.0.45 (2016-04-08 15:42)
-------------------------
* fixed premature commit
  TORU-623
* Handle constructor failures differently
  TORU-623
* Contributors: Ulrich Klank

0.0.44 (2016-04-07 18:06)
-------------------------
* init size_t with 0 instead of -1
* Contributors: Marcel Debout

0.0.43 (2016-04-07 17:42)
-------------------------
* readded HEader after rectification
* Contributors: Ulrich Klank

0.0.42 (2016-04-07 17:11)
-------------------------
* formatting & coding style
* Contributors: Marcel Debout

0.0.41 (2016-04-07 16:32)
-------------------------
* added parameter for inter-pkg-delay for RaspberryPI usage
* Contributors: Marcel Debout

0.0.40 (2016-04-07 15:32)
-------------------------
* linting
* Contributors: Marcel Debout

0.0.39 (2016-04-07 13:12)
-------------------------
* removed dublicated dependency
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera into opencv_rect
* finally added rectification support using the image_geometry::pinhole_model and the CameraInfoManager
* pulled intrinsic calib-reading from opencv_class
* first implementation with the CameraInfoManager
* fixed strange overriding behaviour in case that one requests brightness with auto_exposure and auto_gain set to false
* 0.0.36
* fixed console output of the timeout duration in brightness search
* 0.0.35
* removed unused member, found shorter name for the grabbing action server
* 0.0.34
* finally added rectification support using the image_geometry::pinhole_model and the CameraInfoManager
* pulled intrinsic calib-reading from opencv_class
* first implementation with the CameraInfoManager
* started to integrate rectification
* Contributors: Magazino Version Daemon, Marcel Debout

0.0.38 (2016-04-04)
-------------------
* removed double output in case that the intensity settig fails
* Contributors: Marcel Debout

0.0.37 (2016-03-31 15:56)
-------------------------
* fixed strange overriding behaviour in case that one requests brightness with auto_exposure and auto_gain set to false
* Contributors: Marcel Debout

0.0.36 (2016-03-31 15:31)
-------------------------
* fixed console output of the timeout duration in brightness search
* Contributors: Marcel Debout

0.0.35 (2016-03-31 09:53)
-------------------------
* removed unused member, found shorter name for the grabbing action server
* Contributors: Marcel Debout

0.0.34 (2016-03-30 16:11)
-------------------------
* renamed ActionServer to GrabImagesAS
* Contributors: Marcel Debout

0.0.33 (2016-03-30 15:51)
-------------------------
* added missing 'All rights reserved' tag, added LICENSE.rst file
* Contributors: Marcel Debout

0.0.32 (2016-03-30 15:11)
-------------------------
* README.rst edited online with Bitbucket
* Contributors: Marcel Debout

0.0.31 (2016-03-30 15:01)
-------------------------
* README.rst edited online with Bitbucket
* Contributors: Marcel Debout

0.0.30 (2016-03-30 14:44)
-------------------------
* moved all logos into one file
* Contributors: Marcel Debout

0.0.29 (2016-03-30 13:41)
-------------------------
* added missing wiki_images
* Contributors: Marcel Debout

0.0.28 (2016-03-30 13:31)
-------------------------
* new logos for the documentation
* README.rst edited online with Bitbucket
* Contributors: Marcel Debout

0.0.27 (2016-03-30 11:31)
-------------------------
* edited README, added license text to all files
* Contributors: Marcel Debout

0.0.26 (2016-03-30 10:22)
-------------------------
* moved README to .rst and merged package.xml
* README.md edited online with Bitbucket
* README.md edited online with Bitbucket
* Contributors: Marcel Debout

0.0.25 (2016-03-29)
-------------------
* implemented setBinning -> be careful: CamerInfo now changes binning_x & binning_y entry while the image height and width keeps static
* Contributors: Marcel Debout

0.0.24 (2016-03-17 14:21)
-------------------------
* size of provided data through GrabImagesAction should only be checked, if the corresponding 'is_given' flag is true
* Contributors: Marcel Debout

0.0.23 (2016-03-17 12:41)
-------------------------
* fixed mapping in GrabImagesAction from deprecated to new interface, fixed error in case that values are not provided and the resulting vector size is NOT 0, but 1
* Contributors: Marcel Debout

0.0.22 (2016-03-16)
-------------------
* smarter behaviour, if the goal values of the GrabImagesAction doesn't make sense
* Contributors: Marcel Debout

0.0.21 (2016-03-15 12:52)
-------------------------
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* warnings are not errors
* Contributors: Marcel Debout

0.0.20 (2016-03-15 11:02)
-------------------------
* compiles without warnings (no return value)
* merged the two branches
* adapted device removal behaviour
* 'is deprecated' error is now a 'is deprecated' warning'
* added deprecated handling of 'set_brightness_srv', 'set_exposure_srv' and 'set_sleeping_srv', which now can be found under 'set_brightness', 'set_exposure' and 'set_sleeping'. Furthermore the usage of 'SetBrightnessSrv.srv', 'SetExposureSrv.srv' and 'SetSleepingSrv.srv' is deprecated and should be switched to 'SetBrightness.srv', SetExposure.srv' and 'SetSleeping.srv'
* implemented setBinning as runtime parameter, but finally realized that the camera does not support it. Hence the camera has to be closed and reopened to be able to set the binning. This will be a future feature
* realized new fast opening behaviour, Basler-Feedback was: Sfnc is outdated, so I replaced it using the DeviceClass and the ModelName. Futhermore its possible to detect the desired camera without opening it twice
* increased fail_safe_ctr for dart cameras -> manual: up to 50 frames needed to reach target for dart cameras
* splitted grabImagesRawActionExecuteCB() in two methods, so that it can also be called from the derived PylonCameraOpenCV class
* moved output to #if DEBUG
* did lots of changes but finally I found a logic behaviour!
* linting & formatting
* added setGamma functionallity
* finally found out that the best is to keep default camera settings as long as possible. Added lots of commands to the default config file, hopefully one can verify my thoughts ;-)
* removed outdated scripts from CMakeLists.txt
* making roslint happy
* removed outdated scripts, brightness tests are coveraged in magazino_tests, exp_caller depends maru stuff
* removed test depend, all tests are done in magazino_tests/pylon_camera_tests
* finally got a state, where brightness tests for usb & gigE are running successfull, have still problems with dart cameras
* 0.0.17
* README.md wurden online mit Bitbucket bearbeitet
* removed has_auto_exposure\_ member, because this happens already in GenAPI::isAvailable(cam\_->ExposureAuto), added getter for cam\_->AutoGainUpper & Lower limit, added throwing of std::runtime_errors
* searching for autoBrightnessFunction stuck for dart cameras
* clean up dart
* disabled gainselector setting, because each gige cam has its differen naming
* removed senseless getCurrentExp, Gain... functions, correctly implemented setGain
* removed comments
* calling the grabImagesAction with differen exp-times will no longer affect the continiously published images
* further cleaning
* rows & cols are now size_t, removed unused checkForPylonAutoFunctionRunning()
* cleaning & renaming
* cleaned up the extended brightness search, works now very well!
* setExposure() on the pylon_camera-Object (not on PylonCameraNode) has now target and reached exposure
* enabled output
* fixed GainType-bug
* moved exp_search_params, continued working on brightness fix, still problems with dart
* CMakeLists.txt formatted
* dart camera starts with the same settings like the usb camera
* not all usb cameras have GainSelector_AnalogAll
* formatting
* seperated registerConfig, openCamera and applyStartupSettings
* added output regarding gain and exposure time, facing to problems in difference of usb and dart cams
* gain setting started, checking if gain db range gige equals usb
* check if auto function running not necessary any more
* brightness search now in a seperate thread, added lots of comments (and outpouts which i will remove when the gain stuff is working)
* removed auto-functions parameter limits for gige cameras
* gain for dart cameras not hard coded any more, one can set it in initializeing process using the ros-params
* changed order of setting target brightness value & setting the auto-funktion mode
* try to get rid of all these checkForAutoFuncitonRunning() functions using only one PylonCamera::isBrightnessFunctionRunning() method
* - output to check if auto-function still running
* - added const max allowed delta (tolerance) for the brightness search
  - switched from int-mean to float mean to decrease rounding errors
  - added comments / better readability
* further comments for brightness search
* 0.0.16
* Basler-Feedback: Prevent that the image will be copied twice:
  "
  Es handelt sich um ein Missverstndnis. Bei dem Ausdruck image = std::vector<uint8_t>(pImageBuffer, pImageBuffer + img_size_byte\_); passiert folgendes:
  1.  Konstruktor von std::vector<uint8_t>(pImageBuffer, pImageBuffer + img_size_byte\_) aufrufen (1. Kopie der Bildaten)
  2.  Zuweisungsoperator von image aufrufen (2. Kopie der Bildaten)
  3.  Destruktor von std::vector<uint8_t>() aufrufen (1. Kopie wird verworfen)
  Der Compiler hat unter Umstnden die Mglichkeit hier zu optimieren, wenn die verwendete STL und der Compiler C++11 untersttzt. Da ab C++11 der Move Assignment operator (In der Mail stand Move Constructor) verfgbar ist (class_name & class_name :: operator= ( class_name && ) und der Compiler wei das der R-Value std::vector<uint8_t>() nicht weiter referenziert wird, kann er einen Kopierschritt vermeiden.
  Vorschlag, einfach folgenden Ausdruck:
  image.assign(pImageBuffer, pImageBuffer + img_size_byte\_);
  statt:
  image = std::vector<uint8_t>(pImageBuffer, pImageBuffer + img_size_byte\_);
  verwenden und das Problem ist erledigt.
  "
* removed brightnessValidation() because it's a one-liner
* activated new waitForCamera() function
* added waitForCamera(), which waits for pylon_camera\_->isReady() observing a given timeout
* comment on isReady()
* Basler-Email: cam\_->GetNodeMap().InvalidateNodes() should never be necessary, so I removed it
* resorted methods
* added comments
* Contributors: Magazino Version Daemon, Marcel Debout, Nikolas Engelhard

0.0.19 (2016-02-29)
-------------------
* new device removal behaviour
* Contributors: Marcel Debout

0.0.18 (2016-02-25)
-------------------
* try to catch the logical error exception in grabImagesRawExecuteCB()
* Contributors: Marcel Debout

0.0.17 (2016-02-19)
-------------------
* README.md wurden online mit Bitbucket bearbeitet
* Contributors: Nikolas Engelhard

0.0.16 (2016-02-02)
-------------------
* Basler-Feedback: Prevent that the image will be copied twice:
  "
  Es handelt sich um ein Missverstndnis. Bei dem Ausdruck image = std::vector<uint8_t>(pImageBuffer, pImageBuffer + img_size_byte\_); passiert folgendes:
  1.  Konstruktor von std::vector<uint8_t>(pImageBuffer, pImageBuffer + img_size_byte\_) aufrufen (1. Kopie der Bildaten)
  2.  Zuweisungsoperator von image aufrufen (2. Kopie der Bildaten)
  3.  Destruktor von std::vector<uint8_t>() aufrufen (1. Kopie wird verworfen)
  Der Compiler hat unter Umstnden die Mglichkeit hier zu optimieren, wenn die verwendete STL und der Compiler C++11 untersttzt. Da ab C++11 der Move Assignment operator (In der Mail stand Move Constructor) verfgbar ist (class_name & class_name :: operator= ( class_name && ) und der Compiler wei das der R-Value std::vector<uint8_t>() nicht weiter referenziert wird, kann er einen Kopierschritt vermeiden.
  Vorschlag, einfach folgenden Ausdruck:
  image.assign(pImageBuffer, pImageBuffer + img_size_byte\_);
  statt:
  image = std::vector<uint8_t>(pImageBuffer, pImageBuffer + img_size_byte\_);
  verwenden und das Problem ist erledigt.
  "
* Contributors: Marcel Debout

0.0.15 (2016-02-01 15:33)
-------------------------
* added comment
* moved cam-info setup into new method
* Contributors: Marcel Debout

0.0.14 (2016-02-01 08:22)
-------------------------
* fixed brightness assertion bug: spinOnce() does not result in a new image in case that no subscriber listens to the image topic
* assertion before accumulating
* Contributors: Marcel Debout

0.0.13 (2016-01-25 17:03)
-------------------------
* set gain implemented for gige
* Contributors: Marcel Debout

0.0.12 (2016-01-25 13:32)
-------------------------
* added lots of comments, initialized the camera_info_msg with zero-values
* Contributors: Marcel Debout

0.0.11 (2016-01-21 18:02)
-------------------------
* removed roslint
* Contributors: Markus Grimm

0.0.10 (2016-01-21 15:22)
-------------------------
* SetUserOutput is now a service
* Contributors: Markus Grimm

0.0.9 (2016-01-21 11:51)
------------------------
* README.md edited online with Bitbucket
* Contributors: Nikolas Engelhard

0.0.8 (2016-01-19 18:54)
------------------------
* fixed segfault if no camera-present-bug
* undo set gain for gige
* Contributors: Marcel Debout

0.0.7 (2016-01-19 18:23)
------------------------
* gain to 100 for gige hotfix
* Contributors: Marcel Debout

0.0.6 (2016-01-18 11:02)
------------------------
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* catkin_lint fix
* Contributors: Marcel Debout

0.0.5 (2016-01-18 10:36)
------------------------
* removed all tests, they are now in the new package: pylon_camera_tests to resolve can-dependency-problem
* Contributors: Marcel Debout

0.0.4 (2016-01-15 18:41)
------------------------
* Reviewed ROBEE-212: Found the missing part in order to use the trait
* Removed compaibilty_exposure_action.py as it is outdated (it used the old pylon_camera_msgs package)
* Contributors: Markus Grimm

0.0.3 (2016-01-15 17:12)
------------------------
* Robee-212: Support for setting the digital output pin of USB (non-Dart) and GigE cameras. So far, the std_msgs/Bool topic output_1 can be used to set the pin. Only tested on USB3-Ace camera "
* Contributors: Nikolas Engelhard

0.0.2 (2016-01-13)
------------------
* formatted cmakelist
* check if env: ON_JENKINS_TESTRIG=true before running the tests. if not, tests will have state: 'SUCCESS', but the number of test remains 0
* removed useless error-msg if no camera is present
* Contributors: Marcel Debout

0.0.1 (2016-01-11)
------------------
* Deleted maru_frame_rate_tester.py
* Merge branch 'feature/pylon5' of bitbucket.org:Magazino/pylon_camera into feature/pylon5
* re-enabled tests
* Finally we have a find script for pylon. jeah!
* lint
* own Sfnc-Header no longer needed
* Pylon::autoInitTerm was gone, is now replaced by Pylon::PylonInitialize() and Pylon::PylonTerminate()
* compiles with pylon5
* made single_acquisition_test.py executable
* added further tests and all 3 types of cameras to the jenkins
* fixed duplicated output
* making roslint happy, removed not working 'build/include_what_you_use filter'
* reset version information
* fixed open_wrong_cam bug
* format
* check if shutter-mode is available for the cam
* improved script for bag to action
* node to convert a bag to a action server again
* support for shutter mode added. So far only tested with Pylon that somehow only supports rolling shutter (although global reset is working in PylonViewer)
* fixed format string
* package.xml, moved rostest from set() to find_pacakage()
* fixed ROS_ERROR with wrong arguments
* Make catkin_lint happy again
* CMakeLists corrected
* writing binning-value into camera_info_msg
* fixed typo (fist/first)
* float is not a valid type for ros params, double is
* requesting lower framerate
* using device_user_id instead of magazino_cam_id
* longer timeout for camera test
* no more empty frame in grabImagesRawActionExecuteCB()
* added header_frame to action based rect images
* removed / for gige namespace
* comments, moved mtu param to /gige/ namespace
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* removed default_camera launch file which was outdated
* README.md edited online with Bitbucket
* README.md edited online with Bitbucket
* README.md edited online with Bitbucket
* README.md edited online with Bitbucket
* README.md edited online with Bitbucket
* README.md edited online with Bitbucket
* merge
* merge
* removed hard coding
* pull from master & review
* new calibration-yaml (so far not used)
* test case now opens dedicated test camera (basler dart), attached to test server
* test script now executable
* corrected catkin lint issues
* Added binning feature
* master merge
* removing __init
* new folder for test scripts
* resettes changes on magazino_cam_id
* added missing suffix in CMakeLists:
* added dependency for rostest
* renaming magazino_cam_id to device_user_id
* renamed program to write cameraname so that it corresponds better to the official naming of pylon ( 'DeviceUserID'), removed magazino-specific check of naming convention
* renamed program to write cameraname so that it corresponds better to the official naming of pylon ( 'DeviceUserID'), removed magazino-specific check of naming convention
* started work on ros tests. First test opens random camera and verifies that an image and camera_info is sent
* Removed grabSequence
  Fixed an issue in the setExposure function
  Removed the desired_exp_times parameter which is now part of the opencv node
* Renamed is_sleeping
  updated readme and default config file
* fixed pixel depth error
* Make catkin lint happy
* make roslint happy
* Added doxygen comments
  Code cleanup
* Updated launch file to use a separate yaml file for parameters
* removed wrong comment, check for valid initial grab result
* undo raspi specific configuration
* fixed trigger <-> result confusion
* retrieving result success
* removed .idea folder
* README.md edited online with Bitbucket
* README.md edited online with Bitbucket
* moved spin() to the top, added output
* removed GrabSequenceAction which is now in GrabImages, renamed params\_ into pylon_camera_parameter_set\_, moved init() into constructor
* moved init() into constructor, cleaned code
* README edited
* README v0.01
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera into action_trigger
* added test
* realized optional action based grabbing
* moved parameter reading to the parameter class
* removed sensless auto_brightness = -2 and auto_exp = -2 value
* added comments and return false, if registerconfig fails
* mtu size now in launch file, default is 3000, inter-package-delay increased, but sitll hard coded
* removed MaxRetryCountRead & MaxRetryCountWrite Value -> keep default
* set fix grab timeout of 5s and removed fuzzy cam-specific timeout-funcitons
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* first basler-debug-day results
* camera now also opens if no camera_name was written into it
* correcte usage of cmake source directory
* Sigint Handler disabled
* ctrl-c handler
* shorter return code in brightness search method
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* fixed getCurrenCurrentExposure() typo, wrote return value shorter
* lint
* pylon includes are now marked as SYSTEM includes so that no warnings are printed for them
* renaming: pylon_camera_msgs to camera_control_msgs
* Updated SequenceExposureTimes publisher to new message
* Updated action server message field name
* Splitted the package into pylon_camera, pylon_camera_opencv and hdr_image_utils
* some fixes for sequencer
* fo
* new script to request an image sequence and write it to a folder
* new script that answers image_sequence-actions with files from a folder (work in progress)
* support partial names? not completely working
* param tuning
* Changed to pkgconfig for pylon4
* hdr parameter tuning
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* parameter tuning for toru_0003_sol
* Tuned exp. times
* new calib for cam with filter, new exp times, removed sequencer imwrite
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* new calib for teststand with filter
* reduced log level
* changed parameters
* adapted toru_sol_camera.launch to new load_calib script, camera name is now a parameter
* new calibration
* Fixed brightness service using locks
* testastand calib with acA2000-50gm
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* Pylon camera now compiles with opencv2 again.
  Bugfix: brightness and exposure servers were not working
  Bugfix: pylon camera now compiles without opencv support if opencv could not be found
* added script to simplify loading of intrinsic calibration from db in launch file
* new intrinsic calib for SOL-test env
* Delete old wide angle camera calibration file
* Set start exposure for spectral dart
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* added std:: in the header, reduced start exposure for default camera (intrinsic calib)
* updated toru camera
* Fixed merge mertens algorithm. Matrices need to be manually locked.
* new launch files and new calib for sol-teststand with caA200-50gm & 6mm Lensation
* be quiet cmake
* added another exposure time to hdr
* Updated exposure times
* Possible fix for sequencer images
* Merge
* TORU-148: Rewrote pylon_camera backend. Thank you Basler for all these interface incompatibilities.
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* default launch file for intrinsic calibration
* catkin_lint fixes
  * move mistyped message out of include_directories command
  * don't modify CMAKE_BUILD_TYPE and CMAKE_CXX_FLAGS
  * add build_depends on image_transport and cv_bridge
* added rand as  runtime dependency
* Copied the merge mertens algorithm from opencv, optimized the code and parallelized the computation steps.
  Removed using cv/std etc. from header files.
  Removed OpenCV3 stuff from CMakeLists.txt as we do not need OpenCV 3 anymore in this package ;)
* Added missing dependencies to package.xml. Added pylon4 system dependency which is now installable via rosdep
* hdr parameter tuning
* added additional throttle topic
* (Commiting for somebody else)
  Changed framerate and added throttle for HDR image
* Removed ros_info statement
  Added link to exposure fusion paper
* Implemented a basic HDR algorithm to speed up the HDR generation
  Added some const and & where it may make sense
  Added some if statements to rectify images only if somebody subscribes to the topic
* now using hdr
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* launch files renamed (commit by marcel without rsa_key ;-)
* launch files renamed (commit by marcel without rsa_key ;-)
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* added new calib for new acA1920-40gm with 6mm Lensation Lens
* Added cv_bridge dependency if pylon node is built with opencv
* added launch file for stand-alone sol teststand
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* new launch file for sol standalone test case
* Bug fix in naming
* Delete dart_wide.launch, replaced
* Rename dart_wide to toru_spectral_dart launch file
* Set magazino cam id for wide angle camera
* Deactivate auto gain for DART cameras
* Fixed dependency issues.
* Added new intrinsic calibration file
* launch and calib file for dart camera for galvo laser tests
* new launch
* better gitigonre
* correctly edited sequence launch file
* correct opencv version check output
* cmake now searches for OpenCV 3 first, if fails for opencv 2 -> before: although OpenCV 3 installed, find_package(OpenCV) only detects OpenCV 2 which comes with ROS
* SERSOL-11: Implemented HDR for GIGE cameras.
* added named for nav eval camera, added respawn to sheet of light camera
* New calibration for nav_eval_dart_cam.
* added name to sol camera launch
* Added new calibration files for sol camera
* Launch file for wide angle dart
* Add missing image size to calib file
* Calibration of wide angle dart
* trying to solve django setup problem in cmd line
* enabled compiling on i686 architectures, fix compiling without openCV
* added calibrated transform to toru launch
* new launch file which uses the right marker
* added camera id
* calibrated lamp dart as TORU_0002_temp, added to calib result to dart launch
* removed debug imwrite in hdr generator
* Merge branch 'nav_eval'
* Merge branch 'nav_eval' of bitbucket.org:Magazino/pylon_camera
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera into nav_eval
* services in opencv case
* TORU-5: Added calibration- and launch-file for dartcam which should be used for MIRA evaluation.
* brightness & exp server only available if in non-sequencer mode
* bugfix: pylon_interface is ready after the first grab(cv::Mat) call
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* usb sequencer working
* added seq exp times parameter, fixed bug: set exp in sequencermode
* added python setup for connection test script, adapted launch fiel to try to support older djangoe version
* comment on max retry counter
* frame rate tester will be launched with crane_camera.launch
* frame rate tester writes result to file
* fixed bug: brightness service has to wait until at least one img is grabbed
* start exp in launch file edited
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* start exposure (ros-param)  will be initially, including range check for all cam types
* removed django setup, maybe reuired?, fix
* reactivated comptibility node, minor changes in launch files, error message
* added frame rate tester for maru
* removed skipping warnings 2
* removed skipping warnings
* moved SfncVersion to local pylon include and added warning
* moved SfncVersion to local include
* arm fix
* also arm not only for 'write_magazino_id_to_cam'
* arm adaption
* fixed formatting merge conflicts, fixed intrinsic_calib_loader init order
* remove formatting
* code cleaning
* commit to try on maru
* mean without opencv
* mean without opencv
* removed brightness parameter -> control brightness & exposure only using the service
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* Launch file to start dart camera
* Add calibration file for dart camera
* removed some warnings
* fixed uninitialized pylon_interface\_* bug
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* small script to compute brightness of image to e.g. show in rqt_plot
* launch file renamed
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* added architecture check -> other pylon library path for arm
* added support for toru camera
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* checking if exposure mode on camera was set
* Correctly set WITH_OPENCV option to OFF, if no OpenCV is installed
* warnings eliminated
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* gige max retry counter set to 6, retrieve result timeout changed from max val to current val, own_brighntss_search param added
* added ReadMe explaining manually copy of pylon-header
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* script to check brightness service
* CMake file cleaned
* If OpenCV Version < 3, will not compile HDR
* transport-layer retry sending/receiving 5 times (before 2) to prevent lost frames
* prevent 'isdeprecated' warnings
* merge with ulis fixes
* change from init to setupSequencer
* several minor bug fixes
* keep compatibility node for python scrips
* added hdr for usb-cam
* launch files
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* brightness service wont send true until target really reached
* revert
* publishing camera device name as parameter
* second fix for 'The image buffer was incompletly grabbed' bug'
* fixed 'The image buffer was incompletly grabbed' bug'
* tried to fix 'CreateFirstDevice' Bug
* edited sol_cam.launch
* kais changes where gone -> build pylon_camera_msgs before pylon_camera
* service in new thread -> brightness search response will be send when target reached
* workaround: no new images will be send while own auto brightness function running
* code formatted
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* auto brightness working
* return values now set for compat node
* hopefully fixed merge problems
* improved interface for exp_caller (and no default camera anymore)
* removed old trigger srv
* Added sleeping service: set_sleeping = true -> pause grabbing images
* brightness as reference
* workaround not working brightness srv
* fixed usage of wrong service
* updates
* works now with yaml file entries 'rows & cols' and 'width & height'. ULI -> pleese adapt if uncorrect
* works now without opencv support. PROBLEM: WITH_OPENCV:  wrong 'setupExtendedExposure()' function call -> extended auto brightness function not working
* fixed dart segfault
* working on WITHOUT_OPENCV support
* brightness service working
* service still not working
* fixed double corruption bug
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera into review_marcel
* pylon_interface = pointer, undo ulis time out change, new problem: low framerate
* works now with yaml file entries 'rows & cols' and 'width & height'. ULI -> pleese adapt if uncorrect
* added dependencies to make sure messages are built first
* works now without opencv support. PROBLEM: WITH_OPENCV:  wrong 'setupExtendedExposure()' function call -> extended auto brightness function not working
* fixed dart segfault
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera into review_marcel
* working on WITHOUT_OPENCV support
* added compatibility node, improved exp caller, removed cyclic output
* brightness service working
* service still not working
* fixed double corruption bug
* added pylon update file, minor changes
* bgr not yet implemented
* renamed file, move to src
* made launch files user independant
* bugfix: if no intrinsic yaml data in opencv case -> publish only image raw
* CMake adapted: if could not find opencv, will automaticly build without opencv support
* Version 0.1: Usb-Cameras working
* working on extended auto exposure and software design
* sequencer working the first time
* ROS Style Guide
* ready for review
* tmp
* rectification working
* set exposure in mu_s working
* Desired Cam using magazino_device_id, support for USB and Dart Cameras
* bugfix -> before: crash if no intrinsic calib loaded (out of mem)
* added cam-matrix to initUndistortRectifyMap -> same result as undistort (without shift)
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera into sol_demo
* runtime -90ms, fast undistortion by replacing cv::indistort with cv::initUndistortRectifyMap and cv::remap
* Pylon Node now working with Basler USB3.0 Camera
* Pylon Node now working with Basler USB3.0 Camera
* initialize camera pointer
* last commit just before bille move
* small fix for failed exposure
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* better errormsg if exposure failed
* added ocr cam launch file
* small fix
* added condition for second camera open
* more output when opening camera, no camera is opened if identifier is not unique
* less debug info
* small fixes
* using old method for usb
* new launch file for defaul camera
* work on native exposure calibration, Problems with USB
* towards better auto exposure
* intrinsic calibration yaml now also supports a comment-string. This can be used to easier find wrong connections (crane camera loads calibration of insertion cam)
* crane camera now also only looks for ip
* new launch files for maru2 (won't live long)
* allow to use IP only as camera identifier
* removed second entry for camera_name
* demo tag
* IFDEF DB for calibexposure action
* adapted to book_gripper
* added smoke test
* no default parameter for cam id, new launch file for kado camera
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* new script to test exposure client
* if in launch write_calib_to_db is set, exp/brightness pairs are written in table crane_exposure (should only be set for crane camera
* should compile for Maru usage
* has_auto_exposure will be asked after opening the camera
* working with cam acA1920-155um
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera into book_gripper
* working without DB
* merge
* fo
* new launch for laser camera
* added missing opening command
* removed SensorID'
* now also working with cameras that do not provide auto_exposure
* bugfix. new max_exposure was always set to 1sec after search converged
* removed default value for param_file
* changed respawn times and node names
* maybe speedup
* new max exposure of 915000
* new max exposure of 915000
* merge
* less debug
* lesse debug
* no exposure via msg, only via action
* more exp
* exposure action
* new launch files
* towards exp action
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* small fix for better nodehandle
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* towards exposure action
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* new camera id for crane camera
* higher timeout to enable longer exposure, ErroR msg if requested exposure is invalid (current max is at 916000)
* exposure calibration works
* now with functionality to calib exposure
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* new exposure is checked every frame and update on the camera on change
* changed name of pylon camera node and some parameters
* small bugfix: handling missing yaml-file
* launch files (again) with respawn_delay of 30s, node now works with usb and gige camera (so far, only exposure can be set)
* towards usb
* now with cmake-option for QT-sql
* merged
* added 'add_definitions(-DWITH_QT_DB)' to CMakeLists.txt -> db-libs were only linked if they were needed
* pulled from master, saved merge conflicts
* interface now working for usb and gige camera, exposure can be set again. new define WITH_QT_DB in PylonCameraInterface.h that decides if sql connection is used. TODO: move define into CMakeLists.txt and only link to db-libs if needed
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* Merge branch 'cinstantcamera' of bitbucket.org:Magazino/pylon_camera into cinstantcamera
* switched to CInstantCamera
* corrected crane launch for camera
* including usb cameras
* better names for camera launch files
* speed up, now only publishing if someone is listening
* added respawn and default exposure fpr pylon
* new param for intrinsic camera id (as given in db)
* less debug outout, exposure default to 500 mu s
* exposure-param is read every 10 frames
* removed some old debug, all other msgs are now ROS\_*, exposure in mu s, params in launch-file are now listed before node
* now with new param: pylon_exposure_mu_s to set exposure. A negative values enables auto-exposure
* removed debug
* added check if camera_frame is in tf-tree
* added camera-frame parameter to launch file
* Merge branch 'master' of bitbucket.org:Magazino/pylon_camera
* merged, now with frame as parameter
* added fixed exposure
* added fixed exposure
* using new reference frame name
* added option to set exposure to fixed value
* generalized camera selection for a distinct camera
  Conflicts:
  CMakeLists.txt
* Added CATKIN_IGNORE to .gitignore
* fixing install targets
* added install target for launch file
* small adaptions for new sqlconnection
* now also publishes camera_info and undistorted image so that camera can be visualized in rviz with projections
* now with correct timestamp (using software trigger)
* more output
* time of last img now written to DB
* corrected link error to sqlconnection
* now with launch file
* node now sends image via ros, connection to cam is closed if node is terminated
* initial commit of ROS pylon interface to basler camera
* Contributors: Carsten Zumsande, Kai Franke, Marcel Debout, Markus Grimm, Maru2, Mehdi, Nikolas Engelhard, Nils Berg, Philipp Schmutz, Roman Mansilla, SCITOS Demo User, Tobias Wohlfarth, Ulrich Klank, zumsande
