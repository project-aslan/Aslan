^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pointgrey_camera_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.14.1 (2020-05-05)
-------------------
* [pointgrey_camera_driver] Added future for ARM builds.
* Update 60-pgr.rules
  Added another common vendorID
* Contributors: Jeff Schmidt, Tony Baltovski

0.14.0 (2020-04-03)
-------------------
* [pointgrey_camera_driver] Switched bionic to amd64.
* Use path.join instead of manually inserting a /
* Whitespace cleanup
* Add the crunch-bang python line so Atom's syntax highlighting plays nicely
* Rewrite the download_flycap script so that it downloads packages from the correct endpoint. Drop support for Trusty, add future support for Bionic
* Add messages as build dependencies for driver executables (`#153 <https://github.com/ros-drivers/pointgrey_camera_driver/issues/153>`_)
* update to use non deprecated pluginlib macro (`#150 <https://github.com/ros-drivers/pointgrey_camera_driver/issues/150>`_)
* Contributors: Chris I-B, Kevin Allen, Mikael Arguedas, Tony Baltovski

0.13.4 (2017-10-26)
-------------------
* Build jessie binaries with the legacy SDK as well.
* Contributors: Mike Purvis

0.13.3 (2017-10-03)
-------------------
* Assume current library version for all OSes other than trusty. (`#146 <https://github.com/ros-drivers/pointgrey_camera_driver/issues/146>`_)
* Contributors: Mike Purvis

0.13.2 (2017-09-28)
-------------------
* Adding camera consistency exception. (`#142 <https://github.com/ros-drivers/pointgrey_camera_driver/issues/142>`_)
* Use find_path to look for system headers. (`#139 <https://github.com/ros-drivers/pointgrey_camera_driver/issues/139>`_)
* Added support to RGB8 pixel format and controls for sharpness and saturation (`#116 <https://github.com/ros-drivers/pointgrey_camera_driver/issues/116>`_)
  * Added support to RGB8 pixel format and controls for sharpness and saturation
  * removed whitespaces and reverted changes to camera launch
* Consolidate udev rules. (`#131 <https://github.com/ros-drivers/pointgrey_camera_driver/issues/131>`_)
* Require flycapture library and header files (`#112 <https://github.com/ros-drivers/pointgrey_camera_driver/issues/112>`_)
* Add multi-release support, bump SDK to 2.11.3.121 for xenial and 2.9.3.43 for trusty (`#127 <https://github.com/ros-drivers/pointgrey_camera_driver/issues/127>`_)
* Contributors: Bei Chen Liu, Mike Purvis, Mohammed Al-Qizwini, Vitor Matos, kmhallen

0.13.1 (2017-04-19)
-------------------
* Add flycapture2 lib download rule for aarch64(e.g. Jetson TX1) (`#93 <https://github.com/ros-drivers/pointgrey_camera_driver/issues/93>`_)
* Make flycap d/l logic work on re-build. (`#109 <https://github.com/ros-drivers/pointgrey_camera_driver/issues/109>`_)
* Fix missing roslaunch dep, package format 2.

0.13.0 (2017-03-17)
-------------------
* Adding param to provide path to serial location.
* Fix flycapture install when using local copy.
* Fix thread leaking issue.
* Added stereo launch file and enabled roslaunch checkes for all launch files.
* Enabling GigE packet resend by default (`#97 <https://github.com/ros-drivers/pointgrey_camera_driver/issues/97>`_)
* Enabling 16bit Bayer format (`#98 <https://github.com/ros-drivers/pointgrey_camera_driver/issues/98>`_)
* Allow strobe shorter than 1ms (`#99 <https://github.com/ros-drivers/pointgrey_camera_driver/issues/99>`_)
* Add option to strobe on GPIO2. (`#91 <https://github.com/ros-drivers/pointgrey_camera_driver/issues/91>`_)
* Added Format7_Mode5 and Format7_Mode7 options for Video Modes. (`#90 <https://github.com/ros-drivers/pointgrey_camera_driver/issues/90>`_)
* Improve white balance initialisation and logic. (`#95 <https://github.com/ros-drivers/pointgrey_camera_driver/issues/95>`_)
* Added Format7_Mode4 option for Video Modes. (`#87 <https://github.com/ros-drivers/pointgrey_camera_driver/issues/87>`_)
* Added new camera model (`#86 <https://github.com/ros-drivers/pointgrey_camera_driver/issues/86>`_)
* Added the option to use the format7_mode4. (`#84 <https://github.com/ros-drivers/pointgrey_camera_driver/issues/84>`_)
* Adding string parameter fallback. (`#81 <https://github.com/ros-drivers/pointgrey_camera_driver/issues/81>`_)
* Contributors: Benjamin Blumer, Brian Holt, Devon Ash, Enrique Fernandez, Kazumi Malhan, Léonard Gérard, Mike O'Driscoll, Nicola Castaman, Tony Baltovski, Tracey Spicer, fabrizioschiano

0.12.2 (2016-09-30)
-------------------
* Reconnect on error (`#79 <https://github.com/ros-drivers/pointgrey_camera_driver/issues/79>`_)
* Update to FlyCapture v2.9.3.43 (`#65 <https://github.com/ros-drivers/pointgrey_camera_driver/issues/65>`_)
* Contributors: Anass Al, Enrique Fernández Perdomo, Konrad Banachowicz

0.12.1 (2015-11-06)
-------------------
* Depend on curl to pull in ca-certificates.
* Specify color coding. Without the format7 color coding specified, the driver will crash.
* Adds the vendor ID for Startech-brand Firewire interface cards.  This is necessary for accessing the camera(s) connected through the card.
* Removing check for number of subscribers to publish raw image.
* Support cameras with high framerate.
* Contributors: Jeff Schmidt, Konrad Banachowicz, Mike Purvis

0.12.0 (2015-04-22)
-------------------
* Remove dependency on driver_base.
  Define SensorLevels constants directly in the relevant places, rather
  than using the external message for this.
* Improve list_cameras by outputing more information about it
  The previous list_cameras only output 1 serial number which is
  not very informative. The improved one will print serial, model,
  vendor, sensor, resolution, color and firmware version.
* Add auto white balance and fix not able to write white balance
  Fix the problem of not being able to set white balance using Property.
  When trying to set white balance on my FL3-U3-13E4C-C, both this ros
  driver and flycap software cannot set the white balance naturally.
  After playing around with the flycap software, I found that I have
  to set the highest bit of register 80C (which is white balance) to 1
  to enable this feature. So I modified the original SetWhiteBalance to
  use WriteRegister directly. And add support for auto white balance.
* Framerate improvements.
* Support downloading the SDK for ARM.
* Downgrade flycaptyre SDK to 2.6.3.4, see:
  https://github.com/ros-drivers/pointgrey_camera_driver/issues/28
* Contributors: Chao Qu, Julius Gelšvartas, Konrad Banachowicz, L0g1x, Mike Purvis

0.11.0 (2014-11-07)
-------------------
* Change approach to downloading flycapture SDK.
  The logic which fetches and extracts the archive from ptgrey.com
  has been moved to a more reasonable and comprehensible python script.
  This should better pave the way for better future ARM support in this
  driver.
* Use dh_installudev for udev rules.
* The raw and mono pixel formats (raw8, raw16, mono8, mono16) can be selected from dynamic reconfigure with every video mode (while before it was hard coded that only mono pixel formats could be used with mode1 and mode2).
* Binning information removed from camera_info published by the nodelet.
* Add image_proc as dependency.
* Removed changes to binning_x and binning_y in camera info messages (otherwise image_proc node would performs a further downsampling).
* Now the wrapper allows to set raw and mono pixel formats with any mode.
* Added possibility to set GigE packet delay as launch/conf parameter.
* Changed 'auto_packet_size' to 'true' as default.
* Added possibility to change GigE packet size for GigE cameras.
* For GigE cameras, automatically discover best packet size.
* Fix launch file syntax error (XML comments neither nest nor continue).
* Contributors: Aaron Denney, Jeff Schmidt, Matteo Munaro, Mike Purvis

0.10.0 (2014-08-18)
-------------------
* Added frame rate parameter to launchfiles.
* Fixing lack of dynamic Bayer format detection/incorrect fixed Bayer format detection in the stereo driver, tested on BB2 hardware
* Should prevent multiple camera nodes from conflicting.
* Read camera's resulting trigger configuration.
* Read camera's resulting strobe configuration.
* Refactor GPIO pin comparison into separate function.
* Support outputting strobes.
* Enable altering trigger polarities.
* Don't overwrite currently unused fields.
* Modify firewire rule per issue `#6 <https://github.com/ros-drivers/pointgrey_camera_driver/issues/6>`_
* Make sure camera properties are supported before enabling them
* Contributors: Aaron Denney, Jake Bruce, Jeff Schmidt, Mike Purvis, Ryan Gariepy

0.9.2 (2014-07-13)
------------------
* Added dpkg to build_depend
  During builds, dpkg is explicitly called. This tool is not necessarily on all systems, so we should make sure it is installed during the build.
* Contributors: Scott K Logan

0.9.1 (2014-03-12)
------------------
* Add note to the list_cameras tool about restarting udev.
* Add debayering nodelet to example launcher for monocular camera. Tested with a USB Firefly.
* Automatic lint fixes from astyle.
* Set ROS message image encoding to the bayer format declared by the camera.
* Contributors: Mike Purvis

0.9.0 (2014-02-26)
------------------
* Remove pgrimaging from all USB devices.
* Rename standalone executables, fix priority of udev rules for USB cameras, parameterize example launchfiles better.
* Contributors: Mike Purvis

0.0.2 (2014-02-26)
------------------
* Permissions to world-readable for firewire devices.
* Add nodelet manager to example launch.
* Reorganize bumblebee example launcher.
* Fix installing to i386.
* Contributors: Mike Purvis

0.0.1 (2014-02-23)
------------------
* Fetch FlyCap dependency from pygrey.com at configure time.
* Add PGR udev rules from the flycap installer.
* Catkinize main package.
* Added code for a ROS-compatible point-grey camera driver based on flycap.
* Contributors: Chad Rockey, Dave Bradley, Mike Purvis
