### Radar Drivers

This directory contains the driver for the radar sensor (UMRR-11), Project ASLAN has been tested and validated for.
This ROS driver requires additional steps before the build. For this reason, the CMAKE is skipping the build on this driver.

#### Build
In order to build this radar driver, delete the file: "CATKIN_IGNORE" from its directory.  
Then follow the instructions as described on the README at the /umrr_driver directory, to setup your environment and install the prerequisites. After you have successfully completed the installation steps, the radar ROS driver will be installed.