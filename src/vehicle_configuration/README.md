# Vehicle_configuration

## Getting Started

This folder contains:

- Vehicle description parameters
- The vehicle_model package:  
 This contains a 3D .stl model for each of our supported cars, in addition to a urdf file for visualization. Launch files are also provided for each car to be spawn in rviz visualization tool.

## Folder Structure

```
models
└─── README.md
│    vehicle_model
│    └─── launch (This folder is used to launch each supported car to the rviz simulator)
│        │ └─── sd_env200.launch
│        │     │sd_twizy.launch
│        │ mesh 
│        │ └─── sd_env200.stl.zip
│        │     │sd_twizy.stl.zip
│        │ urdf 
│          └─── sd_end200.urdf
│              │sd_twizy.urdf
│
rviz (This folder contains the default rviz config file)
└─── README.md
│
vehicle_info (This folder contains our supported vehicles parameters info)
└─── yaml_files
│    └───│sd_env200.yaml ("This file contains vehicle params for env200 e.g. maximum_steering_angle, minimum_turning_radius, wheel_base")
│        │sd_twizy.yaml ("This file contains vehicle params for twizy e.g. maximum_steering_angle, minimum_turning_radius, wheel_base")
│
README.md
``` 

A script, vehicle_model/unzip.sh , is provided to unzip the .stl files automatically

## Prerequisites

- RViz
- From the vehicle_model/ directory, run `./unzip.sh sd_twizy` or `./unzip.sh sd_env200`

## Installing

To install this package individually:

```
catkin build vehicle_model
# OR
catkin_make --only-pkg-with-deps vehicle_model #if you have previously built your workspace with catkin_make
```

## Running in Terminal

If you want to launch the env200 car to RVIZ simulator unzip the sd_env200.stl.zip file in the models/vehicle_model/mesh folder, then launch the simulator with car spawned inside using the following command:

```
roslaunch vehicle_model sd_env200.launch
```

If you want to launch the twizy car to RVIZ simulator unzip the sd_twizy.stl.zip file in the models/vehicle_model/mesh folder, then launch the simulator with car spawned inside using the following command:

```
roslaunch vehicle_model sd_twizy.launch
```
