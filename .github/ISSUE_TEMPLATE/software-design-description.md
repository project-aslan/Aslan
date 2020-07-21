---
name: Software Design Description
about: A detailed description of the software to be implemented
title: "[SDD]"
labels: enhancement
assignees: ''

---

[comment]: # (This template works as a design document for new features implementation. The purpose of this document is to:)
[comment]: # (1. Facilitate the developers working on new features)
[comment]: # (2. Request and provide feedback on software design)
[comment]: # (3. Manage and maintain the overall architecture of the stack)
[comment]: # (4. Document the software design of new features)

### Link to feature request
Add the number and title of the feature request this SDD links to

[comment]: # (eg. Links to FR #3 Object Classification)

### Feature description
A clear and concise description of what you want to implement

[comment]: # (eg. Implement YOLOv3: Real-Time Object Detection for classification of vehicles, pedestrians and bicycles)

### Feature details/functionalities
Provide a list of the main functionalities of your feature

### Dependencies
Provide a list of all the packages and libraries your software depends on
1. Sensor: 

[comment]: # (eg. camera, 30fps, res 1280 x 720)

2. Libraries: 

[comment]: # (eg.cuda 10.2, python3, numpy, opencv python bindings)

3. ROS packages: 

[comment]: # (eg. object detection in astar planner, aslan_gui)

4. OS and ROS Version:

### Overall Architecture Dependencies
Are there any changes that need to be made to other packages? Please specify
- Source Code:
- Docker:
- GUI:

### Architecture design
How does this package fit into the overall system?
#### 1. Data flow:
Input topics:
ROS topic | message type | message description | publisher
------------ | ------------- | ------------- | ------------- 
topic name 1 | message type | custom ? | publisher 
topic name 2 | message type | custom ? | publisher

Output topics:
ROS topic | message type | message description | subscriber(s)
------------ | ------------- | ------------- | ------------- 
topic name 1 | message type | custom ? | subscriber(s) 
topic name 2 | message type | custom ? | subscriber(s) 

#### 2. Launch parameters

args | default value | range | description
------------ | ------------- | ------------ | -------------
arg 1 | default value | values range | description
arg 2 | default value | values range | description

#### 3. Configurable parameters
These can be changed on runtime and link to configuration messages
ROS topic: `/config/<name_of_topic>`

param | default value | range | description
------------ | ------------- | ------------- | ------------- 
param 1 | default value | values range | description
param 2 | default value | values range | description

#### 4. Provide a basic data flow diagram

```mermaid
A[twist_filter]-- /twist_cmd -->B[sd_vehicle_interface]
B-- /sent_messages -->C[socketcan_bridge_node]
```
[comment]: # (Attach your graph to this issue, no need to use markdown)

### Additional context
Add any other context about this new feature here

[comment]: # (In the case of urgent feedback needed, assign this issue to yourself and one of the core developers:)
[comment]: # (EfimiaPanagiotaki-StreetDrone)
[comment]: # (AbdelrahmanBarghouth-StreetDrone)
[comment]: # (FionanOSullivan-StreetDrone)
