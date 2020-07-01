cmake_minimum_required(VERSION 2.8.3)
project(aslan_msgs)

find_package(catkin REQUIRED COMPONENTS
  		message_generation
        std_msgs
        geometry_msgs
        sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
        DIRECTORY msg
        FILES
            NDTStat.msg
            Lane.msg
            Waypoint.msg
            LaneArray.msg
            DTLane.msg
            WaypointState.msg
            State.msg
	    	SDControl.msg
            ControlCommandStamped.msg
            ControlCommand.msg
            RemoteCmd.msg
            VehicleCmd.msg
            AccelCmd.msg
            BrakeCmd.msg
            SteerCmd.msg
            LampCmd.msg
            ConfigLaneStop.msg
            ConfigNDT.msg
            ConfigNDTMapping.msg
            ConfigNDTMappingOutput.msg
            ConfigLaneRule.msg
            ConfigWaypointFollower.msg
            ConfigTwistFilter.msg
            ConfigWaypointLoader.msg
            ConfigRayGroundFilter.msg
            ConfigVoxelGridFilter.msg
            ConfigLaneSelect.msg
            ConfigVelocitySet.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        std_msgs
        geometry_msgs
        sensor_msgs
)
