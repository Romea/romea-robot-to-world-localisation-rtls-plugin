# romea_robot_to_world_localisation_rtls_plugin #

This package is a RTLS plugin for robot localisation. It schedules ranging between the robot's rtls transceivers (called initiators) and the infrastructure's rtls transceivers (called responders) within communication range. It provides these measurements to the localization filter as well as a rough estimation of robot pose in using trilateration algorithm base on ranging results.  

# ROS2 plugin node description #

### 1) Subscribed Topics ###

- **filtered_odom** (nav_msg::msg::Odometry) :

    This topic is provided by robot localisation filter. It contains estimated position of the robot, which will be used to select the insfrastructure's rtls transceiver that are within communication range. 

- For each initiator declared in parameters :

  - **initiators_names[i]/range**: (romea_rtls_transceiver_msgs::msg::RangingResult

    This topic is provided by rtls transceiver driver node called initiators_names[i]/driver, it contains ranging result beetween selected initiator and the leader responder

### 2.2 Published Topics ###

- **pose** (romea_localisation_msgs::msg::ObservationPose2DStamped)

  Leader pose estimated by trilateration algorithm using ranging results

- **range** (romea_localisation_msgs::msg::ObservationRangeStamped)

  Ranging results between robot's rtls tranceivers (called initiators) and infrastructure's rtls transceivers (called responders)

- For each initiator declared in parameters :

  - **initiators_names[i]/request** (romea_rtls_transceiver_msgs::msg::RangingRequest)

      This topic is sended to initiator driver node in order to start ranging with selected responder, if ranging succeeded then range result will be send to the plugin node 

### 2.3 Parameters ###

- **enable_sheduler** (bool, default: true)

    Enable ranging scheduling. The scheduler must be actived in live and simulated experiments and disables during replay.

- **~initiators_ids** (vector of int)

    Identifier of each rtls transceivers (called initiators) embedded on the robot

- **~initiators_names** (vector of string)

    Name of each rtls transceivers (called initiators) embedded on the robot, more precisely these names are the ros namespace of their driver nodes

- For each initiator declared previously :

  - **~initiators_positions.initiators_name[i]** (vector of double)

    Position of the antenna of the rtls transceiver embedded on the robot 

- **~responders_ids** (vector of int)

    Identifier of each rtls transceivers (called responders) located in the infrastructure

- **~responders_names** (vector of string)

    Name of the rtls transceivers (called responders) located in the infrastructure

- For each responder declared previously :

  - **~responders_positions.responders_names[j]** (vector of double)

    Position of the antenna of the rtls transceiver located in the infrastructure

- **~minimal_range** (double, default: 0.5)

    Minimal available range distance   

- **~maximal_range** (double, default: 20.)

    Maximal available range distance

- **~range_std** (double, default: 0.02)

    Standard deviation on range measurement

- **~poll_rate** (int, default: 20)

    Rate at which ranging requests are sent to initiators 

- **~map_frame_id** (string, default:map)

    Name of map reference frame 

- **~base_footprint_frame_id** (string, default: base_footprint)

    Name of robot base footprint

## **Usage**

  See romea_ros2_localisation_bringup project 

## **Contributing**

If you'd like to contribute to this project, here are some guidelines:

1. Fork the repository.
2. Create a new branch for your changes.
3. Make your changes.
4. Write tests to cover your changes.
5. Run the tests to ensure they pass.
6. Commit your changes.
7. Push your changes to yo

## License

This project is released under the Apache License 2.0. See the LICENSE file for details.

### Authors

 romea_ros2_robot_to_world_localisation_rtls_plugin project was developed by **Jean Laneurit** in the context of BaudetRob2 ANR project.

### Contact

If you have any questions or comments about romea_ros2_robot_to_world_localisation_rtls_plugin project, please contact **[Jean Laneurit](mailto:jean.laneurit@inrae.fr)** 
