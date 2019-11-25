# evo_rd_platform_example
An open example implementation for evocortex R&amp;D platform.

This includes example nodes for:

- Robot Base Interface (base_controller_node)
- Time of Flight - Sensor Interface (tof_controller_node)
- DC-DC Shield Interface

## Getting Started

### Installation

#### TODO:

1. Install debian package
2. Install ros ws tool 
3. CAN init & udev






## Nodes

### Attach CAN module and start the example node

```bash
roscd evo_rd_platform_example
cd scripts
./can_init.sh
roslaunch evo_rd_platform_example evo_xxx.launch
```

### Launch parameters
####  Common
- **NodeName**: The name the node has while running. Also needed for correct parameter loading.
- **loop_rate_hz**: The frequency the node should be running with. 
- **topic_xxxx**: Topic names as parameters instead of remapping

#### CAN interface
- **can_interface_name**: The name given to the attached can interface in the **can_init** script



## base_controller_node

This node is responsible for the communication with the motorshields. 

Using the mecanum kinematics, the RPM for each wheel is calculated and sent to the corresponding motor.

Odometry is calculated using encoder data from motors.


### Published Topics

- /odom    (nav_msgs/Odometry)
- /tf            (geometry_msgs/TransformStamped)

### Subscribed Topics

- /cmd_vel     (geometry_msgs/Twist)
- /cmd_lift     (std_msgs/Int8)




### Launch Parameters

#### Odometry TF

- **enable_odom_tf**: If true, the odometry will also be published as a TF from *odom_frame_id* to *odom_child_frame_id*
- **odom_frame_id**: Parent frame id for the odometry TF
- **odom_child_frame_id**: Child frame id for the odometry TF


#### Timeouts
- **com_timeout_s**: Duration in seconds after which the communication between PC and MC is assumed to be asynchronous which will result in a resync process.

- **cmd_vel_timeout_s**: Duration in seconds after which the last received message on the *cmd_vel* topic is classified as invalid/outdated. If no new message is received prior to this event, the robot will be stopped.

#### Base parameters
These parameters are essential for a clean performance as the accuracy of the mathematical model for the mecanum drive relies on them. Make sure to set them properly.
- **wheel_radius_in_m**: Distance between center and tread of wheels in m
- **wheel_distance_front_back_in_m**: Distance between the centers of the front and back wheels in m
- **wheel_distance_left_right_in_m**: Distance between the centers of the right and left wheel in m

#### Covariances
The covariances are not determined by measurements. If you face problems with localization, you can try to adjust these parameters according to your conditions.
- **covariance_pos_x**
- **covariance_pos_y**
- **covariance_pos_yaw**
- **covariance_vel_x**
- **covariance_vel_y**
- **covariance_vel_yaw**

#### Motors & Motorcontrollers
You can connect up to 255 Motorshields to the CAN bus. They will only be initialized if they are specified via parameters. It is only possible to initialize a *Motor* if the corresponding *Motorshield* is also being initialized. 

The parameter "syntax" to connect a MC is the same for every additional MC added. It generally consists of a outer *Motorshield* block, which has to be named **msX**, where **X** stands for the ID which starts ascending from **1**, and inner *Motor* blocks, which have to be named **motorY**. **Y** stands for the motor ID and starts ascending from **0**. (see the example launchfile)

##### Motorshield

- **enable**: When connecting motorshields with not seamless ascending IDs, you can skip IDs between by setting this parameter to FALSE
- **timeout_ms**: Duration in ms after which a motorshields will deactivate itself if there is no communication in the meantime.

##### Motor

- **type**: The operation type of the motor ( 0 = NONE , 1 = LIFT , 2 = DRIVE)
- **ctrl_mode**: The control mode of the motor ( 0 = NONE , 1 = PWM , 2 = RPM , 3=POS )
- **kp**: Proportional gain of the controller
- **ki**: Integral gain of the controller
- **kd**: Derivative gain of the controller
- **pwm_limit**: Maximum PWM value that is accepted by the MC (only in PWM mode)
- **gear_ratio**: Multiplier for the motor speed to achieve wheel speeds in drive mode
- **encoder_res**: Encoder resolution
- **motor_mapping**: ID to assign the motor logically to a position on the robot (e.g. mecanum drive) ( 0 = NO_POSITION , 1 = FRONT_LEFT , 2 = FRONT_RIGHT, 3 = BACK_RIGHT, 4 = BACK_LEFT)

### Usage

Send *geometry_msgs/Twist* to **/cmd_vel** to control the robot in X-/Y-/Yaw-direction.

Send std_msgs/Int8 to **/cmd_lift** to control the movement direction for the lift. (-1 = downwards, 0 = no move, 1 = upwards)







## tof_controller_node

This node is responsible for the communication with the sensorboards. 

### Published Topics

- /evo_tof/range            (sensor_msgs/Range)
- /evo_tof/detailed        (evo_rd_platform_example/evo_ToF_detailed)



###  Launch Parameters

#### Sensor

- **ToF_FoV_rad**: The field of view of the sensor.
- **ToF_range_max_m**: max range

#### Sensorboards

- **n_ToF_sensorboards**: number of boards that should be initialized and used.

Parameters for each sensorboard are specified using a simple syntax containing "sensorboard" + **X**, where **X** starts at 1 and counts up to **n_ToF_sensorboards**.

- **id**: CAN ID for the sensorboard
- **frame_id**: frame id for the sensorboard (TF)

#### TF

Each sensorboard carries two ToF sensors which give seperate measurements.
This requires unique frames for each of them. The frame_ids for the two sensors are:
- **frame_id** + _left
- **frame_id** + _right

  

  













