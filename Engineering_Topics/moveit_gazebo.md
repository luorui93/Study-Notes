# How to configure customized robot in Gazebo and control with Moveit!


Some helpful references:


[tutorial](https://medium.com/@tahsincankose/custom-manipulator-simulation-in-gazebo-and-motion-planning-with-moveit-c017eef1ea90)

[git_repo](https://github.com/jmichiels/arm)

## Gazebo configuration
transmission tag in URDF (transmission type, joint hardwareInterface)

```xml
<transmission name="trans_joint_a1">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="joint_a1">
            <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
        </joint>
        <actuator name="joint_a1_motor">
            <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
            <mechanicalReduction>1</mechanicalReduction>
        </actuator>
    </transmission>
```
joint limit(force, velocity)

```xml
    <joint name="joint_a7" type="revolute">
        <origin rpy="0 0 0" xyz="0 0 0" />
        <parent link="link_6" />
        <child link="link_7" />
        <axis xyz="0 0 1" />
        <limit effort="40" lower="-3.0541" upper="3.0541" velocity="2.356" />
    </joint>
```

gazebo_ros_control plugin (namespace, robotSimType)
```xml
    <gazebo>
      <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
        <robotNamespace>/kuka_lbr_iiwa_14_r820</robotNamespace>
        <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
        <legacyModeNS>true</legacyModeNS>
      </plugin>
    </gazebo>
```

## ROS control configuration
An example of controllers in specified namespace
Gazebo uses joint_state_controller to publish simulated robot's joint states to ROS.

```yaml
kuka_lbr_iiwa_14_r820:
  joint_state_controller:
     type: joint_state_controller/JointStateController
     publish_rate: 50
  arm_controller:
     type: position_controllers/JointTrajectoryController
     joints:
      - joint_a1
      - joint_a2
      - joint_a3
      - joint_a4
      - joint_a5
      - joint_a6
      - joint_a7
```

## Moveit! configuration
controller_list requires a special type of controller (FollowJointTrajectory) for moveit to interface with gazebo controller. arm_controller has to be a JointTrajectoryController(could be velocity,position or effor)

```yaml
controller_list:
  - name: /kuka_lbr_iiwa_14_r820/arm_controller
    action_ns: follow_joint_trajectory
    default: true
    type: FollowJointTrajectory
    joints:
      - joint_a1
      - joint_a2
      - joint_a3
      - joint_a4
      - joint_a5
      - joint_a6
      - joint_a7

```