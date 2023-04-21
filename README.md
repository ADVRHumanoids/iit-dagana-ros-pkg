# iit-dagana-ros-pkg
Model for IIT's Dagana gripper

### Usage
Add the gripper to the robot with the provided macro:
```xml
<xacro:include filename="$(find dagana_urdf)/urdf/dagana_macro.urdf.xacro" />

<xacro:dagana_gripper name="dagana_2" parent_link="arm2_6" joint_type="fixed">
  <origin xyz="0. 0.05 -0.08" rpy="0. ${PI} ${PI}"/>
</xacro:dagana_gripper>
```

### Gazebo Usage
check https://github.com/ADVRHumanoids/iit-dagana-ros-pkg/tree/gazebo-plugin/dagana_gazebo
