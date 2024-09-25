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
Usual XBot setting,  
Please note that using default ROS-Gazebo plugin (at https://github.com/ADVRHumanoids/iit-dagana-ros-pkg/tree/master/dagana_gazebo) is deprecated

# Dagana Paper
```bibtex
@inproceedings {Dagana2024,
  author = {Del Bianco, Edoardo and Torielli, Davide and Rollo, Federico and Gasperini, Damiano and Laurenzi, Arturo and Baccelliere, Lorenzo and Muratore, Luca and Roveri, Marco and Tsagarakis, Nikos G.},
  booktitle={2024 IEEE-RAS 23rd International Conference on Humanoid Robots (Humanoids)}, 
  title = {A High-Force Gripper with Embedded Multimodal Sensing for Powerful and Perception Driven Grasping},
  year={2024},
  volume={},
  number={},
  pages={},
  doi={}
}
```

