# dagana_gazebo

This package provides a dagana gazebo plugin. 
Command and state of the gripper are available on the same topic as real robot: `xbotcore/gripper/dagana_2/command` and `xbotcore/gripper/dagana_2/state`

But in simulation is not so compliant with xbot2, so we need some hack. And in rviz it does not move

### Istructions:
- put plugin in urdf:
  ```xml
  <gazebo>
  <plugin filename="libdagana_gazebo_DaganaPlugin.so" name="dagana_plugin">
    <gripperName>dagana_2</gripperName>
    <jointName>dagana_2_claw_joint</jointName>
    <rate>100</rate>
  </plugin>
  </gazebo>
  ```
  
- Ignore joint in the simulation plugin:
  ```xml
  <plugin filename="libxbot2_gz_joint_server.so" name="xbot2_joint_driver">
    <ignore_joints>
      <joint name="dagana_2_claw_joint" />
    </ignore_joints> 
    <pid>
    [...]
  ```

- Not sure if necessary, but I put on the cartesio launch file:
```xml
  <param name="cartesian/joint_blacklist" type="yaml" 
    value="[dagana_2_claw_joint]"/>
```
