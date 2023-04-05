# dagana_gazebo

This package provides a dagana gazebo plugin.  

A gazebo jointStateController in position is used after interpolating the command as done in xbot2 for the real robot  

The gazebo jointStateController permits to "stop" the closure when the contacts happen.  

Command and state of the gripper are available on the same topic as real robot: `xbotcore/gripper/dagana_2/command` and `xbotcore/gripper/dagana_2/state`  

**Note** Sometimes it seems that contacts are not extablished. Try to lift anyway visually there may be some problem with gazebo (enable the contact visualization in gazebo to verify

But in simulation is not so compliant with xbot2, so we need some hack. And in rviz it does not move

### Istructions:

- You need the `new_end_effectors` branch of centauro model (https://github.com/ADVRHumanoids/iit-centauro-ros-pkg/pull/37/commits/9d3ac95208e58ee8cfbe95cf31efe1808ddcf66b)

- When generating the centauro models, put `dagana` for the model used by gazebo, and `dagana_fixed` for all the others (Xbot, Cartesio).  
  There is the xacro arg for this  
  ```xml
  <xacro:arg name="end_effector_right" default="ball"/> <!-- none, ball, dagana_fixed, dagana -->
  ```  
  You can also set it from command line, check ROS docs

### Troubleshoots

- If problem with some objects, try playing with p i d and cmdMax/Min. Look for `libdagana_gazebo_DaganaPlugin` in the urdf where you put the `dagana` (not fixed) option. The one set are the default ones of gazebo jointposcontroller.
  
- If still problem in keeping the grasp, try [this plugin](https://github.com/JenniferBuehler/gazebo-pkgs/wiki/The-Gazebo-grasp-fix-plugin).  
  For dagana and centauro, configs are:
  ~~~xml
  <plugin name="gazebo_grasp_fix" filename="libgazebo_grasp_fix.so">
    <arm>
      <arm_name>centauro</arm_name>
      <palm_link>arm2_6</palm_link>
      <gripper_link> dagana_2_bottom_link </gripper_link>
    </arm>
    <forces_angle_tolerance>100</forces_angle_tolerance>
    <update_rate>4</update_rate>
    <grip_count_threshold>4</grip_count_threshold>
    <max_grip_count>8</max_grip_count>
    <release_tolerance>0.005</release_tolerance>
    <disable_collisions_on_attach>false</disable_collisions_on_attach>
    <contact_topic>__default_topic__</contact_topic>
  </plugin>
  ~~~

#### Other Notes  

Other trial is to add the implicit_spring_damper, and provide the feedback to read the force torque modifying the source code of gazebo

- in the urdf:
  ```xml
  <gazebo reference="dagana_2_claw_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
    <provideFeedback>true</provideFeedback>
  </gazebo>
  ```

- Directly in the sdf (gazebo world loaded)  
  ```xml
  <joint name='dagana_2_claw_joint' type='revolute'>
    [...]
    <physics>
      <provide_feedback>1</provide_feedback>
      <ode>
      <implicit_spring_damper>1</implicit_spring_damper>
      <cfm_damping>1</cfm_damping>
      <limit>
        <cfm>0</cfm>
        <erp>0.2</erp>
      </limit>
      <provide_feedback>1</provide_feedback>
      </ode>
    </physics>
  </joint>
  ```
