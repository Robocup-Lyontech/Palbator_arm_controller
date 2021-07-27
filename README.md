# Palbator_arm_controller

Connect to dynamixel and start controllers
```bash
roslaunch dynamixel_arm_controller joint_trajectory_controller.launch
```

Connect moveit
```bash
roslaunch pmb2_arm_moveit_real moveit_planning.launch
```

You can now use rviz to command the arm!

---

To configure properly the motors in the [config file](./config/pmb2_arm.yaml), you can use the [excel file](./doc/Dynamixel_param.ods)