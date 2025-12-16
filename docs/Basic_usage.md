


```
roslaunch alicia_d_driver alicia_d_driver.launch
```



Read states:
```
rostopic echo /joint_states
```

Joint control:

```

rostopic pub /joint_commands sensor_msgs/JointState "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
name:     ['Joint1','Joint2','Joint3','Joint4','Joint5','Joint6','Gripper']
position: [0.5, 0.1, 0.0, 0.0, 0.1, 0.0, 500.0]
velocity: [0, 0, 0, 0, 0, 0, 0]
effort:   []" -r 5
```



Torque disable:
```
rostopic pub /demonstration std_msgs/Bool "data: true" -1
```

Torque anable:
```
rostopic pub /demonstration std_msgs/Bool "data: false" -1
```


Zero calibration:
Please be careful for this part, once calibrated, the IK solution may be affected (MoveIt accuracy may be affected).


```
rostopic pub /demonstration std_msgs/Bool "data: true" -1
```


```
rostopic pub /zero_calibrate std_msgs/Bool "data: true" -1
```



