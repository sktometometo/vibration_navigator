# vibration_navigator_driver

## ROS Nodes

### vibration_navigator_driver_node

#### Subscribing Topics

- "~input" ( geometry_msgs/PoseStamped )

The Next Footstep which the vibration_navigator is goint to lead one's foot to reach

#### Publishing Topics

- "~output" ( std_msgs/UInt16MultiArray )

#### Parameters

- "~max_duration" ( double, default: 5.0 )

- "~num_spinthread" ( int, default: 4 )

- "vibrator_config" 

an example is below

```
- frame_id: "right_toe"
  position:
    x: -0.1
    y: 0.0
    z: 0.1
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
- frame_id: "right_toe"
  position:
    x: -0.3
    y: 0.0
    z: 0.1
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
```

-
