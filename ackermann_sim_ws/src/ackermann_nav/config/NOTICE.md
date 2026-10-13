The behavior tree forward_only.xml is adapted from ROS Navigation / Nav2's installed
navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml,
Nav2 1.3.13 (Jazzy), Apache-2.0.
https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_bt_navigator/behavior_trees
The full Apache-2.0 text is in config/LICENSE within this package.
Local adaptations: remove Spin and BackUp, change Wait duration to 2 seconds.
Nav2 planning and control binaries are reused without algorithm modifications.
