cd blimp_linux && colcon build --symlink-install && source install/setup.bash
cd ..
cd blimp_msgs && colcon build && source install/setup.bash
cd .. && ros2 launch blimp_linux teleop_launch.launch.py
