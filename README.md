cd ~/tiago_jazzy_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch tiago_gazebo tiago_gazebo.launch.py moveit:=True is_public_sim:=True world_name:=pick
