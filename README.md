## Installation
```bash
# Create workspace
mkdir -p ~/articubot_two_ws/src
cd ~/articubot_two_ws/src
# Clone repo
git clone https://github.com/beyondabhay21/articubot_two.git articubot_two_description
# Build
cd ~/articubot_two_ws
colcon build --symlink-install
source install/setup.bash
```
Add to ~/.bashrc:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/articubot_two_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
## Usage
### Visualize in RViz
```bash
ros2 launch articubot_two_description display.launch.py
```
### Launch in Gazebo
```bash
ros2 launch articubot_two_description gazebo.launch.py
```
### Teleoperation
Open a new terminal after Gazebo is running:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```




# SLAM - 
cd ~/articubot_two_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch articubot_two_description slam.launch.py

# TELOP
source ~/articubot_two_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/diff_drive_controller/cmd_vel_unstamped