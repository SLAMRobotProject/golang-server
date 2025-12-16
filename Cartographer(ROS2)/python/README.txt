#### GET STARTED #####

# Install ROS2 (KILTED) and WSL2 (Windows)
See -> https://docs.ros.org/en/kilted/Tutorials/Advanced/Simulators/Webots/Installation-Windows.html

# Install python dep
pip install tf-transformations websockets PyYAML typing-extensions jinja2 setuptools typeguard

# Create viritual env for python (if trouble with installing py packets)
python3 -m venv venv

# Activate env
source venv/bin/activate

# Deactivate env
deactivate


##### HOW TO RUN #####

# Open wsl to run
Open terminal, run "wsl" 

# Navigate
cd "PATH to server" /golang-server/Cartographer(ROS2)/python

# Start bridge to Go-server
python3 goBridge.py

# Launch ROS2 backend
ros2 launch cartographer.launch.py

# Launch RVIz2 
ros2 run rviz2 rviz2

source /opt/ros/kilted/setup.bash
