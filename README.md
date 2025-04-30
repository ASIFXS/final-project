# ROSA Agent for TurtleBot3 Navigation (ROS 1 Noetic)

This repository integrates the ROSA agent framework with a simulated TurtleBot3 robot using ROS 1 Noetic and the standard ROS Navigation stack (`move_base`).

## Overview

The setup uses two Catkin workspaces to manage separate Python environments:

- `turtlebot3_ws`: For TurtleBot3 simulation and navigation (Python 3.8).
- `zrosa_ws`: For ROSA agent code using a Python 3.9 virtual environment.

The ROSA agent accepts natural language commands and sends goals to the `/move_base` action server to control TurtleBot3.

## Project Structure

```
final-project/
├── turtlebot3_ws/               # Workspace for TurtleBot3 simulation/navigation
│   └── src/
│       ├── turtlebot3/
│       ├── turtlebot3_msgs/
│       └── turtlebot3_simulations/
├── zrosa_ws/                    # Workspace for ROSA Agent (Python 3.9 venv)
│   ├── venv_rosa/               # Python 3.9 virtual environment
│   └── src/
│       └── llm-ur-control/
│           └── ur_agent/       # ROSA agent package
│               ├── scripts/
│               │   ├── ur_agent.py
│               │   ├── locations.py
│               │   ├── llm.py
│               │   ├── prompts.py
│               │   └── help.py
│               ├── package.xml
│               └── CMakeLists.txt
└── README.md                   # This file
```

## Prerequisites

1. **Operating System**: Ubuntu 20.04 LTS
2. **ROS Version**: ROS 1 Noetic
3. **Python**:
   - System: Python 3.8
   - Additional: Python 3.9 (`python3.9`, `python3.9-dev`, `python3.9-venv`)
4. **Dependencies**:
   ```bash
   sudo apt install git python3.9 python3.9-dev python3.9-venv \
       ros-noetic-joy ros-noetic-teleop-twist-joy \
       ros-noetic-teleop-twist-keyboard ros-noetic-slam-gmapping \
       ros-noetic-navigation ros-noetic-amcl \
       ros-noetic-map-server ros-noetic-move-base \
       ros-noetic-turtlebot3-simulations
   ```
5. **Map Files**: Create via:
   ```bash
   rosrun map_server map_saver -f ~/map
   ```
6. **OpenAI API Key**: Required for LLM interaction.

## Setup Instructions

### Step 1: Clone the Repository
```bash
git clone https://github.com/ASIFXS/final-project.git
cd final-project
```

### Step 2: Clean `~/.bashrc`
- Ensure **only** this line is sourced:
  ```bash
  source /opt/ros/noetic/setup.bash
  ```
- Comment or remove any lines that source workspace-specific `setup.bash` files.

### Step 3: Install Python 3.9
```bash
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update
sudo apt install python3.9 python3.9-dev python3.9-venv
```

### Step 4: Build `turtlebot3_ws`
```bash
cd ~/final-project/turtlebot3_ws
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

### Step 5: Setup `zrosa_ws`
```bash
cd ~/final-project/zrosa_ws
python3.9 -m venv venv_rosa
source venv_rosa/bin/activate
pip install --upgrade pip
pip install python-dotenv langchain openai rich pyinputplus numpy
source /opt/ros/noetic/setup.bash
catkin_make
```

### Step 6: Configure Locations
Edit `locations.py`:
```bash
gedit ~/final-project/zrosa_ws/src/llm-ur-control/ur_agent/scripts/locations.py
```
Modify the `LOCATIONS` dictionary with appropriate (x, y, yaw) values.

### Step 7: Add OpenAI API Key
```bash
cd ~/final-project/zrosa_ws
gedit .env
```
Add:
```
OPENAI_API_KEY='sk-YourActualKeyHere'
```

## Running Instructions

### Terminal Setup
Open **3 terminals**. In **each**, export the model:
```bash
export TURTLEBOT3_MODEL=burger
```

### Terminal 1: Launch Gazebo
```bash
cd ~/final-project/turtlebot3_ws
source devel/setup.bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

### Terminal 2: Launch Navigation
```bash
cd ~/final-project/turtlebot3_ws
source devel/setup.bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```
Use the 2D Pose Estimate tool in RViz to localize the robot.

### Terminal 3: Launch ROSA Agent
```bash
cd ~/final-project/zrosa_ws
source venv_rosa/bin/activate
source devel/setup.bash
rosrun ur_agent ur_agent.py
```
Interact with commands like:
```text
> go to L1
> navigate to CENTER
```

## Customization

- **Location Points**: Modify `locations.py`
- **API Key**: Ensure `.env` file is correctly set

## Troubleshooting

- **`catkin_make` errors**: Ensure correct dependencies in `package.xml` and `CMakeLists.txt`. Source `/opt/ros/noetic/setup.bash` first.
- **`ModuleNotFoundError`**: Activate venv and install missing packages.
- **`move_base` not found**: Ensure Navigation stack is running.
- **Poor localization**: Re-estimate pose in RViz.
- **Navigation fails**: Check map for obstacles or localization accuracy.

---

Enjoy controlling your TurtleBot3 using natural language with ROSA!

