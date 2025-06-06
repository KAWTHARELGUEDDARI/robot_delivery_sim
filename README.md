# Robot Delivery Simulation (robot_delivery_sim)

This repository contains a ROS 2-based simulation of a robot performing delivery tasks in a 10x10 grid with obstacles. The robot uses the A* pathfinding algorithm to navigate from a starting position (0,0) to a goal (5,5), avoiding obstacles. The project includes both text-based and visual outputs, with the final visualization implemented using RViz for an interactive 3D experience.

## Project Overview

- **Language**: Python
- **Framework**: ROS 2 Humble
- **Visualization Tools**: Matplotlib (for 2D images), RViz (for 3D simulation)
- **Environment**: Developed in GitHub Codespaces, tested on Ubuntu 22.04
- **Algorithm**: A* for optimal path planning

The simulation starts with a text-based output in the terminal, progresses to generating step-by-step images with Matplotlib, and culminates in a real-time 3D visualization with RViz, showcasing the robot's path, obstacles, and position.

## Prerequisites

- **Operating System**: Ubuntu 22.04
- **ROS 2 Humble**:
  - Install with:
    ```bash
    sudo apt update
    sudo apt install -y curl gnupg lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt install -y ros-humble-desktop
    source /opt/ros/humble/setup.bash
    ```
- **Python**: Pre-installed with Ubuntu 22.04
- **Matplotlib**: Install with:
  ```bash
  pip install matplotlib
  ```
- **Git**: For cloning the repository
  ```bash
  sudo apt install -y git
  ```

## Installation

1. **Clone the Repository**:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/KAWTHARELGUEDDARI/robot_delivery_sim.git
   ```

2. **Build the Workspace**:
   ```bash
   cd ~/ros2_ws
   colcon build
   ```

3. **Source the Environment**:
   ```bash
   source install/setup.bash
   ```
   Add to your `~/.bashrc` for persistence:
   ```bash
   echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

## Usage

### Running the Simulation

1. **Open Two Terminals**:
   - Terminal 1: For RViz
   - Terminal 2: For the robot controller node

2. **Launch RViz** (Terminal 1):
   ```bash
   ros2 run rviz2 rviz2
   ```
   - In RViz, add a `MarkerArray` display:
     - Click **Add** > Select **MarkerArray** > OK.
     - Set the **Topic** to `/visualization_marker_array`.
   - (Optional) Add a `Grid` display:
     - Click **Add** > Select **Grid** > OK.
     - Set **Cell Size** to `1.0` and **Plane Cell Count** to `10`.
   - Set the **Fixed Frame** to `map` in the top toolbar.

3. **Run the Robot Controller** (Terminal 2):
   ```bash
   ros2 run robot_delivery_sim robot_controller
   ```
   - Observe the logs (e.g., `Robot moved to (x, y)`) and the RViz visualization.

### Expected Output
- **Obstacles**: Gray cubes at (2,2), (3,2), and (4,2).
- **Path**: A blue line from (0,0) to (5,5).
- **Robot**: A red sphere moving along the path.

## Development Strategy

The project was developed in three phases for a structured approach:
1. **Textual Visualization in Terminal**: Initial validation of the robot's movement through logs.
2. **Matplotlib Images**: Generation of 2D images (`robot_path_step_X.png`) to visualize each step.
3. **RViz Simulation**: Final interactive 3D visualization, overcoming the lack of GUI in Codespaces by transferring to a local desktop.

## Creative Idea and Optimization
The project integrates concepts from optimization and search algorithms, using the A* algorithm to find the shortest path while avoiding obstacles. A* combines the cost-to-come and a heuristic (e.g., Manhattan distance) to ensure efficiency, making it ideal for robotic navigation and adaptable to future enhancements like dynamic obstacles.

## Troubleshooting
- **RViz Not Opening**: Ensure ROS 2 is sourced and reinstall if needed:
  ```bash
  sudo apt install --reinstall ros-humble-rviz2
  ```
- **No Markers in RViz**: Verify the topic with `ros2 topic list` and ensure the `Fixed Frame` is `map`.
- **Build Errors**: Check `package.xml` for redundant dependencies and re-run `colcon build`.

- **Author**: Kawthar El Gueddari

