# 🍓 Open-loop Robotic Arm Strawberry Harvesting Simulation

This repository provides a simulation framework for **open-loop robotic arm strawberry harvesting**.  
It includes URDF modeling, motion planning with **MoveIt**, and visualization in **RViz** under ROS.  
Multiple simulation scenarios are provided to evaluate robotic arm performance in different harvesting conditions.

---

## ✨ Features
- 🔧 **URDF Modeling**: Define the kinematic structure and geometry of the open-loop robotic arm.  
- 🤖 **MoveIt Integration**: Perform motion planning for strawberry picking tasks.  
- 🖥️ **RViz Visualization**: Display real-time simulation results and harvesting paths.  
- 🌱 **Scenario Simulation**: Different harvesting environments to test and validate path planning strategies.  

---

## 📦 Dependencies
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) (Ubuntu 20.04 recommended)  
- [MoveIt](https://moveit.ros.org/install/)  
- RViz  
- Python 3 / C++ (for custom scripts)  

---

## 🚀 Installation
1. Clone the repository into your ROS workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/yourusername/strawberry_arm_simulation.git
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
Verify installation:

roslaunch arm12_moveconfig demo.launch
📂 Project Structure
graphql

├── arm12/                     # Robotic arm URDF description
├── arm_12_py/                 # Python scripts for control & path planning
├── arm12_moveconfig/          # MoveIt configuration for arm12
├── arm12_moveconfig_2/        # Alternative MoveIt configuration
├── double_arm_moveit_config/  # Dual-arm MoveIt configuration
├── moveit_*                   # MoveIt packages (msgs, resources, tools, tutorials, etc.)
├── panda_moveit_config/       # Reference MoveIt configuration
├── piper_*                    # Robot description & config
├── rviz_visual_tools/         # RViz visualization helpers
├── scout_description/         # Mobile base / environment description
├── straw/                     # Strawberry environment & scenario models
├── CMakeLists.txt             # Build configuration
└── .rosinstall                # ROS workspace configuration
🛠️ Usage
Launch MoveIt demo:


roslaunch arm12_moveconfig demo.launch
Run harvesting path simulation:


roslaunch straw harvesting_path.launch
Visualize in RViz:

Start RViz with pre-configured visualization:

roslaunch rviz_visual_tools demo.launch
Observe strawberry harvesting trajectories.

🌍 Simulation Scenarios
Single strawberry picking

Multiple strawberries in sequence

Occluded strawberries

Dual-arm harvesting (experimental)

📸 Example Results
Simulation results are visualized in RViz, including strawberry positions and the robotic arm's harvesting trajectories.

<p align="center"> <img src="docs/images/rviz_demo.png" width="600"> </p>
📜 License
This project is released under the MIT License.

🙌 Acknowledgements
ROS & MoveIt community

RViz visualization tools
