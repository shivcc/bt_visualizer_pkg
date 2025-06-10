# Behavior Tree Visualizer for ROS 2
A simple and intuitive graphical tool to visualize BehaviorTree XML files, designed for ROS 2 (Jazzy Jellyfish). This tool helps developers and roboticists quickly understand, debug, and present robot behavior patterns by rendering them as a clear and interactive tree diagram.


![demo](https://github.com/shivcc/bt_visualizer_pkg/blob/9b03d3dbff00c8b9eef7c6653d03d6efd20a0b5f/resource/bt_demo.gif)

## Features
Load from XML: Directly load and parse BehaviorTree.CPP v4 format .xml files.

Multiple Themes: Choose between Dark, Light, and Forest themes to suit your preference.

## Interactive Canvas:

Pan: Hold and drag the Middle Mouse Button or Left Mouse Button to move around the tree.

Zoom: Hold Ctrl (or Cmd on macOS) and use the Mouse Scroll Wheel to zoom in and out. The zoom is centered on the cursor for easy navigation.

High-Quality Image Export: Save the current tree view as a high-resolution (300 DPI) .png image, perfect for documentation and presentations.

Intelligent Layout: An automatic layout algorithm prevents node overlaps, even in complex trees.

Color-Coded Nodes: Nodes are colored by their type (Control, Action, Condition, etc.) for at-a-glance identification.

## Installation
### 1. Dependencies
This tool is designed for ROS 2 Jazzy Jellyfish. Before building, ensure you have the following system dependencies installed:

Pillow: The Python Imaging Library, used for image export.

Ghostscript: Required by Pillow to process PostScript data for high-resolution image saving.

### Update package lists
`sudo apt-get update`

### Install Pillow and Ghostscript
`sudo apt-get install python3-pil ghostscript`

### 2. Build from Source
Clone this repository into your ROS 2 workspace, build, and source it.

### Navigate to your ROS 2 workspace source directory
`cd ~/ros2_ws/src/`

### (If you haven't already) Clone the repository from GitHub
`git clone https://github.com/shivcc/bt_visualizer_pkg.git`

### Navigate to the workspace root and build the package
`cd ~/ros2_ws/`
`colcon build --packages-select bt_visualizer_pkg`

### Source the workspace every time you open a new terminal
`source install/setup.bash`

Usage
To run the visualizer node, execute the following command in a terminal that has the workspace sourced:

`ros2 run bt_visualizer_pkg visualizer`

The application window will appear. Use the "Load XML" button to open your Behavior Tree file.

Controls Summary
Pan Canvas: Hold and Drag Left Mouse Button OR Middle Mouse Button.

Zoom Canvas: Ctrl + Mouse Scroll Wheel.

Theme Selection: Use the dropdown menu at the top.

License
This project is licensed under the Apache 2.0 License - see the LICENSE file for details.
