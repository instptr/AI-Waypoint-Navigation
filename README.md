# AI Road and Waypoint Navigator

## Overview
This project utilizes artificial intelligence to detect roads and waypoints on a mini-map in the game "Grand Theft Auto V". It automatically navigates your character towards designated waypoints, simulating real-time pathfinding and navigation capabilities.

## Features
- **Custom Trained Model**: Uses a custom dataset and yolov8 model to detect neccesary objects in real-time.
- **Road Detection**: Uses computer vision to identify roads on the mini-map.
- **Waypoint Navigation**: Detects waypoints and calculates the optimal path for navigation.
- **Dynamic Path Adjustment**: Continuously adjusts the path based on real-time mini-map changes.
- **Autonomous Control**: Automatically controls the game character to follow the detected path.

## Technologies Used
- Python 3.8
- OpenCV for image processing
- MSS for screen capture
- PyGetWindow for window management
- YOLOv8 for object detection
- Pynput for simulating keyboard input

## Setup and Installation
1. **Clone the repository:**
   ```bash
   git clone https://github.com/instptr/AI-Waypoint-Navigation
   ```
2. **Install dependencies:**
   ```bash
   pip install numpy opencv-python mss pygetwindow pynput
   ```
   Note: Ensure you have the appropriate YOLO model files and update the `PATH` variable in the script.

3. **Run the script:**
   ```bash
   python main.py
   ```

## Usage
Ensure "Grand Theft Auto V" is running and visible on the screen. Run the script, and it will start detecting the roads and waypoints on the mini-map, navigating the character accordingly.

## License
Distributed under the MIT License. See `LICENSE` for more information.

## Acknowledgements
- This project is for educational purposes and is not affiliated with "Grand Theft Auto V" or its creators.
