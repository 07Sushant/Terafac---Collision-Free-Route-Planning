# Autonomous Robot Simulator with Vision-Based Navigation

## Overview

This project implements an autonomous self-driving robot simulator using a Python Flask backend with WebSocket communication and a browser-based 3D Three.js frontend. The robot autonomously navigates a dynamic environment with obstacles toward a configurable goal using computer vision for obstacle avoidance and goal detection.

***

## Project Mission

- Build a **fully autonomous** robot that drives itself toward a dynamically set goal near one of the environment’s corners.
- Use **computer vision** (camera image analysis) to detect obstacles and the goal — no hardcoded obstacle locations.
- Minimize robot collisions while navigating increasingly complex scenes.
- The robot continuously perceives, plans, and acts using image data, dynamically adjusting steering and movement.
- All control happens **without manual intervention after launch**.

***

## Features Delivered

### Backend (Flask + WebSocket)

- REST API endpoints to:
  - Set absolute or relative moves (`/move`, `/move_rel`)
  - Stop robot (`/stop`)
  - Capture robot camera image (`/capture`)
  - Set goal position (`/goal`)
  - Set and move obstacles (`/obstacles/positions`, `/obstacles/motion`)
  - Reset collision counts and simulator (`/reset`)
  - Start, stop, and check status of **autopilot** (`/autopilot/start`, `/autopilot/stop`, `/autopilot/status`)
  - Get collision count (`/collisions`)

- WebSocket server to broadcast commands, receive robot state, collisions, and camera images.

- Autopilot implemented as a thread running a loop that:
  - Requests camera images from the robot.
  - Performs computer vision for goal recognition and obstacle avoidance.
  - Commands robot movements accordingly.
  - Dynamically switches goals in a cycle if desired.
  - Stops on goal reach or upon API request.

### Frontend (Three.js)

- Real-time 3D rendering of environment including:
  - Robot with articulated limbs and head-mounted camera.
  - Obstacles (static or moving).
  - Visual goal marker.
- Camera image capture streamed back to server for vision processing.
- HUD with collision count and status messages.
- Control panel with buttons to:
  - Set goals
  - Start/stop/autopilot status
  - Adjust obstacle movements
  - Reset simulation
  - Run predefined challenge levels (with stationary and moving obstacles at different speeds)

***

## How It Works

1. **Set a goal** near one of the four corners or a custom location.
2. **Start autopilot** via the `/autopilot/start` API or using the UI button.
3. The autopilot loop:
   - Sends a request to capture the robot’s camera image.
   - Uses OpenCV to segment the image by color:
     - Identifies the cyan-colored goal flag.
     - Detects green obstacles in front.
   - Makes steering decisions:
     - Turns away from detected obstacles.
     - Steers toward the goal flag when visible.
     - Moves forward a standard step size when path is clear.
4. Commands robot moves via `/move` or `/move_rel` endpoints through WebSocket.
5. Continuously updates pose and sensors until the robot reaches the goal with minimal collisions.
6. Enables multi-level testing with static and moving obstacles of varying speeds.

***

## Requirements & Setup

- Python 3.x
- Flask
- websockets
- (Optional but recommended) OpenCV (`cv2`) for image processing
- numpy
- Browser with WebGL support for frontend display

***

## Running the Simulator

- Run the backend server (Flask + WebSocket).
- Open the frontend HTML page in a browser.
- Use UI buttons or API endpoints to set goals, start autopilot, and monitor status.

***

## Project Structure

- `app.py` — Flask + WebSocket server, autopilot logic, REST API endpoints.
- `index.html` — Frontend UI and Three.js 3D robot simulation.
- `static/` — sprites, assets (if any).
- `README.md` — This file.

***

## Usage Examples

- Start autopilot with moving obstacles enabled at speed 0.06 (via API):
  ```bash
  curl -X POST http://localhost:5000/autopilot/start -H "Content-Type: application/json" -d '{"moving_obstacles": true, "speed": 0.06}'
  ```
- Set goal to Northeast corner:
  ```bash
  curl -X POST http://localhost:5000/goal -H "Content-Type: application/json" -d '{"corner": "NE"}'
  ```
- Stop autopilot:
  ```bash
  curl -X POST http://localhost:5000/autopilot/stop
  ```
- Check collision count:
  ```bash
  curl http://localhost:5000/collisions
  ```

***

## Notes

- The vision system relies on color segmentation (cyan for goal, green for obstacles). Lighting or camera settings may affect performance.
- OpenCV is optional; without it, the robot moves cautiously with forward steps only.
- The autopilot automatically stops when the goal is reached.
- The project demonstrates an integration of perception, planning, and control in a web-based robot simulation.

***
