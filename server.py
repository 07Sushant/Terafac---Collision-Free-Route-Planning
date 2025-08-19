import asyncio
import json
import websockets
from flask import Flask, request, jsonify
import threading
from threading import Event
import base64
import time
import math
import numpy as np
try:
    import cv2
except ImportError:
    cv2 = None

app = Flask(__name__)

# --- CORS: allow simple cross-origin calls from control page ---
@app.after_request
def add_cors_headers(resp):
    resp.headers['Access-Control-Allow-Origin'] = '*'
    resp.headers['Access-Control-Allow-Methods'] = 'GET,POST,OPTIONS'
    resp.headers['Access-Control-Allow-Headers'] = 'Content-Type'
    return resp

# ---------------------------
# Globals
# ---------------------------
connected = set()
async_loop = None
collision_count = 0  # server-tracked collisions

# Autopilot state
autopilot_running = False
autopilot_thread = None
autopilot_stop = Event()
latest_image = None            # numpy array (BGR) or None
latest_image_ts = 0.0
latest_pose = None             # dict like {x,y,z}
image_event = Event()

goal_corners_cycle = ["NE", "SE", "SW", "NW"]
goal_index = 0
current_goal = None            # dict with x,y,z
goal_reached_flag = False

# Config
STEP_SIZE = 1.2                # units per planning step
TURN_DEG = 20                  # degrees per corrective turn
CENTER_THRESHOLD = 5000        # pixel count threshold for "blocked" center

FLOOR_HALF = 50  # index.html uses PlaneGeometry(100, 100) centered at origin

def corner_to_coords(corner: str, margin=5):
    c = corner.upper()
    x = FLOOR_HALF - margin if "E" in c else -(FLOOR_HALF - margin)
    z = FLOOR_HALF - margin if ("S" in c or "B" in c) else -(FLOOR_HALF - margin)
    if c in ("NE", "EN", "TR"): x, z = (FLOOR_HALF - margin, -(FLOOR_HALF - margin))
    if c in ("NW", "WN", "TL"): x, z = (-(FLOOR_HALF - margin), -(FLOOR_HALF - margin))
    if c in ("SE", "ES", "BR"): x, z = (FLOOR_HALF - margin, (FLOOR_HALF - margin))
    if c in ("SW", "WS", "BL"): x, z = (-(FLOOR_HALF - margin), (FLOOR_HALF - margin))
    return {"x": x, "y": 0, "z": z}

# ---------------------------
# WebSocket Handler
# ---------------------------
async def ws_handler(websocket, path=None):
    global collision_count, latest_image, latest_image_ts, image_event, goal_reached_flag
    print("Client connected via WebSocket")
    connected.add(websocket)
    try:
        async for message in websocket:
            # Parse simulator messages and track collisions/images/goal events
            try:
                data = json.loads(message)
                if isinstance(data, dict):
                    # Count collisions
                    if data.get("type") == "collision" and data.get("collision"):
                        collision_count += 1
                    # Capture image response from simulator camera
                    if data.get("type") == "capture_image_response" and isinstance(data.get("image"), str):
                        # data:image/png;base64,.... remove header if present
                        img_str = data["image"]
                        if "," in img_str:
                            img_str = img_str.split(",", 1)[1]
                        try:
                            img_bytes = base64.b64decode(img_str)
                            # Decode PNG to numpy array (BGR) if cv2 available
                            if cv2 is not None:
                                nparr = np.frombuffer(img_bytes, np.uint8)
                                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                                if frame is not None:
                                    latest_image = frame
                                    latest_image_ts = time.time()
                                    image_event.set()
                        except Exception:
                            pass
                    # Any message with position updates the latest_pose
                    pos = data.get("position")
                    if isinstance(pos, dict) and "x" in pos and "z" in pos:
                        try:
                            latest_pose = {
                                "x": float(pos["x"]),
                                "y": float(pos.get("y", 0)),
                                "z": float(pos["z"])
                            }
                        except Exception:
                            pass
                    # Goal reached event
                    if data.get("type") == "goal_reached":
                        goal_reached_flag = True
            except Exception:
                pass
            print("Received from simulator:", message)
    except websockets.exceptions.ConnectionClosed:
        print("Client disconnected")
    finally:
        connected.remove(websocket)

def broadcast(msg: dict):
    if not connected:
        return False
    for ws in list(connected):
        asyncio.run_coroutine_threadsafe(ws.send(json.dumps(msg)), async_loop)
    return True

# ---------------------------
# Existing Endpoints
# ---------------------------
@app.route('/move', methods=['POST'])
def move():
    data = request.get_json()
    if not data or 'x' not in data or 'z' not in data:
        return jsonify({'error': 'Missing parameters. Please provide "x" and "z".'}), 400
    x, z = data['x'], data['z']
    msg = {"command": "move", "target": {"x": x, "y": 0, "z": z}}
    if not broadcast(msg):
        return jsonify({'error': 'No connected simulators.'}), 400
    return jsonify({'status': 'move command sent', 'command': msg})

@app.route('/move_rel', methods=['POST'])
def move_rel():
    data = request.get_json()
    if not data or 'turn' not in data or 'distance' not in data:
        return jsonify({'error': 'Missing parameters. Please provide "turn" and "distance".'}), 400
    msg = {"command": "move_relative", "turn": data['turn'], "distance": data['distance']}
    if not broadcast(msg):
        return jsonify({'error': 'No connected simulators.'}), 400
    return jsonify({'status': 'move relative command sent', 'command': msg})

@app.route('/stop', methods=['POST'])
def stop():
    msg = {"command": "stop"}
    if not broadcast(msg):
        return jsonify({'error': 'No connected simulators.'}), 400
    return jsonify({'status': 'stop command sent', 'command': msg})

@app.route('/capture', methods=['POST'])
def capture():
    msg = {"command": "capture_image"}
    if not broadcast(msg):
        return jsonify({'error': 'No connected simulators.'}), 400
    return jsonify({'status': 'capture command sent', 'command': msg})

# ---------------------------
# Goal + Obstacles (from your previous step)
# ---------------------------
@app.route('/goal', methods=['POST'])
def set_goal():
    data = request.get_json() or {}
    if 'corner' in data:
        pos = corner_to_coords(str(data['corner']))
    elif 'x' in data and 'z' in data:
        pos = {"x": float(data['x']), "y": float(data.get('y', 0)), "z": float(data['z'])}
    else:
        return jsonify({'error': 'Provide {"corner":"NE|NW|SE|SW"} OR {"x":..,"z":..}'}), 400

    msg = {"command": "set_goal", "position": pos}
    if not broadcast(msg):
        return jsonify({'error': 'No connected simulators.'}), 400
    return jsonify({'status': 'goal set', 'goal': pos})

@app.route('/obstacles/positions', methods=['POST'])
def set_obstacle_positions():
    data = request.get_json() or {}
    positions = data.get('positions')
    if not isinstance(positions, list) or not positions:
        return jsonify({'error': 'Provide "positions" as a non-empty list.'}), 400

    norm = []
    for p in positions:
        if not isinstance(p, dict) or 'x' not in p or 'z' not in p:
            return jsonify({'error': 'Each position needs "x" and "z".'}), 400
        norm.append({"x": float(p['x']), "y": float(p.get('y', 2)), "z": float(p['z'])})

    msg = {"command": "set_obstacles", "positions": norm}
    if not broadcast(msg):
        return jsonify({'error': 'No connected simulators.'}), 400
    return jsonify({'status': 'obstacles updated', 'count': len(norm)})

@app.route('/obstacles/motion', methods=['POST'])
def set_obstacle_motion():
    data = request.get_json() or {}
    if 'enabled' not in data:
        return jsonify({'error': 'Missing "enabled" boolean.'}), 400

    msg = {
        "command": "set_obstacle_motion",
        "enabled": bool(data['enabled']),
        "speed": float(data.get('speed', 0.05)),
        "velocities": data.get('velocities'),
        "bounds": data.get('bounds', {"minX": -45, "maxX": 45, "minZ": -45, "maxZ": 45}),
        "bounce": bool(data.get('bounce', True)),
    }
    if not broadcast(msg):
        return jsonify({'error': 'No connected simulators.'}), 400
    return jsonify({'status': 'obstacle motion updated', 'config': msg})

# ---------------------------
# NEW: Autopilot (Perception + Planning + Control)
# ---------------------------

def send_command(cmd: dict) -> bool:
    return broadcast(cmd)


def set_goal_corner(corner: str):
    global current_goal
    pos = corner_to_coords(corner)
    current_goal = pos
    send_command({"command": "set_goal", "position": pos})


def request_capture(timeout=2.0):
    """Ask simulator to capture image and wait for the latest_image to update."""
    image_event.clear()
    send_command({"command": "capture_image"})
    image_event.wait(timeout)
    return latest_image


def analyze_image(frame) -> tuple:
    """Return (turn_deg, forward_distance, had_goal) using HSV segmentation.
    - Steer toward goal flag (cyan) if visible.
    - Avoid center obstacles (green) by turning away.
    - Otherwise go straight.
    """
    if frame is None or cv2 is None:
        # Fallback: small forward steps
        return 0.0, STEP_SIZE, False

    # Resize for speed
    h, w = frame.shape[:2]
    scale = 480.0 / max(h, w)
    if scale < 1.0:
        frame = cv2.resize(frame, (int(w*scale), int(h*scale)))
        h, w = frame.shape[:2]

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Cyan mask (goal flag color ~ 0x00ccff)
    cyan_lower = np.array([85, 80, 60], dtype=np.uint8)
    cyan_upper = np.array([105, 255, 255], dtype=np.uint8)
    cyan = cv2.inRange(hsv, cyan_lower, cyan_upper)

    # Green mask (obstacles ~ 0x00ff00)
    green_lower = np.array([35, 80, 60], dtype=np.uint8)
    green_upper = np.array([85, 255, 255], dtype=np.uint8)
    green = cv2.inRange(hsv, green_lower, green_upper)

    # Center vertical strip for "blocked" detection
    cx0 = int(w*0.4); cx1 = int(w*0.6)
    center_green = green[:, cx0:cx1]
    blocked = int(center_green.sum() // 255) > CENTER_THRESHOLD

    # Goal direction via cyan centroid
    M = cv2.moments(cyan)
    goal_dir = None
    had_goal = False
    if M['m00'] > 500:  # area threshold
        gx = int(M['m10']/M['m00'])
        # horizontal offset: -1..1
        offset = (gx - w/2) / (w/2)
        goal_dir = max(-1.0, min(1.0, offset))
        had_goal = True

    # Decide turn
    turn = 0.0
    if blocked:
        # Compare green density left vs right
        left = int(green[:, :w//2].sum() // 255)
        right = int(green[:, w//2:].sum() // 255)
        # Turn opposite of denser side
        turn = TURN_DEG if left > right else -TURN_DEG
    elif goal_dir is not None:
        # Proportional steering toward goal
        turn = float(goal_dir * TURN_DEG)

    forward = STEP_SIZE if not blocked else 0.0
    if forward == 0.0:
        # If blocked, take a small sidestep after turning
        forward = STEP_SIZE * 0.6
    return turn, forward, had_goal


def autopilot_main():
    global autopilot_running, goal_index, goal_reached_flag
    try:
        # Ensure we have a goal
        if current_goal is None:
            set_goal_corner(goal_corners_cycle[goal_index % len(goal_corners_cycle)])
        goal_reached_flag = False

        while not autopilot_stop.is_set():
            # If goal reached, rotate to next one
            if goal_reached_flag:
                # Stop autopilot on destination reach
                autopilot_stop.set()
                break

            # Perception
            frame = request_capture(timeout=2.0)
            # Planning
            turn, forward, saw_goal = analyze_image(frame)

            # If we have the robot pose and a goal, compute an absolute waypoint toward the goal
            waypoint = None
            if latest_pose and current_goal:
                dx = current_goal["x"] - latest_pose["x"]
                dz = current_goal["z"] - latest_pose["z"]
                dist = math.hypot(dx, dz)
                if dist > 0.5:
                    step = min(STEP_SIZE, dist)
                    nx = latest_pose["x"] + (dx / (dist + 1e-6)) * step
                    nz = latest_pose["z"] + (dz / (dist + 1e-6)) * step
                    # Clamp to floor bounds minus margin
                    margin = 48.5
                    nx = max(-margin, min(margin, nx))
                    nz = max(-margin, min(margin, nz))
                    waypoint = {"x": nx, "y": 0, "z": nz}

            # Control: prefer absolute move toward goal if we have pose; else use relative
            if waypoint:
                send_command({"command": "move", "target": waypoint})
            else:
                send_command({"command": "move_relative", "turn": float(turn), "distance": float(forward)})

            # Pace
            for _ in range(6):
                if autopilot_stop.is_set():
                    break
                time.sleep(0.05)
    finally:
        autopilot_running = False
        # Stop robot when exiting
        send_command({"command": "stop"})


@app.route('/autopilot/start', methods=['POST'])
def autopilot_start():
    """Start autonomous navigation loop. Optional JSON: {"moving_obstacles": bool, "speed": number}
    """
    global autopilot_running, autopilot_thread
    if autopilot_running:
        return jsonify({"status": "already running"})
    # Optionally enable moving obstacles
    # Accept empty body; only read JSON if Content-Type is JSON
    data = {}
    if request.headers.get('Content-Type', '').startswith('application/json'):
        try:
            data = request.get_json(silent=True) or {}
        except Exception:
            data = {}
    if data.get("moving_obstacles") is not None:
        mo_msg = {
            "command": "set_obstacle_motion",
            "enabled": bool(data.get("moving_obstacles")),
            "speed": float(data.get("speed", 0.05)),
        }
        send_command(mo_msg)

    autopilot_stop.clear()
    autopilot_running = True
    autopilot_thread = threading.Thread(target=autopilot_main, daemon=True)
    autopilot_thread.start()
    return jsonify({"status": "started"})


@app.route('/autopilot/stop', methods=['POST'])
def autopilot_stop_ep():
    autopilot_stop.set()
    return jsonify({"status": "stopping"})


@app.route('/autopilot/status', methods=['GET'])
def autopilot_status():
    return jsonify({
        "running": autopilot_running,
        "cv_enabled": cv2 is not None,
        "current_goal": current_goal,
        "collisions": collision_count,
        "last_image_ts": latest_image_ts,
        "next_corner": goal_corners_cycle[(goal_index+1) % len(goal_corners_cycle)] if current_goal else None
    })

# ---------------------------
# NEW: Collisions & Reset
# ---------------------------
@app.route('/collisions', methods=['GET'])
def get_collisions():
    """Return the total number of collisions seen (from simulator messages)."""
    return jsonify({'count': collision_count})

@app.route('/reset', methods=['POST'])
def reset():
    """Reset collision count and broadcast a reset command to the simulator."""
    global collision_count
    collision_count = 0
    if not broadcast({"command": "reset"}):
        # Even if no simulator is connected, we consider the counter reset.
        return jsonify({'status': 'reset done (no simulators connected)', 'collisions': collision_count})
    return jsonify({'status': 'reset broadcast', 'collisions': collision_count})

# ---------------------------
# Flask Thread
# ---------------------------
def start_flask():
    app.run(port=5000)

# ---------------------------
# Main Async for WebSocket
# ---------------------------
async def main():
    global async_loop
    async_loop = asyncio.get_running_loop()
    ws_server = await websockets.serve(ws_handler, "localhost", 8080)
    print("WebSocket server started on ws://localhost:8080")
    await ws_server.wait_closed()

# ---------------------------
# Entry
# ---------------------------
if __name__ == "__main__":
    flask_thread = threading.Thread(target=start_flask, daemon=True)
    flask_thread.start()
    asyncio.run(main())
