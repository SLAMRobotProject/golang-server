import socket
import json
import time
import numpy as np
import threading
import argparse
from robot import RobotDigitalTwin, normalize_angle
from controller import HybridController

class DigitalTwinClient:
    def __init__(self, host='localhost', port=9000, robot_id=1):
        self.robot_id = robot_id

        # Bygg et rom sentrert rundt 0,0 (-2m til +2m i begge retninger)
        self.TRUE_WALLS = np.array([
            [[-2.0, -2.0], [2.0, -2.0]],
            [[2.0, -2.0], [2.0, 2.0]],
            [[2.0, 2.0], [-2.0, 2.0]],
            [[-2.0, 2.0], [-2.0, -2.0]],
            [[0.0, -2.0], [0.0, -0.5]]
        ])

        # Start i kvadrant 3 (nederst til venstre, negativ X og negativ Y)
        self.start_pose = [-0.5, -0.5, 0.0]
        self.robot = RobotDigitalTwin(start_pose=self.start_pose, true_walls=self.TRUE_WALLS)

        self.pose_lock = threading.Lock()

        # Current target in odom frame received from the Go server (metres).
        # None means no command received yet — robot stays still.
        self.current_target = None

        self.conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        print(f"Prøver å koble til Go MQTT/TCP Broker på {host}:{port}...")
        try:
            self.conn.connect((host, port))
            print("Tilkoblet!")
        except Exception as e:
            print(f"Kunne ikke koble til: {e}")
            exit(1)

        # Listener thread receives both pose corrections and target commands from Go server.
        self.running = True
        self.listener_thread = threading.Thread(target=self.listen_for_messages)
        self.listener_thread.daemon = True
        self.listener_thread.start()

        self.controller = HybridController()

    def listen_for_messages(self):
        f = self.conn.makefile('r')
        while self.running:
            try:
                line = f.readline()
                if not line:
                    break
                data = json.loads(line)
                msg_type = data.get("type")

                if msg_type == "target":
                    # Go server sends odom-frame target computed from SLAM-corrected pose.
                    with self.pose_lock:
                        new_target = (data["x"], data["y"])
                        if new_target != self.current_target:
                            self.current_target = new_target
                            self.controller = HybridController()  # reset state machine for new target

            except Exception as e:
                print(f"Listener error: {e}")
                break

    def run(self):
        print("Starter kjøring og sender SLAM data til Go-serveren...")
        while True:
            with self.pose_lock:
                target = self.current_target

            dt = 0.01

            if target is not None:
                # Navigate in odom frame using raw EKF pose — no map correction needed here
                # because the Go server already converted the map-frame waypoint to odom frame.
                ekf_pose = self.robot.ekf.x[:3].copy()
                try:
                    omega_l, omega_r, is_done = self.controller.get_wheel_speeds(
                        current_pose=ekf_pose,
                        target_pose=target,
                        dt=dt
                    )
                    if is_done:
                        # Waypoint reached (pirouette complete). Stop and wait for the
                        # next command from the Go server's VirtualPath control.
                        with self.pose_lock:
                            self.current_target = None
                        omega_l, omega_r = 0.0, 0.0
                except Exception as e:
                    print(f"Controller error: {e}")
                    omega_l, omega_r = 0.0, 0.0
            else:
                omega_l, omega_r = 0.0, 0.0

            # Run physics engine (dt=0.01 for 100 Hz simulation)
            res = self.robot.step_physics(target_omega_l=omega_l, target_omega_r=omega_r, dt=dt)
            time.sleep(dt)

            if res is not None:
                ekf_pose = res['ekf_pose']
                scan = res['scan']

                # Send AdvMsg (Odometry)
                adv_msg = {
                    "Id": self.robot_id,
                    "IsDirect": False,
                    "X": int(np.round(ekf_pose[0] * 1000)),  # Go server expects mm
                    "Y": int(np.round(ekf_pose[1] * 1000)),
                    "Theta": int(np.round(np.rad2deg(ekf_pose[2]))),
                    "Ir1x": 0, "Ir1y": 0, "Ir2x": 0, "Ir2y": 0, "Ir3x": 0, "Ir3y": 0, "Ir4x": 0, "Ir4y": 0
                }
                try:
                    self.conn.sendall((json.dumps(adv_msg) + "\n").encode())
                except:
                    print("Mistet kobling...")
                    break

                # Group lidar points into line segments (StartMM, WidthMM) to simulate the real camera.
                MAX_DIST_DIFF = 0.3
                segments = []
                current_segment = []

                for angle, dist in scan:
                    x_body = dist * np.sin(angle)
                    y_body = dist * np.cos(angle)

                    if y_body <= 0:
                        continue

                    if not current_segment:
                        current_segment.append((x_body, y_body, dist))
                    else:
                        last_x, last_y, last_dist = current_segment[-1]
                        if (dist >= 2.95 and last_dist >= 2.95) or abs(dist - last_dist) < MAX_DIST_DIFF:
                            current_segment.append((x_body, y_body, dist))
                        else:
                            segments.append(current_segment)
                            current_segment = [(x_body, y_body, dist)]

                if current_segment:
                    segments.append(current_segment)

                for seg in segments:
                    if not seg: continue
                    min_x = min(p[0] for p in seg)
                    max_x = max(p[0] for p in seg)

                    is_empty_space = all(p[2] >= 2.95 for p in seg)

                    if is_empty_space:
                        distance_mm = 4000
                    else:
                        min_y = min(p[1] for p in seg)
                        distance_mm = int(np.round(min_y * 1000)) - 90

                    cam_msg = {
                        "Id": self.robot_id,
                        "IsDirect": False,
                        "IsCamera": True,
                        "StartMM": int(np.round(min_x * 1000)),
                        "WidthMM": int(np.round((max_x - min_x) * 1000)),
                        "DistanceMM": distance_mm
                    }

                    try:
                        self.conn.sendall((json.dumps(cam_msg) + "\n").encode())
                    except:
                        break

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Digital Twin Client")
    parser.add_argument("--id", type=int, default=1, help="Robot ID (default: 1)")
    parser.add_argument("--host", type=str, default="localhost", help="Host address (default: localhost)")
    parser.add_argument("--port", type=int, default=9000, help="Port to connect (default: 9000)")

    args = parser.parse_args()

    node = DigitalTwinClient(host=args.host, port=args.port, robot_id=args.id)
    node.run()
