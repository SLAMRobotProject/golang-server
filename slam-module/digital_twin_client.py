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
        
        # TF-style frame tree:
        # odom -> base_link comes from EKF
        # map -> odom is updated from SLAM corrections
        self.map_to_odom = np.array([0.0, 0.0, 0.0], dtype=float)
        self.pose_lock = threading.Lock()

        self.conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        print(f"Prøver å koble til Go MQTT/TCP Broker på {host}:{port}...")
        try:
            self.conn.connect((host, port))
            print("Tilkoblet!")
        except Exception as e:
            print(f"Kunne ikke koble til: {e}")
            exit(1)

        # Start en lyttetråd for å motta korreksjoner fra Go-serveren
        self.running = True
        self.listener_thread = threading.Thread(target=self.listen_for_pose_updates)
        self.listener_thread.daemon = True
        self.listener_thread.start()

        self.controller = HybridController()
        self.waypoints = [
            (-1.0, -1.0),  # Tilbake til start
            (-1.0, 0.0),   # Kjør nord for å unngå veggen i midten
            (1.0, 0.0),    # Kryss til høyre side
            (1.0, -1.0),   # Kjør ned i høyre hjørne
            (1.0, 1.0),    # Kjør helt opp til høyre
            (-1.0, 1.0)    # Kryss tilbake til venstre
        ]
        self.current_wp_idx = 0

    @staticmethod
    def compose_pose(a, b):
        ax, ay, ath = a
        bx, by, bth = b
        ca = np.cos(ath)
        sa = np.sin(ath)
        x = ax + ca * bx - sa * by
        y = ay + sa * bx + ca * by
        th = normalize_angle(ath + bth)
        return np.array([x, y, th], dtype=float)

    @staticmethod
    def inverse_pose(p):
        x, y, th = p
        c = np.cos(th)
        s = np.sin(th)
        inv_x = -(c * x + s * y)
        inv_y = s * x - c * y
        return np.array([inv_x, inv_y, normalize_angle(-th)], dtype=float)

    def listen_for_pose_updates(self):
        f = self.conn.makefile('r')
        while self.running:
            try:
                line = f.readline()
                if not line:
                    break
                data = json.loads(line)
                if data.get("type") == "pose_update":
                    # SLAM sender en korrigert pose i map-frame.
                    # Vi oppdaterer kun map->odom-transformen og lar EKF være ren odometri.
                    with self.pose_lock:
                        odom_pose = self.robot.ekf.x[:3].copy()
                        map_pose = np.array([
                            data["x"],
                            data["y"],
                            normalize_angle(data["theta"]),
                        ], dtype=float)
                        self.map_to_odom = self.compose_pose(map_pose, self.inverse_pose(odom_pose))
            except Exception as e:
                print(f"Listener error: {e}")
                break

    def run(self):
        print("Starter kjøring og sender SLAM data til Go-serveren...")
        while True:
            # 1. Hent neste waypoint
            target_wp = self.waypoints[self.current_wp_idx]

            # 2. Hent rå odometri fra EKF og komponer korrigert map-pose.
            with self.pose_lock:
                ekf_pose = self.robot.ekf.x[:3].copy()
                map_to_odom = self.map_to_odom.copy()

            slam_pose = self.compose_pose(map_to_odom, ekf_pose)

            # Controller should follow the map-corrected pose, while EKF remains odom-only.
            with self.pose_lock:
                self.slam_pose = slam_pose.copy()

            dt = 0.01

            # Formel for å konvertere v, w vha HybridController direkte
            # Vi bruker slam_pose for styringen, så den navigerer etter kartet
            try:
                omega_l, omega_r, is_done = self.controller.get_wheel_speeds(
                    current_pose=slam_pose,
                    target_pose=target_wp,
                    dt=dt
                )

                # Bytt waypoint NÅR kontrolleren sier den er ferdig med target (inkludert piruett)
                if is_done:
                    self.current_wp_idx = (self.current_wp_idx + 1) % len(self.waypoints)

            except Exception as e:
                print(f"Controller error: {e}")
                # Fallback dersom no går feil
                omega_l, omega_r = 0.0, 0.0

            # Kjører fysikkmotoren (dt=0.01 for 100Hz simulering)
            res = self.robot.step_physics(target_omega_l=omega_l, target_omega_r=omega_r, dt=dt)
            time.sleep(dt) # Sørg for at den ruller i sanntid

            if res is not None:
                ekf_pose = res['ekf_pose']
                scan = res['scan']

                # Send AdvMsg (Odometri)
                adv_msg = {
                    "Id": self.robot_id,
                    "IsDirect": False,
                    "X": int(np.round(ekf_pose[0] * 1000)), # Go server expect mm (converted to cm in backend)
                    "Y": int(np.round(ekf_pose[1] * 1000)),
                    "Theta": int(np.round(np.rad2deg(ekf_pose[2]))),
                    "Ir1x": 0, "Ir1y": 0, "Ir2x": 0, "Ir2y": 0, "Ir3x": 0, "Ir3y": 0, "Ir4x": 0, "Ir4y": 0
                }
                # Pakk som JSON for digitalTwin.go
                try:
                    self.conn.sendall((json.dumps(adv_msg) + "\n").encode())
                except:
                    print("Mistet kobling...")
                    break

                # Grupper lidar-punkter til linjesegmenter (StartMM og WidthMM) for å simulere det virkelige kameraet
                # Kameraet identifiserer SAMMENHENGENDE objekter (bredde), og gir EN avstand rett frem til det nærmeste punktet på objektet.
                MAX_DIST_DIFF = 0.3 # 30 cm toleranse for å binde sammen kontinuerlige vegger til ett objekt
                segments = []
                current_segment = []

                for angle, dist in scan:
                    # x_body er horisontal (til sidene), y_body er dybde (fremover)
                    x_body = dist * np.sin(angle)
                    y_body = dist * np.cos(angle)

                    # Kameraet ser kun ting som faktisk er *foran* bilen
                    if y_body <= 0:
                        continue

                    if not current_segment:
                        current_segment.append((x_body, y_body, dist))
                    else:
                        last_x, last_y, last_dist = current_segment[-1]
                        # Grupper basert på radiell kontinuitet
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
                        # Send beyond MaxBuildDist so Go treats this as a no-hit ray
                        # (clears free space up to MaxBuildDist, marks no wall).
                        # 3000 mm was wrong: after +90 mm mount offset it became 3090 mm
                        # which is inside the hit threshold and created ghost walls.
                        distance_mm = 4000
                    else:
                        # FYSISK KORREKT KAMERA: Det gir kun 1 avstand («closest object straight in front»)
                        min_y = min(p[1] for p in seg)
                        # Trekk fra camera_mount_offset som legges til igjen i go-server
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
