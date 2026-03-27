import socket
import json
import time
import numpy as np
import threading
import argparse
from robot import RobotDigitalTwin # Importerer robotmodellen fra det gamle oppsettet
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
        self.start_pose = [0.0, 0.0, 0.0]
        self.robot = RobotDigitalTwin(start_pose=self.start_pose, true_walls=self.TRUE_WALLS)
        
        # Vi lagrer en separat posisjon ("slam_pose") for at Go-serverens 
        # korreksjoner ikke skal ødelegge den rå odometrien (ekf.x).
        self.slam_pose = np.copy(self.robot.ekf.x)

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
        self.listener_thread = threading.Thread(target=self.listen_for_corrections)
        self.listener_thread.daemon = True
        self.listener_thread.start()

        self.controller = HybridController()
        self.waypoints = [
            (-1.5, 0.0),   # Kjør nord for å unngå veggen i midten
            (1.0, 0.0),    # Kryss til høyre side
            (1.0, -1.5),   # Kjør ned i høyre hjørne
            (1.0, 1.5),    # Kjør helt opp til høyre
            (-1.5, 1.5),   # Kryss tilbake til venstre
            (-1.5, -1.5)   # Tilbake til start
        ]
        self.current_wp_idx = 0

    def listen_for_corrections(self):
        f = self.conn.makefile('r')
        while self.running:
            try:
                line = f.readline()
                if not line:
                    break
                data = json.loads(line)
                if data.get("type") == "correction":
                    # Oppdaterer SLAM posisjonen i stedet for den rå EKF-en.
                    # Dette forhindrer en "feedback loop" som får roboten til å kjøre i sirkler.
                    self.slam_pose[0] = data["x"]
                    self.slam_pose[1] = data["y"]
                    self.slam_pose[2] = data["theta"]
            except Exception as e:
                print(f"Listener error: {e}")
                break

    def run(self):
        print("Starter kjøring og sender SLAM data til Go-serveren...")
        while True:
            # 1. Hent neste waypoint
            target_wp = self.waypoints[self.current_wp_idx]

            # 2. Hent rå odometri fra EKF
            ekf_pose = self.robot.ekf.x[:3].flatten()

            dt = 0.01

            # Formel for å konvertere v, w vha HybridController direkte
            # Vi bruker slam_pose for styringen, så den navigerer etter kartet
            try:
                omega_l, omega_r, is_done = self.controller.get_wheel_speeds(
                    current_pose=self.slam_pose[:3].flatten(),
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
                        # For "tomt rom" tvinger vi avstanden til 3000 for at Go-serveren ikke skal tegne en vegg
                        distance_mm = 3000
                    else:
                        # FYSISK KORREKT KAMERA: Det gir kun 1 avstand («closest object straight in front»)
                        min_y = min(p[1] for p in seg)
                        # Trekk fra camera_mount_offset som legges til igjen i go-server
                        distance_mm = int(np.round(min_y * 1000)) - 90
                        
                    cam_msg = {
                        "Id": self.robot_id,
                        "IsDirect": False,
                        "IsCamera": True,
                        "P1X": 0, "P1Y": 0,
                        "P2X": 0, "P2Y": 0,
                        "RhoMM": 0, "AlphaMRad": 0, 
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
