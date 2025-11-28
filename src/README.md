# golang-server
This is a golang server based on communication via MQTT. It is made to be compatible with the robots from the SLAM robot project at NTNU. 

## How to run
Prerequisites: 
- Ensure that the robots are using the correct version of the robot code, which is currently located in the golang-server branch of the robot code.
- Ensure that the broker specified in ./config/config.go can be reached on the network. The code quits if it cannot connect to a broker, see the log file for confirmation.

Running from source:
1. Clone/download this repository.
1. Follow the Fyne (GUI package) [installation guide](https://developer.fyne.io/started/)
1. Open a terminal, navigate to this directory (the *src* directory) and run with `go run .` or build with `go build .`
   
The executable was created with `broker="slam"` meaning that it will only connect to the physical Raspberry Pi broker. Connecting to the Raspberry Pi broker can be done by connecting your computer to the Raspberry Pi WiFi: `BorderRouter-AP` with password `12345678`.

## How to run with Nicla Vision camera
To enable camera integration in the server, open config/config.go and set:
```
const UseNiclaVision = true
```
When enabled, the server subscribes to the camera topic and visualizes camera segments sent by the robot.

### With a physical camera
Follow the setup instructions provided in the ``robot_code`` repository to connect and configure the Nicla Vision camera.
Once the robot and camera are running, start the Go server. Incoming camera data should now appear in the GUI.

### Without a camera (simulation)
If no physical camera is available, camera input can be simulated using the testing/camera_e2e_test.go file, which publishes camera segments.

1. Start the server with UseNiclaVision = true.

2. Connect a robot (e.g., nRF5) to the system.

3. Open testing/camera_e2e_test.go.

4. Set robotID to match the connected robot ID:
```
robotID := 5
```
5. Run the test from the testing directory:
```
go test -v
```

This sends a simulated camera segment to the server.

To modify the test data, adjust the segment parameters inside camera_e2e_test.go:

```
	start := int16(0)
	width := int16(400)
	distance := int16(400)
```