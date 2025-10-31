package slam

// This package contains code to process SLAM messages from robots.
// It converts MQTT messages to Cartographer format and processes them.
// It maintains the SLAM state and updates the map accordingly.

import (
	"log"
	"math"
	"runtime"
	"sync"
	"time"

	T "golang-server/types"

	"github.com/gorilla/websocket" // 'go get' this library
)

const (
	SCAN_ANGLE_INCREMENT_DEG = 2                              // 2 degrees
	MAX_RANGER_DATA_POINTS   = 360 / SCAN_ANGLE_INCREMENT_DEG // 180 data points
)

// --- JSON Message Structs (to match ROS) ---

// WsWrapperMsg is a generic container for all message types
type WsWrapperMsg struct {
	Type string      `json:"Type"` // "odom", "imu", "scan"
	Data interface{} `json:"Data"`
}

// WsOdomMsg matches the essential fields for nav_msgs/Odometry
type WsOdomMsg struct {
	X     float64 `json:"x"`
	Y     float64 `json:"y"`
	Theta float64 `json:"theta"`
}

// WsImuMsg matches the essential fields for sensor_msgs/Imu
type WsImuMsg struct {
	AccelX float64 `json:"accel_x"`
	AccelY float64 `json:"accel_y"`
	AccelZ float64 `json:"accel_z"`
	GyroX  float64 `json:"gyro_x"`
	GyroY  float64 `json:"gyro_y"`
	GyroZ  float64 `json:"gyro_z"`
}

type WsPointData struct {
	Range      float32 `json:"range"`
	TimeOffset float64 `json:"time_offset"`
}

// WsScanMsg matches the essential fields for sensor_msgs/LaserScan
type WsScanMsg struct {
	AngleMin       float64 `json:"angle_min"`       // 0.0
	AngleMax       float64 `json:"angle_max"`       // 2*Pi
	AngleIncrement float64 `json:"angle_increment"` // (2*Pi) / 180
	RangeMin       float32 `json:"range_min"`       // e.g., 0.1m
	RangeMax       float32 `json:"range_max"`       // e.g., 8.0m
	//Ranges         []float32 `json:"ranges"`          // The 180 scan points
	Points []WsPointData `json:"points"`
}

// --- Helper Functions ---

// cartesianToDistance converts a 2D point to just its radial distance
func cartesianToDistance(point T.Point2D) float32 {
	x := float64(point.X)
	y := float64(point.Y)
	r := math.Sqrt(x*x + y*y)
	// Might need to convert this from 'mm' to 'm' depending on the robot
	return float32(r) / 1000.0 // m
	//return float32(r) // mm
}

// sendJSON over WebSocket in a thread-safe way
func sendJSON(conn *websocket.Conn, mu *sync.Mutex, msg interface{}) error {
	mu.Lock()
	defer mu.Unlock()
	err := conn.WriteJSON(msg)
	if err != nil {
		log.Printf("Error sending JSON: %v", err)
		return err
	}
	return nil
}

// --- Main Loop ---

func SlamBackendLoop(
	chSensorData <-chan T.SensorData,
	conn *websocket.Conn,
) {
	var mu sync.Mutex

	fullScanData := make([]WsPointData, MAX_RANGER_DATA_POINTS)

	// Pre-calculate scan metadata
	scanAngleIncrementRad := float64(SCAN_ANGLE_INCREMENT_DEG) * math.Pi / 180.0
	scanMsgMetadata := WsScanMsg{
		AngleMin: 0.0,
		// AngleMax is (N-1) * increment. Here N=180
		AngleMax:       float64(MAX_RANGER_DATA_POINTS-1) * scanAngleIncrementRad,
		AngleIncrement: scanAngleIncrementRad,
		RangeMin:       0.1,
		RangeMax:       0.5,
	}

	var previousAngle int = -1 // Use -1 to indicate first run
	var sweepingUp bool = true

	var scanStartTime time.Time

	log.Println("SlamBackendLoop started. Waiting for sensor data...")

	for {
		select {
		case newData := <-chSensorData:
			// --- 1. Send Odometry ---
			// Cartographer needs odom at a high rate.
			odomMsg := WsOdomMsg{
				X:     newData.Odometry.X,
				Y:     newData.Odometry.Y,
				Theta: newData.Odometry.Theta,
			}

			if err := sendJSON(conn, &mu, WsWrapperMsg{Type: "odom", Data: odomMsg}); err != nil {
				log.Printf("Error sending odom: %v. Exiting loop.", err)
				return
			}

			// --- 2. Send IMU ---
			// Cartographer needs IMU at a high rate.
			imuMsg := WsImuMsg{
				AccelX: newData.Imu.AccelX,
				AccelY: newData.Imu.AccelY,
				AccelZ: newData.Imu.AccelZ,
				GyroX:  newData.Imu.GyroX,
				GyroY:  newData.Imu.GyroY,
				GyroZ:  newData.Imu.GyroZ,
			}
			if err := sendJSON(conn, &mu, WsWrapperMsg{Type: "imu", Data: imuMsg}); err != nil {
				log.Printf("Error sending imu: %v. Exiting loop.", err)
				return
			}

			// --- 3. Accumulate Scan Data ---
			// This logic assumes IRTowerAngle is in degrees [0, 358] and increments by 2
			currentAngle := int(newData.IRTowerAngle)

			if (sweepingUp && previousAngle > 350 && currentAngle < 10) ||
				(!sweepingUp && previousAngle < 10 && currentAngle > 350) {
				scanStartTime = time.Now()
			}

			currentTimeOffset := time.Since(scanStartTime).Seconds()

			// Sensor 0 (at IRTowerAngle)
			idx0 := (newData.IRTowerAngle) / SCAN_ANGLE_INCREMENT_DEG
			if idx0 >= 0 && idx0 < MAX_RANGER_DATA_POINTS {
				fullScanData[idx0].Range = cartesianToDistance(newData.IRSensors.IrSensorData_0)
				fullScanData[idx0].TimeOffset = currentTimeOffset
			}

			// Sensor 1 (at IRTowerAngle + 90)
			angle1 := (newData.IRTowerAngle + 90) % 360
			idx1 := angle1 / SCAN_ANGLE_INCREMENT_DEG
			if idx1 >= 0 && idx1 < MAX_RANGER_DATA_POINTS {
				fullScanData[idx1].Range = cartesianToDistance(newData.IRSensors.IrSensorData_1)
				fullScanData[idx1].TimeOffset = currentTimeOffset
			}

			// Sensor 2 (at IRTowerAngle + 180)
			angle2 := (newData.IRTowerAngle + 180) % 360
			idx2 := angle2 / SCAN_ANGLE_INCREMENT_DEG
			if idx2 >= 0 && idx2 < MAX_RANGER_DATA_POINTS {
				fullScanData[idx2].Range = cartesianToDistance(newData.IRSensors.IrSensorData_2)
				fullScanData[idx2].TimeOffset = currentTimeOffset
			}

			// Sensor 3 (at IRTowerAngle + 270)
			angle3 := (newData.IRTowerAngle + 270) % 360
			idx3 := angle3 / SCAN_ANGLE_INCREMENT_DEG
			if idx3 >= 0 && idx3 < MAX_RANGER_DATA_POINTS {
				fullScanData[idx3].Range = cartesianToDistance(newData.IRSensors.IrSensorData_3)
				fullScanData[idx3].TimeOffset = currentTimeOffset
			}

			// --- 4. Send Full Scan ---
			sendScan := false

			if previousAngle != -1 {
				if currentAngle > previousAngle {
					sweepingUp = true
				} else if currentAngle < previousAngle {
					sweepingUp = false
				}
				// 1. Finished sweeping up:
				if sweepingUp && previousAngle < 90 && currentAngle >= 90 {
					sendScan = true
				}
				// 2. Finished sweeping down:
				if !sweepingUp && previousAngle > 0 && currentAngle <= 0 {
					sendScan = true
				}
			}
			previousAngle = currentAngle

			if sendScan {
				scanMsg := scanMsgMetadata
				scanMsg.Points = make([]WsPointData, MAX_RANGER_DATA_POINTS)
				copy(scanMsg.Points, fullScanData)
				if err := sendJSON(conn, &mu, WsWrapperMsg{Type: "scan", Data: scanMsg}); err != nil {
					log.Printf("Error sending scan: %v. Exiting loop.", err)
					return
				}
				log.Println("Sent FULL SCAN")

				// --- Clear data for next sweep ---
				fullScanData = make([]WsPointData, MAX_RANGER_DATA_POINTS)

				// Reset scan start time for the next sweep
				scanStartTime = time.Now()
			}

		default:
			// No new data, yield to other goroutines
			runtime.Gosched()
			time.Sleep(100 * time.Millisecond)
		}
	}
}

// --- Simulation Function ---

// SimulateSensorData sends fake sensor data to a channel
func SimulateSensorData(chSensorData chan<- T.SensorData) {
	log.Println("Starting sensor simulation...")

	// Ticker to send data ~20 times per second
	ticker := time.NewTicker(50 * time.Millisecond)
	defer ticker.Stop()
	defer close(chSensorData)

	// --- State for 0-90-0 sweep simulation ---
	var currentAngle int32 = 0
	var simSweepingUp bool = true

	var currentX float64 = 0.0
	var currentTheta float64 = 0.0

	for range ticker.C {
		// Create fake data
		data := T.SensorData{
			Id: 1,
			Odometry: T.Odometry{
				X:     currentX,
				Y:     0.0,
				Theta: currentTheta,
			},
			Imu: T.Imu{
				AccelX: 0.1,
				AccelY: 0.0,
				AccelZ: 0.0,
				GyroZ:  0.05, // Simulating a slight turn
			},
			IRSensors: T.IRSensors{
				// Simulate seeing a wall 2 meters away
				IrSensorData_0: T.Point2D{X: 2.0, Y: 0.0},
				IrSensorData_1: T.Point2D{X: 3.0, Y: 0.0},
				IrSensorData_2: T.Point2D{X: 5.0, Y: 0.0},
				IrSensorData_3: T.Point2D{X: 7.0, Y: 0.0},
			},
			IRTowerAngle: currentAngle,
		}

		// Send data to the processing loop
		chSensorData <- data

		// Update simulation state
		if simSweepingUp {
			currentAngle += SCAN_ANGLE_INCREMENT_DEG
			if currentAngle >= 90 {
				currentAngle = 90
				simSweepingUp = false
			}
		} else {
			currentAngle -= SCAN_ANGLE_INCREMENT_DEG
			if currentAngle <= 0 {
				currentAngle = 0
				simSweepingUp = true
			}
		}
		currentTheta += 0.001
	}
}
