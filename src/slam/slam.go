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
	X         float64 `json:"x"`
	Y         float64 `json:"y"`
	Theta     float64 `json:"theta"`
	TimeStamp int64   `json:"time_stamp"`
}

// WsImuMsg matches the essential fields for sensor_msgs/Imu
type WsImuMsg struct {
	AccelX    float64 `json:"accel_x"`
	AccelY    float64 `json:"accel_y"`
	AccelZ    float64 `json:"accel_z"`
	GyroX     float64 `json:"gyro_x"`
	GyroY     float64 `json:"gyro_y"`
	GyroZ     float64 `json:"gyro_z"`
	TimeStamp int64   `json:"time_stamp"`
}

type WsPointData struct {
	Point     T.Point2D `json:"point"`
	TimeStamp int64     `json:"time_stamp"`
}

// WsScanMsg matches the essential fields for sensor_msgs/LaserScan
type WsScanMsg struct {
	AngleMin       float64 `json:"angle_min"`       // 0.0
	AngleMax       float64 `json:"angle_max"`       // 2*Pi
	AngleIncrement float64 `json:"angle_increment"` // (2*Pi) / 180
	RangeMin       float32 `json:"range_min"`       // e.g., 0.1m
	RangeMax       float32 `json:"range_max"`       // e.g., 0.5m
	//Ranges         []float32 `json:"ranges"`          // The 180 scan points
	Points []WsPointData `json:"points"`
}

// --- Helper Functions ---

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

	// Pre-calculate scan metadata
	scanAngleIncrementRad := float64(SCAN_ANGLE_INCREMENT_DEG) * math.Pi / 180.0
	scanMsgMetadata := WsScanMsg{
		AngleMin: 0.0,
		// AngleMax is (N-1) * increment. Here N=180
		AngleMax:       float64(MAX_RANGER_DATA_POINTS-1) * scanAngleIncrementRad,
		AngleIncrement: scanAngleIncrementRad,
		RangeMin:       0.1,
		RangeMax:       0.6,
	}

	log.Println("SlamBackendLoop started. Waiting for sensor data...")

	for {
		select {
		case newData := <-chSensorData:
			timeNow := time.Now()

			// --- 1. Send Odometry ---
			// Cartographer needs odom at a high rate.
			odomMsg := WsOdomMsg{
				X:         newData.Odometry.X,
				Y:         newData.Odometry.Y,
				Theta:     newData.Odometry.Theta,
				TimeStamp: timeNow.Unix(),
			}

			if err := sendJSON(conn, &mu, WsWrapperMsg{Type: "odom", Data: odomMsg}); err != nil {
				log.Printf("Error sending odom: %v. Exiting loop.", err)
				return
			}

			// --- 2. Send IMU ---
			// Cartographer needs IMU at a high rate.
			imuMsg := WsImuMsg{
				AccelX:    newData.Imu.AccelX,
				AccelY:    newData.Imu.AccelY,
				AccelZ:    newData.Imu.AccelZ,
				GyroX:     newData.Imu.GyroX,
				GyroY:     newData.Imu.GyroY,
				GyroZ:     newData.Imu.GyroZ,
				TimeStamp: timeNow.Unix(),
			}
			if err := sendJSON(conn, &mu, WsWrapperMsg{Type: "imu", Data: imuMsg}); err != nil {
				log.Printf("Error sending imu: %v. Exiting loop.", err)
				return
			}

			// --- 3. Accumulate Scan Data ---
			// This logic assumes IRTowerAngle is in degrees [0, 358] and increments by 2
			//currentAngle := int(newData.IRTowerAngle)

			points := []WsPointData{
				{
					Point: T.Point2D{
						X: newData.IRSensors.IrSensorData_0.X,
						Y: newData.IRSensors.IrSensorData_0.Y,
					},
					TimeStamp: timeNow.UnixNano(),
				},
				{
					Point: T.Point2D{
						X: newData.IRSensors.IrSensorData_1.X,
						Y: newData.IRSensors.IrSensorData_1.Y,
					},
					TimeStamp: timeNow.UnixNano(),
				},
				{
					Point: T.Point2D{
						X: newData.IRSensors.IrSensorData_2.X,
						Y: newData.IRSensors.IrSensorData_2.Y,
					},
					TimeStamp: timeNow.UnixNano(),
				},
				{
					Point: T.Point2D{
						X: newData.IRSensors.IrSensorData_3.X,
						Y: newData.IRSensors.IrSensorData_3.Y,
					},
					TimeStamp: timeNow.UnixNano(),
				},
			}
			scanMsgMetadata.Points = points

			if err := sendJSON(conn, &mu, WsWrapperMsg{Type: "points", Data: scanMsgMetadata}); err != nil {
				log.Printf("Error sending points: %v. Exiting loop.", err)
				return
			}

		default:
			// No new data, yield to other goroutines
			runtime.Gosched()
			time.Sleep(1 * time.Millisecond)
		}
	}
}
