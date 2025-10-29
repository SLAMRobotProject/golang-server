package slam

// This package contains code to process SLAM messages from robots.
// It converts MQTT messages to Cartographer format and processes them.
// It maintains the SLAM state and updates the map accordingly.

import (
	"encoding/json"
	T "golang-server/types"
	"math"
	"runtime"
)

const SCAN_ANGLE_INCREMENT = 2
const MAX_RANGER_DATA = 360 / SCAN_ANGLE_INCREMENT

type Ranger struct {
	ScanAngleMin       float32      `json:"scan_angle_min"`
	ScanAngleMax       float32      `json:"scan_angle_max"`
	ScanAngleIncrement float32      `json:"scan_angle_increment"`
	RangerData         []RangerData `json:"ranger_data"`
}

type RangerData struct {
	Angle    int32 `json:"angle"`
	Distance int32 `json:"distance"`
}

type CartoSlamMsg struct {
	Id       int
	Odometry T.Odometry
	Imu      T.Imu
	Ranger   Ranger
}

func sensorTo2DCartoMsg(sensorData T.SensorData) (CartoSlamMsg, error) {
	var cartoMsg CartoSlamMsg

	// Robot ID
	cartoMsg.Id = int(sensorData.Id)

	// Odometry
	cartoMsg.Odometry.X = sensorData.Odometry.X
	cartoMsg.Odometry.Y = sensorData.Odometry.Y
	cartoMsg.Odometry.Z = 0
	cartoMsg.Odometry.Theta = sensorData.Odometry.Theta

	// Accel
	cartoMsg.Imu.AccelX = sensorData.Imu.AccelX
	cartoMsg.Imu.AccelY = sensorData.Imu.AccelY
	cartoMsg.Imu.AccelZ = 0

	// Gyro
	cartoMsg.Imu.GyroX = sensorData.Imu.GyroX
	cartoMsg.Imu.GyroY = sensorData.Imu.GyroY
	cartoMsg.Imu.GyroZ = 0

	// Ranger Data - combine all IR sensors into one slice
	cartoMsg.Ranger.ScanAngleMin = -math.Pi / 2
	cartoMsg.Ranger.ScanAngleMax = math.Pi / 2
	cartoMsg.Ranger.ScanAngleIncrement = 2 * math.Pi / 180 // 2 degrees in radians

	// IR sensor data // ir tower 4 sectors to 360 scan
	point2D := sensorData.IRSensors.IrSensorData_0
	cartoMsg.Ranger.RangerData[sensorData.IRTowerAngle] = cartesianToPolar(point2D)

	point2D = sensorData.IRSensors.IrSensorData_1
	cartoMsg.Ranger.RangerData[sensorData.IRTowerAngle+90] = cartesianToPolar(point2D)

	point2D = sensorData.IRSensors.IrSensorData_2
	cartoMsg.Ranger.RangerData[sensorData.IRTowerAngle+180] = cartesianToPolar(point2D)

	point2D = sensorData.IRSensors.IrSensorData_3
	cartoMsg.Ranger.RangerData[sensorData.IRTowerAngle+270] = cartesianToPolar(point2D)

	return cartoMsg, nil
}

func sendCartoMsg(cartoMsg CartoSlamMsg) {
	// Send the CartoSlamMsg to the Cartographer
	//fmt.Sprintf("CartoSlamMsg: %+v", cartoMsg)
	msg := map[string]interface{}{
		"Id":       cartoMsg.Id,
		"Odometry": cartoMsg.Odometry,
		"Imu":      cartoMsg.Imu,
	}

	if len(cartoMsg.Ranger.RangerData) > 0 {
		msg["Ranger"] = cartoMsg.Ranger
	}

	jsonData, err := json.Marshal(msg)
	if err != nil {
		return
	}

	_ = jsonData // Replace with actual sending logic

}

func SlamBackendLoop(
	chSensorData <-chan T.SensorData,
) {
	// Initialize SLAM state
	// var slamState SlamState
	var fullScanData []RangerData

	for {
		select {
		case newData := <-chSensorData:
			cartoMsg, err := sensorTo2DCartoMsg(newData)
			if err != nil {
				continue
			}

			if newData.IRTowerAngle == 0 || newData.IRTowerAngle == 180 {
				// Full rotation completed
				cartoMsg.Ranger.RangerData = fullScanData
				fullScanData = fullScanData[:0] // Reset full scan data

				sendCartoMsg(cartoMsg)
			} else {
				// Send odometry and IMU data to Cartographer
				sendCartoMsg(cartoMsg)
			}

		default:
			// No new data
			// Release task to avoid busy waiting
			runtime.Gosched()

		}

		// Send
	}
}

func cartesianToPolar(point T.Point2D) RangerData {
	x := float64(point.X)
	y := float64(point.Y)

	r := math.Sqrt(x*x + y*y)
	theta := math.Atan2(y, x)

	return RangerData{
		Angle:    int32(theta * 180 / math.Pi),
		Distance: int32(r),
	}
}
