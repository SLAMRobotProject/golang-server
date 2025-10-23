package slam

// This package contains code to process SLAM messages from robots.
// It converts MQTT messages to Cartographer format and processes them.
// It maintains the SLAM state and updates the map accordingly.

import (
	T "golang-server/types"
	"math"
	"runtime"
)

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

func sensorTo2DCartoMsg(sensorData T.SensorDataMsg) (CartoSlamMsg, error) {
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

	// IR sensor data
	for _, irPoint := range sensorData.IRSensors.IrSensorData_0 {
		cartoMsg.Ranger.RangerData = append(cartoMsg.Ranger.RangerData, RangerData{
			Angle:    irPoint.X,
			Distance: irPoint.Y,
		})
	}
	for _, irPoint := range sensorData.IRSensors.IrSensorData_1 {
		cartoMsg.Ranger.RangerData = append(cartoMsg.Ranger.RangerData, RangerData{
			Angle:    irPoint.X,
			Distance: irPoint.Y,
		})
	}
	for _, irPoint := range sensorData.IRSensors.IrSensorData_2 {
		cartoMsg.Ranger.RangerData = append(cartoMsg.Ranger.RangerData, RangerData{
			Angle:    irPoint.X,
			Distance: irPoint.Y,
		})
	}
	for _, irPoint := range sensorData.IRSensors.IrSensorData_3 {
		cartoMsg.Ranger.RangerData = append(cartoMsg.Ranger.RangerData, cartesianToPolar(irPoint))
	}
	return cartoMsg, nil
}

func isFullRotation(rangerData []RangerData) bool {
	if len(rangerData) < 180 {
		return false
	}
	return true
}

func sendCartoMsg(cartoMsg CartoSlamMsg) {
	// Send the CartoSlamMsg to the Cartographer
}

func slamBackendLoop(
	chSensorData <-chan T.SensorDataMsg,
) {
	// Initialize SLAM state
	// var slamState SlamState

	for {
		select {
		case newData := <-chSensorData:
			cartoMsg, err := sensorTo2DCartoMsg(newData)
			if err != nil {
				continue
			}

			// Check for full rotation (ir tower)
			if isFullRotation(cartoMsg.Ranger.RangerData) {
				// Send IR data to Cartographer
			}

			// Send odometry and IMU data to Cartographer
			sendCartoMsg(cartoMsg)

		default:
			// No new data
			// Release task to avoid busy waiting
			runtime.Gosched()

		}

		// Send
	}
}

func cartesianToPolar(point T.IRPoint) RangerData {
	x := float64(point.X)
	y := float64(point.Y)

	r := math.Sqrt(x*x + y*y)
	theta := math.Atan2(y, x)

	return RangerData{
		Angle:    int32(theta * 180 / math.Pi),
		Distance: int32(r),
	}
}
