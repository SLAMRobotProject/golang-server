package types

//This package contains types that are used by multiple packages.
//Generally they are used by channels to communicate between packages.

type AdvMsg struct {
	Id, X, Y, Theta, Ir1x, Ir1y, Ir2x, Ir2y, Ir3x, Ir3y, Ir4x, Ir4y int
}

const (
	AutomaticCommand = iota
	ManualCommand
)

type Command struct {
	CommandType int //E.g. AutomaticCommand
	Id, X, Y    int
}

type RobotState struct {
	X, Y, Theta             int //cm, degrees
	XInit, YInit, ThetaInit int
}

type UpdateGui struct {
	MultiRobot  []RobotState
	Id2index    map[int]int
	NewOpen     [][2]int
	NewObstacle [][2]int
}

// SLAM related types

type Odometry struct {
	X     int32 `json:"x"`
	Y     int32 `json:"y"`
	Z     int32 `json:"z"`
	Theta int32 `json:"theta"`
}

type Imu struct {
	AccelX int32 `json:"accel_x"`
	AccelY int32 `json:"accel_y"`
	AccelZ int32 `json:"accel_z"`
	GyroX  int32 `json:"gyro_x"`
	GyroY  int32 `json:"gyro_y"`
	GyroZ  int32 `json:"gyro_z"`
}

type IRPoint struct {
	X int32 `json:"x"`
	Y int32 `json:"y"`
}

type IRSensors struct {
	IrSensorData_0 []IRPoint `json:"ir_0"`
	IrSensorData_1 []IRPoint `json:"ir_1"`
	IrSensorData_2 []IRPoint `json:"ir_2"`
	IrSensorData_3 []IRPoint `json:"ir_3"`
}

type SensorDataMsg struct {
	Id        int32     `json:"id"`
	IRSensors IRSensors `json:"ir_sensors"`
	Odometry  Odometry  `json:"odometry"`
	Imu       Imu       `json:"imu"`
}
