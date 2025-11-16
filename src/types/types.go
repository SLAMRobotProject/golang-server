package types

//This package contains types that are used by multiple packages.
//Generally they are used by channels to communicate between packages.

type AdvMsg struct {
	Id                       int
	X                        int
	Y                        int
	Theta                    int
	Ir1x                     int
	Ir1y                     int
	Ir2x                     int
	Ir2y                     int
	Ir3x                     int
	Ir3y                     int
	Ir4x                     int
	Ir4y                     int
	Valid                    uint8
	CovarianceMatrixNumber1  float32
	CovarianceMatrixNumber2  float32
	CovarianceMatrixNumber3  float32
	CovarianceMatrixNumber4  float32
	CovarianceMatrixNumber5  float32
	CovarianceMatrixNumber6  float32
	CovarianceMatrixNumber7  float32
	CovarianceMatrixNumber8  float32
	CovarianceMatrixNumber9  float32
	CovarianceMatrixNumber10 float32
	CovarianceMatrixNumber11 float32
	CovarianceMatrixNumber12 float32
	CovarianceMatrixNumber13 float32
	CovarianceMatrixNumber14 float32
	CovarianceMatrixNumber15 float32
	CovarianceMatrixNumber16 float32
	CovarianceMatrixNumber17 float32
	CovarianceMatrixNumber18 float32
	CovarianceMatrixNumber19 float32
	CovarianceMatrixNumber20 float32
	CovarianceMatrixNumber21 float32
	CovarianceMatrixNumber22 float32
	CovarianceMatrixNumber23 float32
	CovarianceMatrixNumber24 float32
	CovarianceMatrixNumber25 float32
	// Nicla vision camera fields (present only when camera detected)
	CameraStartMM    int
	CameraWidthMM    int
	CameraDistanceMM int
	CameraPresent    bool
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
