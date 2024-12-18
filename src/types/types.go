package types

//This package contains types that are used by multiple packages.
//Generally they are used by channels to communicate between packages.

type AdvMsg struct {
	Id, X, Y, Theta, Ir1x, Ir1y, Ir2x, Ir2y, Ir3x, Ir3y, Ir4x, Ir4y int
}

type RectangleMsg struct{
	Id		int
	TotalMap bool
	X,Y,Width,Height,Obstacle int
	Reachable bool
}

type MapRectangle struct{
	X,Y,Width,Height int
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
