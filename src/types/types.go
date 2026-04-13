package types

import "image"

//This package contains types that are used by multiple packages.
//Generally they are used by channels to communicate between packages.

type AdvMsg struct {
	Id, X, Y, Theta, Ir1x, Ir1y, Ir2x, Ir2y, Ir3x, Ir3y, Ir4x, Ir4y int
}

// CameraMsg represents a camera line segment reported by a camera module.
type CameraMsg struct {
	Id         int
	IsViritual bool // True = Digital Tvilling

	// Visualisering (Endepunkter i mm)
	P1X, P1Y int
	P2X, P2Y int

	// SLAM (Hesses normalform)
	RhoMM     int // Avstand fra robot til linje (mm)
	AlphaMRad int // Vinkel på normalen (milliradianer, for å bruke int)

	// Fall-back
	StartMM    int
	WidthMM    int
	DistanceMM int
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
	MultiRobot      []RobotState
	Id2index        map[int]int
	NewOpen         [][2]int
	NewObstacle     [][2]int
	Lines           [][2]Point
	SlamMapImgs     map[int]image.Image // per-robot SLAM local map, keyed by robot ID
	SlamGlobalImgs  map[int]image.Image // per-robot SLAM global composite, keyed by robot ID
	SlamSubmapCount map[int]int         // number of submaps per robot
}

type Position struct {
	X     int
	Y     int
	Theta int
}

type SlamData struct {
	RobotID  int
	Position Position
}

type Point struct {
	X int
	Y int
}

type MapLine struct {
	Rho   float64 // Distance to origin
	Alpha float64 // Angle of normal vector

	// Confidence (Lower Variance = Higher Confidence)
	VarRho   float64
	VarAlpha float64

	// Physical extent of the wall
	P1_X, P1_Y float64
	P2_X, P2_Y float64

	Existence float64 // How well do we know this wall?
	Count     int     // How many times have we seen this wall?
}

type PoseUpdateMsg struct {
	Id    int
	X     float64
	Y     float64
	Theta float64
}
