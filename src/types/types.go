package types

import "image"

//This package contains types that are used by multiple packages.
//Generally they are used by channels to communicate between packages.

type AdvMsg struct {
	Id, X, Y, Theta, Ir1x, Ir1y, Ir2x, Ir2y, Ir3x, Ir3y, Ir4x, Ir4y int
}

type CameraObject struct {
	DistMM  int
	StartMM int
	WidthMM int
}

// CameraMsg represents a camera line segment reported by a camera module.
type CameraMsg struct {
	Id         int
	IsViritual bool   // True = Digital Tvilling
	Source     string // sensor source identifier, e.g. camera, lidar-tower

	Obj CameraObject
}

const (
	AutomaticCommand = iota
	ManualCommand
)

type Command struct {
	Id, X, Y    int
	CommandType int
}

type RobotState struct {
	X, Y, Theta             int //cm, degrees
	XInit, YInit, ThetaInit int
}

type Point struct {
	X, Y int
}

type UpdateGui struct {
	MultiRobot        []RobotState
	Id2index          map[int]int
	NewOpen           [][2]int
	NewObstacle       [][2]int
	Lines             [][2]Point
	SlamMapImgs       map[int]image.Image // per-robot SLAM local map, keyed by robot ID
	SlamGlobalImgs    map[int]image.Image // per-robot SLAM global composite, keyed by robot ID
	SlamGlobalDbgImgs map[int]image.Image // per-robot SLAM global map with debug match overlays
	SlamSubmapCount   map[int]int         // number of submaps per robot
}

type SlamData struct {
	RobotID  int
	Position Pose2D
}

type PoseUpdateMsg struct {
	Id    int
	X     float64
	Y     float64
	Theta float64
}

// Pose2D represents a planar pose (metres/radians).
type Pose2D struct {
	X, Y, Theta float64
}

// RobotCommand is the backend->communication command payload for robot control.
type RobotCommand struct {
	RobotID    int
	TargetPose Pose2D
}
