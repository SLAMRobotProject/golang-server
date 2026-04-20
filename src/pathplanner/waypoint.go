package pathplanner

import "strconv"

// Waypoint is a single XY target in centimeters.
type Waypoint struct {
	X int
	Y int
}

func NewWaypoint(x, y int) Waypoint {
	return Waypoint{X: x, Y: y}
}

func ParseWaypoint(xText, yText string) (Waypoint, error) {
	x, errX := strconv.Atoi(xText)
	if errX != nil {
		return Waypoint{}, errX
	}
	y, errY := strconv.Atoi(yText)
	if errY != nil {
		return Waypoint{}, errY
	}
	return Waypoint{X: x, Y: y}, nil
}
