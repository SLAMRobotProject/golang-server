package pathplanner

func SquareTestPath() Path {
	return NewPath(
		NewWaypoint(0, 100), NewWaypoint(100, 100), NewWaypoint(100, 0), NewWaypoint(0, 0),
		NewWaypoint(100, 0), NewWaypoint(100, 100), NewWaypoint(0, 100), NewWaypoint(0, 0),
	)
}

func VirtualDefaultPath() Path {
	return NewPath(
		NewWaypoint(-100, -100),
		NewWaypoint(-100, 0),
		NewWaypoint(100, 0),
		NewWaypoint(100, -100),
		NewWaypoint(100, 100),
		NewWaypoint(-100, 100),
	)
}
