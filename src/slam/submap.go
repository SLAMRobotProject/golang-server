package slam

import (
	"time"
)

const (
	// SubmapSwitchRadiusM is the primary submap-size knob in the dynamic-grid model.
	// Increase to keep each submap active longer; decrease to switch more often.
	SubmapSwitchRadiusM = 2.0 // metres

	// LocalViewHalfWidthM controls the local map viewport half-width around the robot.
	// Kept here so submap behavior and local rendering scale are easy to tune together.
	LocalViewHalfWidthM = 3.0 // metres
)

const SwitchDist = SubmapSwitchRadiusM // metres — distance threshold for switching to a new submap

// CellKey uniquely identifies a grid cell within a dynamic submap.
// (0,0) is the exact origin of the submap.
type CellKey struct {
	X, Y int
}

// Submap is a local occupancy grid anchored at a fixed world pose.
// It uses a dynamic map, meaning it has infinite bounds and uses memory
// only for space it has actually observed.
type Submap struct {
	ID         int
	Origin     Pose
	grid       map[CellKey]float32
	IsKeyframe bool
	CreatedAt  time.Time
}

func newSubmap(id int, origin Pose) *Submap {
	return &Submap{
		ID:         id,
		Origin:     origin,
		grid:       make(map[CellKey]float32),
		IsKeyframe: false,
		CreatedAt:  time.Now(),
	}
}

// wallCellCount returns the number of cells with confirmed wall occupancy.
func (s *Submap) wallCellCount() int {
	n := 0
	for _, v := range s.grid {
		if v > 0.5 {
			n++
		}
	}
	return n
}
