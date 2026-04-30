package slam

import (
	"math"
	"time"
)

const (
	// SubmapSwitchRadiusM is the primary submap-size knob in the dynamic-grid model.
	// Increase to keep each submap active longer; decrease to switch more often.
	SubmapSwitchRadiusM = 1.3 // metres

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
	RobotID    int
	Origin     Pose
	grid       map[CellKey]float32
	IsKeyframe bool
	CreatedAt  time.Time
}

func newSubmap(id, robotID int, origin Pose) *Submap {
	return &Submap{
		ID:         id,
		RobotID:    robotID,
		Origin:     origin,
		grid:       make(map[CellKey]float32),
		IsKeyframe: false,
		CreatedAt:  time.Now(),
	}
}

// wallConfidenceCount returns a coarser count of unique wall evidence clusters.
// It intentionally groups nearby occupied cells so one long wall does not look
// like many independent observations just because the robot moved forward.
func (s *Submap) wallConfidenceCount() int {
	const bucketSizeM = GridRes * 5
	seen := make(map[CellKey]struct{})
	for key, v := range s.grid {
		if v <= 0.5 {
			continue
		}
		bucketX := int(math.Floor(float64(key.X) * GridRes / bucketSizeM))
		bucketY := int(math.Floor(float64(key.Y) * GridRes / bucketSizeM))
		seen[CellKey{X: bucketX, Y: bucketY}] = struct{}{}
	}
	return len(seen)
}
