package slam

// Submap represents a small local occupancy grid attached to a specific pose
type Submap struct {
	ID         int
	OriginPose Pose
	Grid       []float64
	GridSize   int
	NumScans   int
	Finished   bool
}

func NewSubmap(id int, origin Pose, size int) *Submap {
	return &Submap{
		ID:         id,
		OriginPose: origin,
		Grid:       make([]float64, size*size),
		GridSize:   size,
		NumScans:   0,
		Finished:   false,
	}
}

// Transform global position to local submap index
func (sm *Submap) globalPosToLocalIdx(globalX, globalY float64, res float64) (int, int, bool) {
	// Offset relative to submap origin
	dx := globalX - sm.OriginPose.X
	dy := globalY - sm.OriginPose.Y

	// We align the submap grid with global axes, so no rotation needed for indices.
	// Center of local grid is GridSize/2
	ix := int(dx/res) + sm.GridSize/2
	iy := sm.GridSize/2 - int(dy/res)

	if ix >= 0 && ix < sm.GridSize && iy >= 0 && iy < sm.GridSize {
		return ix, iy, true
	}
	return 0, 0, false
}
