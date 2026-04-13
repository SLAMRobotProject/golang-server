package slam

import (
	"golang-server/utilities"
	"math"
)

const (
	SubHalf    = 50          // ±2.5 m radius in cells
	SubSize    = 2 * SubHalf // 100 cells = 5 m x 5 m
	SwitchDist = 1.2         // start new submap when robot is > 1.2 m from submap origin
)

// Submap is a local occupancy grid anchored at a fixed world pose.
type Submap struct {
	ID     int
	Origin Pose
	grid   [SubSize][SubSize]float32
}

func newSubmap(id int, origin Pose) *Submap {
	return &Submap{ID: id, Origin: origin}
}

// update performs Bresenham raytracing in submap-local coordinates.
// robotX/Y is the robot world position; hitX/Y is the measured wall point.
func (s *Submap) update(robotX, robotY, hitX, hitY, dist float64) {
	ox, oy := s.Origin.X, s.Origin.Y

	// Robot cell in submap frame
	rx := int(math.Round((robotX-ox)/GridRes)) + SubHalf
	ry := SubHalf - int(math.Round((robotY-oy)/GridRes))

	// Hit cell in submap frame
	hx := int(math.Round((hitX-ox)/GridRes)) + SubHalf
	hy := SubHalf - int(math.Round((hitY-oy)/GridRes))

	cells := utilities.BresenhamAlgorithm(rx, ry, hx, hy)
	for i, cell := range cells {
		cx, cy := cell[0], cell[1]
		if cx < 0 || cx >= SubSize || cy < 0 || cy >= SubSize {
			continue
		}
		if i == len(cells)-1 && dist <= MaxBuildDist {
			weight := float32(math.Exp(-dist / 2.0))
			s.clampSet(cx, cy, s.grid[cy][cx]+0.5*weight, 5.0)
		} else {
			// Don't erase cells already established as walls.
			if s.grid[cy][cx] < 1.5 {
				s.clampSet(cx, cy, s.grid[cy][cx]-0.15, -5.0)
			}
		}
	}
}

// wallCellCount returns the number of cells with confirmed wall occupancy.
func (s *Submap) wallCellCount() int {
	n := 0
	for cy := 0; cy < SubSize; cy++ {
		for cx := 0; cx < SubSize; cx++ {
			if s.grid[cy][cx] > 0.5 {
				n++
			}
		}
	}
	return n
}

func (s *Submap) clampSet(x, y int, val, limit float32) {
	if limit > 0 && val > limit {
		val = limit
	} else if limit < 0 && val < limit {
		val = limit
	}
	s.grid[y][x] = val
}
