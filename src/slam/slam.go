package slam

import (
	"image"
	"math"
)

const (
	GridRes      = 0.05 // meters per cell (5 cm)
	GridWidth    = 400  // world grid cells → 20 m total
	GridHeight   = 400
	GridOffX     = 200  // cell index of the world origin
	GridOffY     = 200
	MaxBuildDist = 3.5  // only mark walls within sensor reliable range (m)
	MinBuildDist = 0.15 // ignore points closer than this (noise floor)
)

// OccupancyMap manages a set of submaps for one robot.
// A new submap is created whenever the robot moves far enough from the current submap origin.
type OccupancyMap struct {
	CurrentPose Pose
	submaps     []*Submap
	active      *Submap
	nextID      int
	// Smoothed display position within the active submap (EMA, in submap cell coords).
	// Prevents jitter from EKF noise snapping the crop window by 1 cell.
	dispX, dispY float64
	dispInit     bool
}

func NewOccupancyMap() *OccupancyMap {
	return &OccupancyMap{}
}

// SubmapCount returns the total number of submaps created so far.
func (m *OccupancyMap) SubmapCount() int {
	return len(m.submaps)
}

// ActiveSubmapID returns the ID of the currently active submap, or -1 if none.
func (m *OccupancyMap) ActiveSubmapID() int {
	if m.active == nil {
		return -1
	}
	return m.active.ID
}

// ProcessUpdate ingests one EKF pose + one scan frame.
// Returns the rendered local map and, when a submap switch triggered a match,
// a non-nil *Pose holding the corrected absolute robot pose (metres, radians)
// to be forwarded to the robot's EKF.
func (m *OccupancyMap) ProcessUpdate(ekfX, ekfY, ekfTheta float64, scan [][2]float64) (image.Image, *Pose) {
	m.CurrentPose = Pose{X: ekfX, Y: ekfY, Theta: ekfTheta}

	var correction *Pose
	if m.active == nil {
		m.switchSubmap() // first submap — no predecessor to match
	} else {
		dx := ekfX - m.active.Origin.X
		dy := ekfY - m.active.Origin.Y
		if math.Hypot(dx, dy) > SwitchDist {
			corrX, corrY, corrTheta := m.switchSubmap()
			if corrX != 0 || corrY != 0 || corrTheta != 0 {
				correction = &Pose{
					X:     ekfX + corrX,
					Y:     ekfY + corrY,
					Theta: ekfTheta + corrTheta,
				}
			}
		}
	}

	// EMA-smoothed display position in active submap cell space (absorbs EKF jitter)
	const alpha = 0.4
	ox, oy := m.active.Origin.X, m.active.Origin.Y
	lxF := (ekfX-ox)/GridRes + SubHalf
	lyF := float64(SubHalf) - (ekfY-oy)/GridRes
	if !m.dispInit {
		m.dispX, m.dispY = lxF, lyF
		m.dispInit = true
	} else {
		m.dispX = alpha*lxF + (1-alpha)*m.dispX
		m.dispY = alpha*lyF + (1-alpha)*m.dispY
	}

	// Raytrace each scan ray into the active submap
	for _, pt := range scan {
		angle, dist := pt[0], pt[1]
		if dist < MinBuildDist || dist > MaxBuildDist {
			continue
		}
		globalAngle := ekfTheta + angle
		hitX := ekfX + dist*math.Cos(globalAngle)
		hitY := ekfY + dist*math.Sin(globalAngle)
		m.active.update(ekfX, ekfY, hitX, hitY, dist)
	}

	return m.renderLocal(m.active, m.dispX, m.dispY), correction
}

// RenderGlobal composites all submaps onto a single world-sized image.
func (m *OccupancyMap) RenderGlobal() image.Image {
	return m.renderGlobal()
}

// switchSubmap seals the current submap and starts a fresh one.
// Runs sequential matching then loop closure on the outgoing submap (maximum data).
// Returns the total pose correction (dx, dy, dTheta) so the caller can forward
// it to the robot's EKF.
func (m *OccupancyMap) switchSubmap() (corrX, corrY, corrTheta float64) {
	if m.active != nil && m.active.wallCellCount() >= matchMinWallCells {
		dx1, dy1, dt1 := m.tryMatchToPrev()
		dx2, dy2, dt2 := m.tryLoopClosure()
		corrX = dx1 + dx2
		corrY = dy1 + dy2
		corrTheta = dt1 + dt2
	}

	// Start next submap at the corrected current pose so mapping continues
	// from the right position even before the robot receives the update.
	correctedPose := Pose{
		X:     m.CurrentPose.X + corrX,
		Y:     m.CurrentPose.Y + corrY,
		Theta: m.CurrentPose.Theta + corrTheta,
	}
	s := newSubmap(m.nextID, correctedPose)
	m.nextID++
	m.submaps = append(m.submaps, s)
	m.active = s
	m.dispInit = false
	return
}
