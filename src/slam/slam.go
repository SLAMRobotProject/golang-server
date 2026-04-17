package slam

import (
	"golang-server/types"
	"golang-server/utilities"
	util "golang-server/utilities"
	"image"
	"math"
	"time"
)

const (
	GridRes      = 0.03 // meters per cell (3 cm)
	GridWidth    = 666  // world grid cells -> ~20 m total
	GridHeight   = 666
	GridOffX     = 333 // cell index of the world origin
	GridOffY     = 333
	MaxBuildDist = 3.5 // only mark walls within sensor reliable range (m)
	// Distances very close to MaxBuildDist are treated as "no hit" rays:
	// we clear free-space along the ray but do not mark the endpoint occupied.
	RangeNoHitMargin = 0.08 // metres
	MinBuildDist     = 0.15 // ignore points closer than this (noise floor)
	trailLenM        = 3.0  // draw last 3 meters of driven path
	trailStepM       = 0.05 // add trail point every 5 cm

	// Switching to a new submap criteria:
	MaxSubmapTime      = 30 * time.Second
	MaxSubmapRotation  = 4 * math.Pi // 2 full rotations (720°)
	MinWallCellsNeeded = 850
)

var ToFSensorFOV = utilities.DegreesToRadians(27.0) // radians — total field of view of the ToF sensor

type trailPoint struct {
	x, y float64
}

type matchOverlay struct {
	points []wallCell
}

// OccupancyMap manages a set of submaps for one robot.
// A new submap is created whenever the robot moves far enough from the current submap origin.
type OccupancyMap struct {
	CurrentPose      Pose
	submaps          []*Submap
	active           *Submap
	nextID           int
	trail            []trailPoint
	trailLen         float64
	dispX, dispY     float64
	dispInit         bool
	edges            []poseEdge
	totalRotation    float64
	lastTheta        float64
	seqMatchOverlays []matchOverlay
	lcMatchOverlays  []matchOverlay
}

func (m *OccupancyMap) appendMatchOverlay(dst *[]matchOverlay, s *Submap, maxPoints, maxHistory int) {
	if s == nil {
		return
	}
	walls := collectWalls(s)
	if len(walls) == 0 {
		return
	}
	walls = downsampleWalls(walls, maxPoints)
	pts := make([]wallCell, len(walls))
	copy(pts, walls)
	*dst = append(*dst, matchOverlay{points: pts})
	if len(*dst) > maxHistory {
		*dst = (*dst)[len(*dst)-maxHistory:]
	}
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
func (m *OccupancyMap) ProcessUpdate(ekfX, ekfY, ekfTheta float64, cameraObjs types.CameraObject) (image.Image, *Pose) {
	m.CurrentPose = Pose{X: ekfX, Y: ekfY, Theta: ekfTheta}
	m.appendTrailPoint(ekfX, ekfY)

	var correction *Pose
	skipScan := false
	if m.active == nil {
		m.switchSubmap() // first submap — no predecessor to match
	} else if m.shouldSwitch(ekfX, ekfY, ekfTheta) {
		corrX, corrY, corrTheta := m.switchSubmap()

		m.totalRotation = 0
		m.lastTheta = ekfTheta + corrTheta

		if corrX != 0 || corrY != 0 || corrTheta != 0 {
			correction = &Pose{
				X:     ekfX + corrX,
				Y:     ekfY + corrY,
				Theta: util.NormalizeAngle(ekfTheta + corrTheta),
			}
			skipScan = true

			cosT := math.Cos(corrTheta)
			sinT := math.Sin(corrTheta)
			for i := range m.trail {
				dx := m.trail[i].x - ekfX
				dy := m.trail[i].y - ekfY
				m.trail[i].x = ekfX + corrX + dx*cosT - dy*sinT
				m.trail[i].y = ekfY + corrY + dx*sinT + dy*cosT
			}
		}
	}

	// EMA-smoothed display position in active submap cell space (absorbs EKF jitter)
	const alpha = 0.4
	lx, ly := m.active.Origin.GlobalToLocal(ekfX, ekfY)
	lxF := lx / GridRes
	lyF := -ly / GridRes

	if !m.dispInit {
		m.dispX, m.dispY = lxF, lyF
		m.dispInit = true
	} else {
		m.dispX = alpha*lxF + (1-alpha)*m.dispX
		m.dispY = alpha*lyF + (1-alpha)*m.dispY
	}

	if !skipScan {
		if cameraObjs.DistMM <= 0 {
			// DistMM == 0 (zero value) means no obstacle detected.
			// Clear free space up to MaxBuildDist minus the hit-band depth so the eraser
			// never reaches the blur fringe of walls sitting just at sensor range.
			m.active.ClearFOV(ekfX, ekfY, ekfTheta, MaxBuildDist-ismDepthNoise*4, ToFSensorFOV)
		} else {
			// CameraObject fields are in mm integers; convert to metres for the ISM.
			// UpdateSolidCone takes absolute lateral edges (startX, endX), not width.
			distM := float64(cameraObjs.DistMM) / 1000.0
			startXM := float64(cameraObjs.StartMM) / 1000.0
			endXM := float64(cameraObjs.StartMM+cameraObjs.WidthMM) / 1000.0
			if distM >= MinBuildDist && distM <= MaxBuildDist {
				m.active.UpdateSolidCone(ekfX, ekfY, ekfTheta, distM, startXM, endXM)
			}
		}
	}

	return m.renderLocal(m.active, m.dispX, m.dispY), correction
}

func (m *OccupancyMap) appendTrailPoint(x, y float64) {
	p := trailPoint{x: x, y: y}
	n := len(m.trail)
	if n == 0 {
		m.trail = append(m.trail, p)
		return
	}
	last := m.trail[n-1]
	seg := math.Hypot(p.x-last.x, p.y-last.y)
	if seg < trailStepM {
		return
	}
	m.trail = append(m.trail, p)
	m.trailLen += seg

	for len(m.trail) > 1 && m.trailLen > trailLenM {
		head := m.trail[0]
		next := m.trail[1]
		m.trailLen -= math.Hypot(next.x-head.x, next.y-head.y)
		m.trail = m.trail[1:]
	}
	if m.trailLen < 0 {
		m.trailLen = 0
	}
}

// RenderGlobal composites all submaps onto a single world-sized image.
func (m *OccupancyMap) RenderGlobal() image.Image {
	return m.renderGlobal(false)
}

// RenderGlobalDebug renders the global map with match overlays for debugging.
func (m *OccupancyMap) RenderGlobalDebug() image.Image {
	return m.renderGlobal(true)
}

func (m *OccupancyMap) shouldSwitch(ekfX, ekfY, ekfTheta float64) bool {
	if m.active == nil {
		return true
	}

	// 1. Traveled Distance
	dist := math.Hypot(ekfX-m.active.Origin.X, ekfY-m.active.Origin.Y)

	// 2. Time
	age := time.Since(m.active.CreatedAt)

	// 3. Rotated more than 2 times
	deltaTheta := math.Abs(util.NormalizeAngle(ekfTheta - m.lastTheta))
	m.totalRotation += deltaTheta
	m.lastTheta = ekfTheta

	// 4. Data Confidence (Filled out submap)
	// We check wallCellCount to ensure we aren't saving a submap with no data
	isConfident := m.active.wallCellCount() > MinWallCellsNeeded

	// Switch condition:
	// If we've drifted (Time/Rotation) or moved (Distance),
	// AND we have enough data to make the submap useful.
	return (dist > SwitchDist || age > MaxSubmapTime || m.totalRotation > MaxSubmapRotation) && isConfident
}

func (m *OccupancyMap) switchSubmap() (corrX, corrY, corrTheta float64) {
	if m.active != nil && m.active.wallCellCount() >= matchMinWallCells {

		// 1. Finalize current submap and add to graph
		m.submaps = append(m.submaps, m.active)
		n := len(m.submaps)
		curIdx := n - 1

		// 2. Add the sequential odometry edge (correctly connecting n-2 to n-1)
		if n >= 2 {
			m.addSequentialEdge(curIdx-1, curIdx)
		}

		// 3. Snapshot local pose BEFORE rotation
		oldOt := m.active.Origin.Theta
		dt := normalizeAngle(m.CurrentPose.Theta - oldOt)
		localX, localY := m.active.Origin.GlobalToLocal(m.CurrentPose.X, m.CurrentPose.Y)

		// 4. Run Matchers (They will now safely pull from m.submaps[n-1])
		seqDX, seqDY, seqDT := m.tryMatchToPrev()
		if seqDX != 0 || seqDY != 0 || seqDT != 0 {
			m.appendMatchOverlay(&m.seqMatchOverlays, m.active, 900, 40)
		}

		_, _, _, lcFired := m.tryLoopClosure()
		if lcFired {
			m.appendMatchOverlay(&m.lcMatchOverlays, m.active, 900, 40)
		}

		// 5. Project robot's pose back out
		newOx, newOy, newOt := m.active.Origin.X, m.active.Origin.Y, m.active.Origin.Theta
		cosNew := math.Cos(newOt)
		sinNew := math.Sin(newOt)

		correctedGlobalX := newOx + localX*cosNew - localY*sinNew
		correctedGlobalY := newOy + localX*sinNew + localY*cosNew
		correctedGlobalTheta := normalizeAngle(newOt + dt)

		corrX = correctedGlobalX - m.CurrentPose.X
		corrY = correctedGlobalY - m.CurrentPose.Y
		corrTheta = normalizeAngle(correctedGlobalTheta - m.CurrentPose.Theta)

		// Keyframe Logic
		walls := collectWalls(m.active)
		shapeRatio := scanGeometricDiversity(walls)
		isRedundant := false
		const minKeyframeDist = 1.0
		for i := 0; i < curIdx; i++ {
			s := m.submaps[i]
			if s.IsKeyframe {
				dist := math.Hypot(m.active.Origin.X-s.Origin.X, m.active.Origin.Y-s.Origin.Y)
				if dist < minKeyframeDist {
					isRedundant = true
					break
				}
			}
		}
		if shapeRatio > 0.15 && !isRedundant {
			m.active.IsKeyframe = true
		}
	}

	// Start next submap
	correctedPose := Pose{
		X:     m.CurrentPose.X + corrX,
		Y:     m.CurrentPose.Y + corrY,
		Theta: normalizeAngle(m.CurrentPose.Theta + corrTheta),
	}
	s := newSubmap(m.nextID, correctedPose)
	m.nextID++

	m.active = s
	m.dispInit = false
	return
}
