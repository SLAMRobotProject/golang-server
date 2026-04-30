package slam

import (
	"fmt"
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
	MinWallCellsNeeded = 75

	// Keyframe radius for loop closure pruning (m):
	MinKeyframeDist = 1.0
)

var ToFSensorFOV = utilities.DegreesToRadians(27.0) // radians — total field of view of the ToF sensor

type trailPoint struct {
	x, y float64
}

type matchOverlay struct {
	points []wallCell
}

// robotLocalState holds per-robot mutable SLAM state within the global map.
type robotLocalState struct {
	CurrentPose      Pose
	trail            []trailPoint
	trailLen         float64
	dispX, dispY     float64
	dispInit         bool
	totalRotation    float64
	lastTheta        float64
	seqMatchOverlays []matchOverlay
	lcMatchOverlays  []matchOverlay
}

// OccupancyMap is the global multi-robot SLAM map.
// A single instance is shared by all robots; per-robot state is keyed by robot ID.
// submaps[0] is always the world anchor (Robot 0's first finalized submap) and
// is held fixed by the Ceres optimizer.
type OccupancyMap struct {
	submaps            []*Submap              // all finalized submaps, all robots, chronological
	activeSubmaps      map[int]*Submap        // robotID -> currently active (being built) submap
	robots             map[int]*robotLocalState
	nextID             int
	edges              []poseEdge
	lastLCAttempt      int
	pendingCorrections map[int]Pose // robotID -> absolute corrected pose from cross-robot LC
}

func NewOccupancyMap() *OccupancyMap {
	return &OccupancyMap{
		activeSubmaps:      make(map[int]*Submap),
		robots:             make(map[int]*robotLocalState),
		lastLCAttempt:      -loopMinSubmapGap,
		pendingCorrections: make(map[int]Pose),
	}
}

func (m *OccupancyMap) ensureRobot(robotID int) *robotLocalState {
	rs := m.robots[robotID]
	if rs == nil {
		rs = &robotLocalState{}
		m.robots[robotID] = rs
	}
	return rs
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

// SubmapCount returns the total number of finalized submaps across all robots.
func (m *OccupancyMap) SubmapCount() int {
	return len(m.submaps)
}

// SubmapCountPerRobot returns finalized submap counts keyed by robot ID.
// Active (in-progress) submaps are not counted.
func (m *OccupancyMap) SubmapCountPerRobot() map[int]int {
	counts := make(map[int]int, len(m.robots))
	for _, s := range m.submaps {
		counts[s.RobotID]++
	}
	// Ensure every registered robot has an entry even if zero
	for id := range m.robots {
		if _, ok := counts[id]; !ok {
			counts[id] = 0
		}
	}
	return counts
}

// ActiveSubmapID returns the ID of the currently active submap for robotID, or -1 if none.
func (m *OccupancyMap) ActiveSubmapID(robotID int) int {
	s := m.activeSubmaps[robotID]
	if s == nil {
		return -1
	}
	return s.ID
}

// ProcessUpdate ingests one EKF pose + one scan frame for a specific robot.
// Returns the rendered local map and a map of absolute pose corrections keyed by
// robot ID. Corrections are non-nil when a submap switch with scan matching
// produced a pose change; cross-robot loop closures may correct other robots too.
func (m *OccupancyMap) ProcessUpdate(robotID int, ekfX, ekfY, ekfTheta float64, cameraObjs types.CameraObject) (image.Image, map[int]*Pose) {
	rs := m.ensureRobot(robotID)
	rs.CurrentPose = Pose{X: ekfX, Y: ekfY, Theta: ekfTheta}
	m.appendTrailPoint(robotID, ekfX, ekfY)

	corrections := make(map[int]*Pose)

	// Consume any pending cross-robot loop-closure correction for this robot.
	if pending, ok := m.pendingCorrections[robotID]; ok {
		corrections[robotID] = &Pose{X: pending.X, Y: pending.Y, Theta: pending.Theta}
		delete(m.pendingCorrections, robotID)
	}

	skipScan := false
	if m.activeSubmaps[robotID] == nil {
		m.switchSubmap(robotID)
	} else if m.shouldSwitch(robotID, ekfX, ekfY, ekfTheta) {
		corrX, corrY, corrTheta := m.switchSubmap(robotID)

		rs.totalRotation = 0
		rs.lastTheta = ekfTheta + corrTheta

		if corrX != 0 || corrY != 0 || corrTheta != 0 {
			// This correction overrides any pending one — it is computed fresh.
			corrections[robotID] = &Pose{
				X:     ekfX + corrX,
				Y:     ekfY + corrY,
				Theta: util.NormalizeAngle(ekfTheta + corrTheta),
			}
			skipScan = true

			cosT := math.Cos(corrTheta)
			sinT := math.Sin(corrTheta)
			for i := range rs.trail {
				dx := rs.trail[i].x - ekfX
				dy := rs.trail[i].y - ekfY
				rs.trail[i].x = ekfX + corrX + dx*cosT - dy*sinT
				rs.trail[i].y = ekfY + corrY + dx*sinT + dy*cosT
			}
		}
	}

	active := m.activeSubmaps[robotID]

	// EMA-smoothed display position in active submap cell space (absorbs EKF jitter).
	const alpha = 0.4
	lx, ly := active.Origin.GlobalToLocal(ekfX, ekfY)
	lxF := lx / GridRes
	lyF := -ly / GridRes

	if !rs.dispInit {
		rs.dispX, rs.dispY = lxF, lyF
		rs.dispInit = true
	} else {
		rs.dispX = alpha*lxF + (1-alpha)*rs.dispX
		rs.dispY = alpha*lyF + (1-alpha)*rs.dispY
	}

	if !skipScan {
		if cameraObjs.DistMM <= 0 {
			active.ClearFOV(ekfX, ekfY, ekfTheta, MaxBuildDist-ismDepthNoise*4, ToFSensorFOV)
		} else {
			distM := float64(cameraObjs.DistMM) / 1000.0
			startXM := float64(cameraObjs.StartMM) / 1000.0
			endXM := float64(cameraObjs.StartMM+cameraObjs.WidthMM) / 1000.0
			if distM >= MinBuildDist && distM <= MaxBuildDist {
				active.UpdateSolidCone(ekfX, ekfY, ekfTheta, distM, startXM, endXM)
			}
		}
	}

	return m.renderLocal(active, rs.dispX, rs.dispY, rs.CurrentPose.Theta), corrections
}

func (m *OccupancyMap) appendTrailPoint(robotID int, x, y float64) {
	rs := m.ensureRobot(robotID)
	p := trailPoint{x: x, y: y}
	n := len(rs.trail)
	if n == 0 {
		rs.trail = append(rs.trail, p)
		return
	}
	last := rs.trail[n-1]
	seg := math.Hypot(p.x-last.x, p.y-last.y)
	if seg < trailStepM {
		return
	}
	rs.trail = append(rs.trail, p)
	rs.trailLen += seg

	for len(rs.trail) > 1 && rs.trailLen > trailLenM {
		head := rs.trail[0]
		next := rs.trail[1]
		rs.trailLen -= math.Hypot(next.x-head.x, next.y-head.y)
		rs.trail = rs.trail[1:]
	}
	if rs.trailLen < 0 {
		rs.trailLen = 0
	}
}

// RenderGlobal composites all submaps from all robots onto a single world-sized image.
func (m *OccupancyMap) RenderGlobal() image.Image {
	return m.renderGlobal(false)
}

// RenderGlobalDebug renders the global multi-robot map with match overlays.
func (m *OccupancyMap) RenderGlobalDebug() image.Image {
	return m.renderGlobal(true)
}

func (m *OccupancyMap) shouldSwitch(robotID int, ekfX, ekfY, ekfTheta float64) bool {
	active := m.activeSubmaps[robotID]
	if active == nil {
		return true
	}
	rs := m.robots[robotID]

	// 1. Traveled distance from submap origin
	dist := math.Hypot(ekfX-active.Origin.X, ekfY-active.Origin.Y)

	// 2. Time alive
	age := time.Since(active.CreatedAt)

	// 3. Cumulative rotation
	deltaTheta := math.Abs(util.NormalizeAngle(ekfTheta - rs.lastTheta))
	rs.totalRotation += deltaTheta
	rs.lastTheta = ekfTheta

	// 4. Data confidence (unique wall clusters)
	isConfident := active.wallConfidenceCount() > MinWallCellsNeeded

	switchTriggered := age > MaxSubmapTime || rs.totalRotation > MaxSubmapRotation
	if dist > SwitchDist {
		if !isConfident {
			return false
		}
		switchTriggered = true
	}
	return switchTriggered
}

func (m *OccupancyMap) switchSubmap(robotID int) (corrX, corrY, corrTheta float64) {
	rs := m.ensureRobot(robotID)
	active := m.activeSubmaps[robotID]

	if active != nil && active.wallConfidenceCount() >= MinWallCellsNeeded {
		fmt.Printf("[SLAM R%d] finalize submap %d: wallCells=%d\n",
			robotID, active.ID, active.wallConfidenceCount())

		// 1. Finalize: push to global list
		m.submaps = append(m.submaps, active)
		n := len(m.submaps)
		curIdx := n - 1

		// 2. Add sequential odometry edge to the previous same-robot submap
		prevIdx := -1
		for i := curIdx - 1; i >= 0; i-- {
			if m.submaps[i].RobotID == robotID {
				prevIdx = i
				break
			}
		}
		if prevIdx >= 0 {
			m.addSequentialEdge(prevIdx, curIdx)
		}

		// 3. Snapshot robot's local position BEFORE matching modifies the origin
		oldOt := active.Origin.Theta
		dt := normalizeAngle(rs.CurrentPose.Theta - oldOt)
		localX, localY := active.Origin.GlobalToLocal(rs.CurrentPose.X, rs.CurrentPose.Y)

		// 4. Run matchers
		seqDX, seqDY, seqDT := m.tryMatchToPrev(robotID)
		if seqDX != 0 || seqDY != 0 || seqDT != 0 {
			m.appendMatchOverlay(&rs.seqMatchOverlays, active, 900, 40)
		}

		_, _, _, lcFired := m.tryLoopClosure(robotID)
		if lcFired {
			m.appendMatchOverlay(&rs.lcMatchOverlays, active, 900, 40)
		}

		// 5. Re-project robot position using the (possibly corrected) origin
		newOx, newOy, newOt := active.Origin.X, active.Origin.Y, active.Origin.Theta
		cosNew := math.Cos(newOt)
		sinNew := math.Sin(newOt)

		correctedGlobalX := newOx + localX*cosNew - localY*sinNew
		correctedGlobalY := newOy + localX*sinNew + localY*cosNew
		correctedGlobalTheta := normalizeAngle(newOt + dt)

		corrX = correctedGlobalX - rs.CurrentPose.X
		corrY = correctedGlobalY - rs.CurrentPose.Y
		corrTheta = normalizeAngle(correctedGlobalTheta - rs.CurrentPose.Theta)

		// Keyframe decision
		walls := collectWalls(active)
		shapeRatio := scanGeometricDiversity(walls)
		isRedundant := false
		for i := 0; i < curIdx; i++ {
			s := m.submaps[i]
			if s.IsKeyframe {
				dist := math.Hypot(active.Origin.X-s.Origin.X, active.Origin.Y-s.Origin.Y)
				if dist < MinKeyframeDist {
					isRedundant = true
					break
				}
			}
		}
		if shapeRatio > 0.15 && !isRedundant {
			active.IsKeyframe = true
		}
	}

	// Start next submap anchored at the corrected current pose
	correctedPose := Pose{
		X:     rs.CurrentPose.X + corrX,
		Y:     rs.CurrentPose.Y + corrY,
		Theta: normalizeAngle(rs.CurrentPose.Theta + corrTheta),
	}
	s := newSubmap(m.nextID, robotID, correctedPose)
	m.nextID++

	m.activeSubmaps[robotID] = s
	rs.dispInit = false
	return
}
