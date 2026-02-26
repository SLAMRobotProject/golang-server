package slam

import (
	"golang-server/types"
	"math"
)

type MapLine struct {
	types.MapLine
}

// SlamMap manages the collection of all walls
type Map struct {
	Lines []*MapLine
}

func NewMap() *Map {
	return &Map{
		Lines: []*MapLine{},
	}
}

// func ThreadSlam(chReceive <-chan types.AdvMsg, chCamera <-chan types.CameraMsg, chSlamData chan<- types.SlamData) {
// 	// Initialize Map
// 	slamMap := NewMap()
// 	for {
// 		select {
// 		case camMsg := <-chCamera:
// 			// Process Camera Message
// 			p1x := float64(camMsg.P1X) / 10.0
// 			p1y := float64(camMsg.P1Y) / 10.0
// 			p2x := float64(camMsg.P2X) / 10.0
// 			p2y := float64(camMsg.P2Y) / 10.0

// 			slamMap.ProcessLine(p1x, p1y, p2x, p2y)
// 		case advMsg := <-chReceive:
// 			// Process Advanced Message (Robot Position)
// 			if chSlamData != nil {
// 				chSlamData <- types.SlamData{
// 					RobotID: advMsg.Id,
// 					Position: types.Position{
// 						X:     advMsg.X,
// 						Y:     advMsg.Y,
// 						Theta: advMsg.Theta,
// 					},
// 				}
// 			}
// 		}
// 	}
// }

func (m *Map) ProcessLine(robotX, robotY, p1x, p1y, p2x, p2y float64) {
	// 0. FREE SPACE CHECK: Remove walls that contradict this observation
	m.CheckFreeSpace(robotX, robotY, p1x, p1y) // Check left endpoint ray
	m.CheckFreeSpace(robotX, robotY, p2x, p2y) // Check right endpoint ray

	// 1. Convert Cartesian endpoints to Polar (Rho/Alpha) for math matching
	newRho, newAlpha := pointsToLineParams(p1x, p1y, p2x, p2y)

	// Thresholds for matching (Is this the same wall?)
	const MatchDistThresh = 20.0  // 20cm (Assuming units are cm)
	const MatchAngleThresh = 0.26 // ~15 degrees in radians

	bestMatchIdx := -1
	bestMatchScore := 1000000.0

	// 2. Data Association: Check against all known lines
	for i, line := range m.Lines {
		// Check Angle Difference (Handle wrap-around)
		angleDiff := math.Abs(newAlpha - line.Alpha)
		if angleDiff > math.Pi {
			angleDiff = 2*math.Pi - angleDiff
		}

		// Check Distance Difference
		rhoDiff := math.Abs(newRho - line.Rho)

		// If it's close enough in both Angle and Distance...
		if angleDiff < MatchAngleThresh && rhoDiff < MatchDistThresh {
			// Simple scoring: sum of errors
			score := angleDiff + rhoDiff
			if score < bestMatchScore {
				bestMatchScore = score
				bestMatchIdx = i
			}
		}
	}

	// 3. Update or Add
	if bestMatchIdx != -1 {
		// MATCH FOUND: Merge and improve the existing line
		m.Lines[bestMatchIdx].update(newRho, newAlpha, p1x, p1y, p2x, p2y)

		// Increase confidence
		m.Lines[bestMatchIdx].Score += 5.0
		if m.Lines[bestMatchIdx].Score > 100.0 {
			m.Lines[bestMatchIdx].Score = 100.0
		}
	} else {
		// NO MATCH: Add as a new wall
		newLine := &MapLine{
			MapLine: types.MapLine{
				Rho:      newRho,
				Alpha:    newAlpha,
				VarRho:   5.0,
				VarAlpha: 1.0,
				P1_X:     p1x, P1_Y: p1y, P2_X: p2x, P2_Y: p2y,
				Score: 10.0,
				Count: 1,
			},
		}
		m.Lines = append(m.Lines, newLine)
	}

	m.OptimizeMap()
}

func (m *MapLine) update(newRho, newAlpha, p1x, p1y, p2x, p2y float64) {

	// 1. Define Sensor Uncertainty (The "Unknown" sensor noise)
	// You tune these values!
	// If the camera is noisy, make these HIGH.
	const SensorVarRho = 0.1    // 10cm variance
	const SensorVarAlpha = 0.05 // ~3 degrees variance

	// 2. Kalman Update for Rho (Distance)
	// New Estimate = (Var1*Est2 + Var2*Est1) / (Var1 + Var2)
	gainRho := m.VarRho / (m.VarRho + SensorVarRho)
	m.Rho = m.Rho + gainRho*(newRho-m.Rho)
	m.VarRho = (1 - gainRho) * m.VarRho

	// 3. Kalman Update for Alpha (Angle)
	// Handle angle wrapping (e.g. 359 degrees vs 1 degree)
	angleDiff := newAlpha - m.Alpha
	if angleDiff > math.Pi {
		angleDiff -= 2 * math.Pi
	}
	if angleDiff < -math.Pi {
		angleDiff += 2 * math.Pi
	}

	gainAlpha := m.VarAlpha / (m.VarAlpha + SensorVarAlpha)
	m.Alpha = m.Alpha + gainAlpha*angleDiff
	m.VarAlpha = (1 - gainAlpha) * m.VarAlpha

	// 4. Update Endpoints (Extend the wall)
	// We project all 4 points (Old P1, Old P2, New P1, New P2) onto the NEW infinite line
	// And keep the two that are furthest apart.

	// (Helper function to project point onto line defined by Rho/Alpha)
	// Then find min/max along the line vector.
	m.P1_X, m.P1_Y, m.P2_X, m.P2_Y = mergeEndpoints(
		m.Rho, m.Alpha,
		m.P1_X, m.P1_Y, m.P2_X, m.P2_Y,
		p1x, p1y, p2x, p2y,
	)

	m.Count++
}

// Helper to convert Endpoints -> Rho/Alpha
func pointsToLineParams(x1, y1, x2, y2 float64) (rho, alpha float64) {
	// Normal vector angle
	alpha = math.Atan2(y2-y1, x2-x1) + math.Pi/2

	// Normalize Alpha to -PI..PI
	if alpha > math.Pi {
		alpha -= 2 * math.Pi
	}

	// Distance to origin (Normal form: x*cos(a) + y*sin(a) = rho)
	// We use the midpoint to calculate rho
	mx, my := (x1+x2)/2, (y1+y2)/2
	rho = mx*math.Cos(alpha) + my*math.Sin(alpha)

	// Enforce Rho >= 0 convention (flip alpha if needed)
	if rho < 0 {
		rho = -rho
		alpha += math.Pi
		if alpha > math.Pi {
			alpha -= 2 * math.Pi
		}
	}
	return rho, alpha
}

func mergeEndpoints(rho, alpha, oldX1, oldY1, oldX2, oldY2, newX1, newY1, newX2, newY2 float64) (float64, float64, float64, float64) {

	// 1. Calculate the Direction Vector of the new line
	// (Perpendicular to the Normal Alpha)
	dirX := -math.Sin(alpha)
	dirY := math.Cos(alpha)

	// 2. Calculate the "Anchor" point on the line (closest to world origin 0,0)
	anchorX := rho * math.Cos(alpha)
	anchorY := rho * math.Sin(alpha)

	// 3. Helper to project a 2D point onto the 1D line
	// Returns 't', which is the distance along the line from the anchor
	project := func(x, y float64) float64 {
		dx := x - anchorX
		dy := y - anchorY
		return (dx * dirX) + (dy * dirY) // Dot product
	}

	// 4. Project all 4 candidate points (Old endpoints + New endpoints)
	t1 := project(oldX1, oldY1)
	t2 := project(oldX2, oldY2)
	t3 := project(newX1, newY1)
	t4 := project(newX2, newY2)

	// 5. Find the Min and Max 't' (The extremes)
	minT := math.Min(math.Min(t1, t2), math.Min(t3, t4))
	maxT := math.Max(math.Max(t1, t2), math.Max(t3, t4))

	// 6. Convert back to 2D Coordinates
	// Point = Anchor + (t * Direction)
	finalX1 := anchorX + (minT * dirX)
	finalY1 := anchorY + (minT * dirY)

	finalX2 := anchorX + (maxT * dirX)
	finalY2 := anchorY + (maxT * dirY)

	return finalX1, finalY1, finalX2, finalY2
}

func (m *Map) CheckFreeSpace(robotX, robotY, sensorX, sensorY float64) {
	// 1. Define the Ray (From Robot -> Sensor Hit)
	rayDist := math.Hypot(sensorX-robotX, sensorY-robotY)

	// Safety buffer: Don't delete a wall if we are just 5cm in front of it.
	// We only penalize if we see significantly PAST the wall.
	const Buffer = 30.0 // 30cm buffer

	for i := len(m.Lines) - 1; i >= 0; i-- {
		line := m.Lines[i]

		// 2. Check Intersection: Does the View Ray hit this Map Line?
		// We use a segment-segment intersection test
		intersect, ix, iy := getIntersection(
			robotX, robotY, sensorX, sensorY, // The View Ray
			line.P1_X, line.P1_Y, line.P2_X, line.P2_Y, // The Map Line
		)

		if intersect {
			// 3. Distance Check
			distToWall := math.Hypot(ix-robotX, iy-robotY)

			// If the wall is closer than what we saw (minus buffer),
			// then we are looking THROUGH this wall. It shouldn't be here.
			if distToWall < (rayDist - Buffer) {
				// PENALIZE
				line.Score -= 2.0 // Decrease confidence faster than we build it

				// DELETE if confidence is gone
				if line.Score <= 0 {
					// Remove line from slice (Fast swap-delete)
					m.Lines[i] = m.Lines[len(m.Lines)-1]
					m.Lines = m.Lines[:len(m.Lines)-1]
				}
			}
		}
	}
}

func getIntersection(ax, ay, bx, by, cx, cy, dx, dy float64) (bool, float64, float64) {
	// Standard Cramer's rule or Cross Product approach
	det := (bx-ax)*(dy-cy) - (by-ay)*(dx-cx)
	if det == 0 {
		return false, 0, 0
	} // Parallel

	t := ((cx-ax)*(dy-cy) - (cy-ay)*(dx-cx)) / det
	u := ((cx-ax)*(by-ay) - (cy-ay)*(bx-ax)) / det

	// t and u must be between 0 and 1 for strict segment intersection
	if t >= 0 && t <= 1 && u >= 0 && u <= 1 {
		return true, ax + t*(bx-ax), ay + t*(by-ay)
	}
	return false, 0, 0
}

func (m *Map) OptimizeMap() {
	// Thresholds (Looser than the live matcher to force merging)
	const MergeAngleThresh = 0.35 // ~20 degrees
	const MergeDistThresh = 30.0  // 30 cm

	for i := 0; i < len(m.Lines); i++ {
		if m.Lines[i] == nil {
			continue
		} // Was deleted

		for j := i + 1; j < len(m.Lines); j++ {
			if m.Lines[j] == nil {
				continue
			}
			l1 := m.Lines[i]
			l2 := m.Lines[j]

			// Check similarity
			angleDiff := math.Abs(l1.Alpha - l2.Alpha)
			if angleDiff > math.Pi {
				angleDiff = 2*math.Pi - angleDiff
			}
			rhoDiff := math.Abs(l1.Rho - l2.Rho)

			if angleDiff < MergeAngleThresh && rhoDiff < MergeDistThresh {
				// MERGE l2 INTO l1
				// We project l2's endpoints onto l1
				l1.update(l2.Rho, l2.Alpha, l2.P1_X, l2.P1_Y, l2.P2_X, l2.P2_Y)
				// Combine scores
				l1.Score = math.Min(l1.Score+l2.Score, 100.0)

				// Mark l2 for deletion
				m.Lines[j] = m.Lines[len(m.Lines)-1]
				m.Lines = m.Lines[:len(m.Lines)-1]
				j-- // Check this index again since we swapped
			}
		}
	}
}
