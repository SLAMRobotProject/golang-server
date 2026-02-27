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
	// Filter out too small obstacles (e.g. less than 15 cm)
	const MinWallLength = 15.0
	if math.Hypot(p2x-p1x, p2y-p1y) < MinWallLength {
		return
	}

	// 0. FREE SPACE CHECK: Remove walls that contradict this observation
	// Instead of just 2 endpoints, cast 5 rays along the segment to catch hypotenuses inside the cone.
	for step := 0; step <= 4; step++ {
		t := float64(step) / 4.0
		px := p1x + t*(p2x-p1x)
		py := p1y + t*(p2y-p1y)
		m.CheckFreeSpace(robotX, robotY, px, py)
	}

	// 1. Convert Cartesian endpoints to Polar (Rho/Alpha) for math matching
	newRho, newAlpha := pointsToLineParams(p1x, p1y, p2x, p2y)

	// Thresholds for matching (Is this the same wall?)
	const MatchDistThresh = 30.0  // 20cm
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

		// Point-to-Line Euclidean check:
		// Calculate the perpendicular distance from the new segment's endpoints to the known infinite line
		perpDist1 := math.Abs(line.Rho - (p1x*math.Cos(line.Alpha) + p1y*math.Sin(line.Alpha)))
		perpDist2 := math.Abs(line.Rho - (p2x*math.Cos(line.Alpha) + p2y*math.Sin(line.Alpha)))
		maxPerpDist := math.Max(perpDist1, perpDist2)

		// If it's close enough in both Angle and geometrically close to the line...
		if angleDiff < MatchAngleThresh && maxPerpDist < MatchDistThresh {
			// Check if the segments are longitudinally close (no large gaps)
			t1 := projectPointOnLine(line.P1_X, line.P1_Y, line.Rho, line.Alpha)
			t2 := projectPointOnLine(line.P2_X, line.P2_Y, line.Rho, line.Alpha)
			t3 := projectPointOnLine(p1x, p1y, line.Rho, line.Alpha)
			t4 := projectPointOnLine(p2x, p2y, line.Rho, line.Alpha)

			minOld, maxOld := math.Min(t1, t2), math.Max(t1, t2)
			minNew, maxNew := math.Min(t3, t4), math.Max(t3, t4)

			gap := 0.0
			if maxNew < minOld {
				gap = minOld - maxNew
			} else if minNew > maxOld {
				gap = minNew - maxOld
			}

			if gap < 10.0 { // Safe to bump gap back up to 10cm since Point-to-Line is much stricter geometrically
				// Simple scoring: sum of errors
				score := angleDiff + maxPerpDist
				if score < bestMatchScore {
					bestMatchScore = score
					bestMatchIdx = i
				}
			}
		}
	}

	// 3. Update or Add
	if bestMatchIdx != -1 {
		// MATCH FOUND: Merge and improve the existing line
		m.Lines[bestMatchIdx].update(newRho, newAlpha, p1x, p1y, p2x, p2y, true)

		// Increase confidence (max 200)
		m.Lines[bestMatchIdx].Existence += 10.0
		if m.Lines[bestMatchIdx].Existence > 200.0 {
			m.Lines[bestMatchIdx].Existence = 200.0
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
				Existence: 20.0, // Start with low existence
				Count:     1,
			},
		}
		m.Lines = append(m.Lines, newLine)
	}

	m.OptimizeMap()
}

func (m *MapLine) update(newRho, newAlpha, p1x, p1y, p2x, p2y float64, isObservation bool) {

	// 1. Define Sensor Uncertainty (The "Unknown" sensor noise)
	// You tune these values!
	// If the camera is noisy, make these HIGH.
	const SensorVarRho = 0.1    // 10cm variance
	const SensorVarAlpha = 0.05 // ~3 degrees variance

	// 2. Length-Weighted Inertia
	// Calculate how long the existing wall is versus the newly observed segment
	oldLength := math.Hypot(m.P2_X-m.P1_X, m.P2_Y-m.P1_Y)
	newLength := math.Hypot(p2x-p1x, p2y-p1y)

	lengthRatio := newLength / oldLength
	if oldLength == 0 {
		lengthRatio = 1.0
	}
	// Cap the ratio so a huge new observation doesn't completely overwrite a good old wall instantly
	if lengthRatio > 1.0 {
		lengthRatio = 1.0
	}

	// 3. Kalman Update for Rho (Distance)
	// New Estimate = (Var1*Est2 + Var2*Est1) / (Var1 + Var2)
	gainRho := m.VarRho / (m.VarRho + SensorVarRho)
	gainRho *= lengthRatio // Weight the gain! Small lines barely move big walls.

	m.Rho = m.Rho + gainRho*(newRho-m.Rho)
	m.VarRho = (1 - gainRho) * m.VarRho

	// 4. Kalman Update for Alpha (Angle)
	// Handle angle wrapping (e.g. 359 degrees vs 1 degree)
	angleDiff := newAlpha - m.Alpha
	if angleDiff > math.Pi {
		angleDiff -= 2 * math.Pi
	}
	if angleDiff < -math.Pi {
		angleDiff += 2 * math.Pi
	}

	gainAlpha := m.VarAlpha / (m.VarAlpha + SensorVarAlpha)
	gainAlpha *= lengthRatio // Weight the gain!

	m.Alpha = m.Alpha + gainAlpha*angleDiff
	m.VarAlpha = (1 - gainAlpha) * m.VarAlpha

	// 5. Update Endpoints (Extend the wall)
	// We project all 4 points (Old P1, Old P2, New P1, New P2) onto the NEW infinite line
	// And keep the two that are furthest apart.

	// (Helper function to project point onto line defined by Rho/Alpha)
	// Then find min/max along the line vector.
	m.P1_X, m.P1_Y, m.P2_X, m.P2_Y = mergeEndpoints(
		m.Rho, m.Alpha,
		m.P1_X, m.P1_Y, m.P2_X, m.P2_Y,
		p1x, p1y, p2x, p2y,
		isObservation,
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

func projectPointOnLine(x, y, rho, alpha float64) float64 {
	dirX := -math.Sin(alpha)
	dirY := math.Cos(alpha)
	anchorX := rho * math.Cos(alpha)
	anchorY := rho * math.Sin(alpha)
	dx := x - anchorX
	dy := y - anchorY
	return (dx * dirX) + (dy * dirY)
}

func mergeEndpoints(rho, alpha, oldX1, oldY1, oldX2, oldY2, newX1, newY1, newX2, newY2 float64, isObservation bool) (float64, float64, float64, float64) {

	// 1. Calculate the Direction Vector of the new line
	// (Perpendicular to the Normal Alpha)
	dirX := -math.Sin(alpha)
	dirY := math.Cos(alpha)

	// 2. Calculate the "Anchor" point on the line (closest to world origin 0,0)
	anchorX := rho * math.Cos(alpha)
	anchorY := rho * math.Sin(alpha)

	// 3. (Replaced by projectPointOnLine)

	// 4. Project all 4 candidate points (Old endpoints + New endpoints)
	t1 := projectPointOnLine(oldX1, oldY1, rho, alpha)
	t2 := projectPointOnLine(oldX2, oldY2, rho, alpha)
	t3 := projectPointOnLine(newX1, newY1, rho, alpha)
	t4 := projectPointOnLine(newX2, newY2, rho, alpha)

	// 5. Find the Min and Max 't' (The extremes)
	oldMinT := math.Min(t1, t2)
	oldMaxT := math.Max(t1, t2)
	newMinT := math.Min(t3, t4)
	newMaxT := math.Max(t3, t4)

	finalMinT := oldMinT
	finalMaxT := oldMaxT

	if isObservation {
		const GrowthRate = 0.2 // Grow towards new max/min by 20% per frame
		if newMinT < oldMinT {
			finalMinT = oldMinT + (newMinT-oldMinT)*GrowthRate
		}
		if newMaxT > oldMaxT {
			finalMaxT = oldMaxT + (newMaxT-oldMaxT)*GrowthRate
		}
	} else {
		// Merging two existing walls, snap to the absolute extremities to avoid losing wall data
		finalMinT = math.Min(oldMinT, newMinT)
		finalMaxT = math.Max(oldMaxT, newMaxT)
	}

	// 6. Convert back to 2D Coordinates
	// Point = Anchor + (t * Direction)
	finalX1 := anchorX + (finalMinT * dirX)
	finalY1 := anchorY + (finalMinT * dirY)

	finalX2 := anchorX + (finalMaxT * dirX)
	finalY2 := anchorY + (finalMaxT * dirY)

	return finalX1, finalY1, finalX2, finalY2
}

func (m *Map) CheckFreeSpace(robotX, robotY, sensorX, sensorY float64) {
	// 1. Define the Ray (From Robot -> Sensor Hit)
	rayDist := math.Hypot(sensorX-robotX, sensorY-robotY)

	// Safety buffer: Don't delete a wall if we are just 8cm in front of it.
	// We only penalize if we see significantly PAST the wall.
	const Buffer = 8.0

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
				penalty := 1.0
				line.Existence -= penalty

				// DELETE if confidence is gone
				if line.Existence <= 0 {
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
	const MergeAngleThresh = 0.55 // ~20 degrees
	const MergeDistThresh = 8.0   // 30 cm

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

			// Point-to-Line Euclidean Check for merging
			perpDist1 := math.Abs(l1.Rho - (l2.P1_X*math.Cos(l1.Alpha) + l2.P1_Y*math.Sin(l1.Alpha)))
			perpDist2 := math.Abs(l1.Rho - (l2.P2_X*math.Cos(l1.Alpha) + l2.P2_Y*math.Sin(l1.Alpha)))
			maxPerpDist := math.Max(perpDist1, perpDist2)

			if angleDiff < MergeAngleThresh && maxPerpDist < MergeDistThresh {
				// Check gap
				t1 := projectPointOnLine(l1.P1_X, l1.P1_Y, l1.Rho, l1.Alpha)
				t2 := projectPointOnLine(l1.P2_X, l1.P2_Y, l1.Rho, l1.Alpha)
				t3 := projectPointOnLine(l2.P1_X, l2.P1_Y, l1.Rho, l1.Alpha)
				t4 := projectPointOnLine(l2.P2_X, l2.P2_Y, l1.Rho, l1.Alpha)

				minOld, maxOld := math.Min(t1, t2), math.Max(t1, t2)
				minNew, maxNew := math.Min(t3, t4), math.Max(t3, t4)

				gap := 0.0
				if maxNew < minOld {
					gap = minOld - maxNew
				} else if minNew > maxOld {
					gap = minNew - maxOld
				}

				if gap < 10.0 {
					// MERGE l2 INTO l1
					// We project l2's endpoints onto l1
					l1.update(l2.Rho, l2.Alpha, l2.P1_X, l2.P1_Y, l2.P2_X, l2.P2_Y, false)
					// Combine existence scores (cap at 200)
					l1.Existence = math.Min(l1.Existence+l2.Existence, 200.0)

					// Mark l2 for deletion
					m.Lines[j] = m.Lines[len(m.Lines)-1]
					m.Lines = m.Lines[:len(m.Lines)-1]
					j-- // Check this index again since we swapped
				}
			}
		}
	}

	const MinWallLength = 15.0
	for i := 0; i < len(m.Lines); i++ {
		line := m.Lines[i]
		if line == nil {
			continue
		}

		length := math.Hypot(line.P2_X-line.P1_X, line.P2_Y-line.P1_Y)
		if length < MinWallLength {
			m.Lines[i] = m.Lines[len(m.Lines)-1]
			m.Lines = m.Lines[:len(m.Lines)-1]
			i--
		}
	}
}
