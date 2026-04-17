package slam

import (
	util "golang-server/utilities"
	"math"
)

// Sonar / Gaussian-beam Inverse Sensor Model
//
// Each scan ray is modelled as a narrow cone with a Gaussian angular profile.
// The linear taper (0.2 + 0.8*ratio) was too flat — edge cells received 20 %
// of centre weight.  A Gaussian concentrates evidence sharply at the beam
// centre and falls off rapidly, matching the physical PSF of ToF sensors.
//
//   • Free wedge  (cellDist < hitDist - ismDepthNoise)
//       → uniform free-space decrement regardless of angular position
//
//   • Hit band    (hitDist - ismDepthNoise ≤ cellDist ≤ hitDist)
//       → ismHitOdds * exp(−angleDiff² / (2σ²))
//         σ = ismBeamHalfFOV * ismBeamSigmaFrac
//         At the cone centre the full increment is applied; at the halfFOV
//         edge the weight is exp(−(1/sigmaFrac)²/2).
//         With sigmaFrac=0.45: edge weight ≈ 0.08 (8 %) vs 20 % before.
//
// Constants are tuned for a 5 cm grid and per-ray half-cone of ±3°.

const (
	ismHitOdds       float32 = 0.40          // log-odds per confirmed hit at beam centre
	ismFreeOdds      float32 = 0.08          // log-odds removed per free-space cell
	ismMaxOdds       float32 = 3.5           // occupied saturation cap
	ismMinOdds       float32 = -1.0          // free saturation cap
	ismDepthNoise            = GridRes * 1.0 // metres — hit-band depth tolerance (one cell)
	ismBeamHalfFOV           = 0.052         // radians — per-ray half-cone ≈ 3°
	ismBeamSigmaFrac         = 0.45          // Gaussian σ = halfFOV * sigmaFrac; edge weight ≈ 8 %
)

// UpdateSolidCone applies a complete Inverse Sensor Model for a fused Camera+ToF object.
// It clears free space from the robot up to the object, and paints a solid arc at the depth.
func (s *Submap) UpdateSolidCone(robotX, robotY, robotTheta, dist, startX, endX float64) {
	// 1. Convert Robot Pose to Submap Local Frame
	locRx, locRy := s.Origin.GlobalToLocal(robotX, robotY)
	locRt := util.NormalizeAngle(robotTheta - s.Origin.Theta)

	angle1 := math.Atan2(startX, dist)
	angle2 := math.Atan2(endX, dist)
	minAngle := math.Min(angle1, angle2)
	maxAngle := math.Max(angle1, angle2)

	// 2. Define a bounding box in local cell space around the robot.
	// Must use the rotated local position (locRx/locRy), not the raw world offset,
	// otherwise the box is wrong whenever Origin.Theta != 0.
	maxRange := dist + ismDepthNoise
	robotCX := locRx / GridRes
	robotCY := -locRy / GridRes

	spanCells := maxRange/GridRes + 1
	minCX := int(math.Floor(robotCX - spanCells))
	maxCX := int(math.Ceil(robotCX + spanCells))
	minCY := int(math.Floor(robotCY - spanCells))
	maxCY := int(math.Ceil(robotCY + spanCells))

	// Trust weight for the wall (exponential falloff based on distance)
	weight := float32(math.Exp(-dist / 2.0))
	addOdds := ismHitOdds * weight

	// 3. Sweep the grid
	for cy := minCY; cy <= maxCY; cy++ {
		for cx := minCX; cx <= maxCX; cx++ {
			// Local physical coordinates of this cell
			lx := float64(cx) * GridRes
			ly := -float64(cy) * GridRes

			// Distance from local robot
			ldx := lx - locRx
			ldy := ly - locRy
			cellDist := math.Sqrt(ldx*ldx + ldy*ldy)
			if cellDist < 1e-6 {
				continue
			}

			// Calculate the angle of this cell relative to the robot's heading
			// normalizeAngle wraps the result to [-Pi, Pi]
			cellAngle := util.NormalizeAngle(math.Atan2(ldy, ldx) - locRt)

			// Check if the cell falls inside the object's angular cone
			if cellAngle >= minAngle && cellAngle <= maxAngle {
				var delta float32

				// Dynamic Depth Thickening
				// The further the angle is from the center of the robot's vision (0.0),
				// the more the depth smears. We expand the depth tolerance so steep
				// grazing angles don't leave gaps in the grid.
				angleFromCenter := math.Abs(cellAngle)

				// Increase thickness by up to 100% at the extreme edges of the FOV
				// (You can tune the 1.5 multiplier based on how jagged your walls look)
				dynamicDepthNoise := ismDepthNoise * (1.0 + (angleFromCenter * 1.5))

				if cellDist < dist-dynamicDepthNoise {
					// REGION 1: Free space (between robot and object)
					delta = -ismFreeOdds
				} else if cellDist <= dist+dynamicDepthNoise {
					// REGION 2: The Object Surface (with dynamic thickness)
					delta = addOdds
				}

				// Apply the log-odds update and clamp
				if delta != 0 {
					key := CellKey{X: cx, Y: cy}
					v := s.grid[key] + delta

					if v > ismMaxOdds {
						v = ismMaxOdds
					} else if v < ismMinOdds {
						v = ismMinOdds
					}
					s.grid[key] = v
				}
			}
		}
	}
}

// ClearFOV acts as an occluded eraser using Raytracing.
// It sweeps rays across the FOV. If a ray hits a known wall, it chips
// away at the wall's confidence but STOPS, preventing it from clearing
// the unseen space (shadow) behind the wall.
func (s *Submap) ClearFOV(robotX, robotY, robotTheta, maxRange, fovAngle float64) {
	halfFOV := fovAngle / 2.0

	// Convert Robot Pose to Submap Local Frame
	locRx, locRy := s.Origin.GlobalToLocal(robotX, robotY)
	locRt := util.NormalizeAngle(robotTheta - s.Origin.Theta)

	// 1. Calculate how many rays we need.
	// To prevent gaps at maxRange, the arc length between rays must be <= GridRes.
	// ArcLength = radius * theta  ->  theta = GridRes / maxRange
	minRays := int(math.Ceil((fovAngle * maxRange) / GridRes))
	if minRays < 10 {
		minRays = 10 // Safety minimum
	}
	angleStep := fovAngle / float64(minRays)

	// 2. Shoot the rays
	for i := 0; i <= minRays; i++ {
		rayAngle := locRt - halfFOV + float64(i)*angleStep
		cosA := math.Cos(rayAngle)
		sinA := math.Sin(rayAngle)

		// 3. March along the ray step-by-step
		for r := 0.0; r <= maxRange; r += GridRes {
			hitX := locRx + r*cosA
			hitY := locRy + r*sinA

			// Convert physical coordinates to dynamic submap grid indices
			cx := int(math.Floor(hitX / GridRes))
			cy := -int(math.Floor(hitY / GridRes)) // Inverted Y-axis

			key := CellKey{X: cx, Y: cy}

			// Read current cell value
			currentVal := s.grid[key]

			// Apply the free-space penalty
			newVal := currentVal - ismFreeOdds
			if newVal < ismMinOdds {
				newVal = ismMinOdds
			}
			s.grid[key] = newVal

			// --- OCCLUSION CHECK ---
			// If this cell was considered a solid wall BEFORE we just penalized it,
			// the physical sensor beam is blocked!
			// We break the loop so it casts a shadow and doesn't erase unseen space.
			// (Assuming 0.08 is your visual threshold for a wall)
			if currentVal > 0.08 {
				break
			}
		}
	}
}
