package slam

import (
	"math"
)

// MatchScan performs a highly optimized grid-search correlative scan matching.
// It searches around the `estPose` (Odometry) to find an offset that maximizes
// the alignment of `localHits` (lidar hits in robot frame) with the `ActiveSubmap`.
func (s *GoSLAM) MatchScan(estPose Pose, localHits []Pose) Pose {
	if s.ActiveSubmap == nil || s.ActiveSubmap.NumScans < 10 {
		// Not enough geometry to match against yet, trust odometry completely
		return estPose
	}

	bestPose := estPose

	// 1. Calculate the baseline score of the exact estimated pose (zero shift)
	bestScore := 0.0
	c_base := math.Cos(estPose.Theta)
	s_base := math.Sin(estPose.Theta)
	for _, pt := range localHits {
		gx := pt.X*c_base - pt.Y*s_base + estPose.X
		gy := pt.X*s_base + pt.Y*c_base + estPose.Y
		ix, iy, valid := s.ActiveSubmap.globalPosToLocalIdx(gx, gy, s.GridRes)
		if valid {
			bestScore += s.ActiveSubmap.Grid[iy*s.ActiveSubmap.GridSize+ix]
		}
	}

	// Dynamically scale search window based on if we are mostly translating or rotating.
	distMoved := math.Hypot(estPose.X-s.LastIcpPose.X, estPose.Y-s.LastIcpPose.Y)
	windowXY := 0.06 // Max translation search (6cm)
	if distMoved < 0.03 {
		// If the car barely moved translationally, it's mostly turning in place.
		// Strictly prevent the SLAM from 'sliding' sideways to find a fake better fit.
		windowXY = 0.02
	}

	stepXY := 0.02
	windowTheta := 4.0 * math.Pi / 180.0
	stepTheta := 1.0 * math.Pi / 180.0

	rotatedPoints := make([]Pose, len(localHits))

	for dtheta := -windowTheta; dtheta <= windowTheta+0.001; dtheta += stepTheta {
		testTheta := NormalizeAngle(estPose.Theta + dtheta)
		c := math.Cos(testTheta)
		sn := math.Sin(testTheta)

		// Pre-rotate all scan points for this specific heading
		for i, pt := range localHits {
			rotatedPoints[i].X = pt.X*c - pt.Y*sn
			rotatedPoints[i].Y = pt.X*sn + pt.Y*c
		}

		// Inner loops just strictly do fast additions for X/Y shifts
		for dx := -windowXY; dx <= windowXY+0.001; dx += stepXY {
			for dy := -windowXY; dy <= windowXY+0.001; dy += stepXY {
				// Skip re-evaluating the zero-shift center, we already scored it
				if dx == 0 && dy == 0 && dtheta == 0 {
					continue
				}

				score := 0.0
				testX := estPose.X + dx
				testY := estPose.Y + dy

				// Inline scoring to avoid function call overhead
				for _, rpt := range rotatedPoints {
					// 1) Match against Active Submap
					ix, iy, valid := s.ActiveSubmap.globalPosToLocalIdx(rpt.X+testX, rpt.Y+testY, s.GridRes)
					if valid {
						score += s.ActiveSubmap.Grid[iy*s.ActiveSubmap.GridSize+ix]
					}

					// 2) Bonus Score from the previous submap to increase Match density / stability
					numSubmaps := len(s.Submaps)
					if numSubmaps > 0 {
						prev := s.Submaps[numSubmaps-1]
						pix, piy, pValid := prev.globalPosToLocalIdx(rpt.X+testX, rpt.Y+testY, s.GridRes)
						if pValid {
							pScore := prev.Grid[piy*prev.GridSize+pix]
							// Only add structural hits from history, ignore free-space conflicts between maps
							if pScore > 0 {
								score += pScore
							}
						}
					}
				}

				// Regularization: Heavily penalize moving away from the estimated odometry.
				// Since grid odds cap at 5.0, a penalty of 1000 means sliding 5cm costs 50 points (10 solid wall hits).
				// This stops the algorithm from jumping sideways just to grab 1 or 2 extra random ray hits.
				penalty := (math.Abs(dx)*1000.0 + math.Abs(dy)*1000.0 + math.Abs(dtheta)*500.0)
				score -= penalty

				if score > bestScore {
					bestScore = score
					bestPose = Pose{X: testX, Y: testY, Theta: testTheta}
				}
			}
		}
	}

	return bestPose
}
