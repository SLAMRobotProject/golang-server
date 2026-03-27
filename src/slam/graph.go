package slam

import (
	"log"
	"math"
)

// LoopClosureEdge represents a relative constraint between two submaps
type LoopClosureEdge struct {
	FromIdx int
	ToIdx   int
	// The pose of 'To' relative to 'From'
	RelPose Pose
}

// DetectAndCloseLoops checks the latest completed submap against older submaps
func (s *GoSLAM) DetectAndCloseLoops() {
	if len(s.Submaps) < 5 { // Need at least a few submaps to have a history
		return
	}

	latestIdx := len(s.Submaps) - 1
	latestSm := s.Submaps[latestIdx]

	for i := 0; i < latestIdx-4; i++ {
		oldSm := s.Submaps[i]

		dist := math.Hypot(latestSm.OriginPose.X-oldSm.OriginPose.X, latestSm.OriginPose.Y-oldSm.OriginPose.Y)
		if dist < 1.5 { // Candidates must be within 1.5 meters visually

			// 1. Synthesize local hits from the new submap to test against the old one
			pseudoHits := latestSm.extractHitsLocal()
			if len(pseudoHits) < 10 {
				continue
			}

			// 2. Do a wider search around to see if they align perfectly
			// We temporarily swap the active submap to the OLD one so we can use s.scorePose()
			tempActive := s.ActiveSubmap
			s.ActiveSubmap = oldSm

			// Wide Scan Match search
			bestPose, bestScore := s.wideMatchScan(latestSm.OriginPose, pseudoHits)

			// Restore active submap
			s.ActiveSubmap = tempActive

			// 3. If a strong match is found, distribute the error backwards
			if bestScore > float64(len(pseudoHits))*0.6 {
				log.Printf("Loop Closure detected! Between Submap %d and %d. Score: %v", i, latestIdx, bestScore)

				// Calculate how much we need to shift the latest submap to match the old submaps geometry
				dx := bestPose.X - latestSm.OriginPose.X
				dy := bestPose.Y - latestSm.OriginPose.Y
				dtheta := NormalizeAngle(bestPose.Theta - latestSm.OriginPose.Theta)

				// Distribute error proportionally along the chain of submaps
				s.distributeError(i, latestIdx, dx, dy, dtheta)
				// Break out after distributing so we don't double apply in one frame
				break
			}
		}
	}
}

// wideMatchScan performs a larger grid search specifically for loop closures
func (s *GoSLAM) wideMatchScan(estPose Pose, localHits []Pose) (Pose, float64) {
	bestPose := estPose
	bestScore := -1e10

	// Wide search: +/- 30cm, +/- 15 degrees
	windowXY := 0.30
	stepXY := 0.05
	windowTheta := 15.0 * math.Pi / 180.0
	stepTheta := 3.0 * math.Pi / 180.0

	rotatedPoints := make([]Pose, len(localHits))

	for dtheta := -windowTheta; dtheta <= windowTheta+0.001; dtheta += stepTheta {
		testTheta := NormalizeAngle(estPose.Theta + dtheta)
		c := math.Cos(testTheta)
		sn := math.Sin(testTheta)

		// Pre-rotate all scan points
		for i, pt := range localHits {
			rotatedPoints[i].X = pt.X*c - pt.Y*sn
			rotatedPoints[i].Y = pt.X*sn + pt.Y*c
		}

		for dx := -windowXY; dx <= windowXY+0.001; dx += stepXY {
			for dy := -windowXY; dy <= windowXY+0.001; dy += stepXY {
				score := 0.0
				testX := estPose.X + dx
				testY := estPose.Y + dy

				// Inline scoring
				for _, rpt := range rotatedPoints {
					ix, iy, valid := s.ActiveSubmap.globalPosToLocalIdx(rpt.X+testX, rpt.Y+testY, s.GridRes)
					if valid {
						score += s.ActiveSubmap.Grid[iy*s.ActiveSubmap.GridSize+ix]
					}
				}

				if score > bestScore {
					bestScore = score
					bestPose = Pose{X: testX, Y: testY, Theta: testTheta}
				}
			}
		}
	}

	return bestPose, bestScore
}

// extractHitsLocal pulls out the highest probability walls of a submap and turns them back into local coordinates
func (sm *Submap) extractHitsLocal() []Pose {
	var hits []Pose
	res := 0.01 // Assuming GridRes hasn't changed

	for y := 0; y < sm.GridSize; y++ {
		for x := 0; x < sm.GridSize; x++ {
			if sm.Grid[y*sm.GridSize+x] > 2.0 { // Must be a solid wall
				// Convert to local relative coordinates
				lx := float64(x-sm.GridSize/2) * res
				ly := float64(sm.GridSize/2-y) * res
				hits = append(hits, Pose{X: lx, Y: ly})
			}
		}
	}
	return hits
}

// distributeError slowly relaxes the error across all submaps leading up to the loop closure
func (s *GoSLAM) distributeError(fromIdx, toIdx int, dx, dy, dtheta float64) {
	numSteps := toIdx - fromIdx
	if numSteps <= 0 {
		return
	}

	for i := fromIdx + 1; i <= toIdx; i++ {
		ratio := float64(i-fromIdx) / float64(numSteps)

		s.Submaps[i].OriginPose.X += dx * ratio
		s.Submaps[i].OriginPose.Y += dy * ratio
		s.Submaps[i].OriginPose.Theta = NormalizeAngle(s.Submaps[i].OriginPose.Theta + dtheta*ratio)
	}

	// Make sure the active submap anchoring is also shifted so the very next drawing steps aren't broken
	if s.ActiveSubmap != nil {
		s.ActiveSubmap.OriginPose.X += dx
		s.ActiveSubmap.OriginPose.Y += dy
		s.ActiveSubmap.OriginPose.Theta = NormalizeAngle(s.ActiveSubmap.OriginPose.Theta + dtheta)
	}

	s.LastIcpPose.X += dx
	s.LastIcpPose.Y += dy
	s.LastIcpPose.Theta = NormalizeAngle(s.LastIcpPose.Theta + dtheta)

	// CRITICAL: We must also shift the running Odometry offset!
	// Otherwise, the very next frame the raw EKF will snap the robot right back
	// out of the loop-closure correction and cause a massive zigzag on the map.
	s.CurrentPose.X += dx
	s.CurrentPose.Y += dy
	s.CurrentPose.Theta = NormalizeAngle(s.CurrentPose.Theta + dtheta)
}
