package slam

import (
	"image"
	"math"
	"sync"
)

// GoSLAM is the native Go implementation of the Grid-based SLAM and Matcher
type GoSLAM struct {
	Poses      []Pose
	GridRes    float64
	GlobalSize int // Used for rendering the viewing window

	Submaps      []*Submap
	ActiveSubmap *Submap
	SubmapRadius float64 // Distance before starting a new submap

	DistThresh  float64
	AngleThresh float64
	LastIcpPose Pose

	CurrentPose Pose // The true, integrated SLAM pose
	LastRawOdom Pose // EKF pose from the previous frame to calculate deltas
	Initialized bool // Flag to drop the first frame's relative odometry

	PendingScan []RayPoint // accumulate single rays until the robot moves

	mu sync.Mutex
}

func NewGoSLAM() *GoSLAM {
	return &GoSLAM{
		Poses:        make([]Pose, 0),
		GridRes:      0.01, // 1cm per cell
		GlobalSize:   500,  // 5m x 5m global viewing area out of the box
		Submaps:      make([]*Submap, 0),
		SubmapRadius: 0.8, // build new submap every 0.8 meters
		DistThresh:   0.05,
		AngleThresh:  0.10,
		CurrentPose:  Pose{0, 0, 0},
		LastRawOdom:  Pose{0, 0, 0},
		Initialized:  false,
		PendingScan:  make([]RayPoint, 0),
	}
}

// ProcessUpdate is the replacement for the Python match/update cycle
func (s *GoSLAM) ProcessUpdate(ekfX, ekfY, ekfTheta float64, scan [][2]float64) image.Image {
	s.mu.Lock()
	defer s.mu.Unlock()

	rawOdom := Pose{X: ekfX, Y: ekfY, Theta: ekfTheta}

	if !s.Initialized {
		s.CurrentPose = rawOdom
		s.LastRawOdom = rawOdom
		s.Initialized = true
	}

	// 1. Calculate relative movement from raw Odometry since last frame
	deltaX := rawOdom.X - s.LastRawOdom.X
	deltaY := rawOdom.Y - s.LastRawOdom.Y
	deltaTheta := NormalizeAngle(rawOdom.Theta - s.LastRawOdom.Theta)

	// Translate that movement into the robot's local frame
	cosRaw := math.Cos(s.LastRawOdom.Theta)
	sinRaw := math.Sin(s.LastRawOdom.Theta)
	localDX := deltaX*cosRaw + deltaY*sinRaw
	localDY := -deltaX*sinRaw + deltaY*cosRaw

	// 2. Apply that pure relative local movement to our "corrected" SLAM Pose
	cosSLAM := math.Cos(s.CurrentPose.Theta)
	sinSLAM := math.Sin(s.CurrentPose.Theta)
	s.CurrentPose.X += localDX*cosSLAM - localDY*sinSLAM
	s.CurrentPose.Y += localDX*sinSLAM + localDY*cosSLAM
	s.CurrentPose.Theta = NormalizeAngle(s.CurrentPose.Theta + deltaTheta)

	s.LastRawOdom = rawOdom
	estPose := s.CurrentPose

	// Only append to Poses if it's the first pose or if the pose has changed
	if len(s.Poses) == 0 || s.Poses[len(s.Poses)-1].X != estPose.X || s.Poses[len(s.Poses)-1].Y != estPose.Y || s.Poses[len(s.Poses)-1].Theta != estPose.Theta {
		s.Poses = append(s.Poses, estPose)
	}

	// Accumulate rays continuously
	for _, sc := range scan {
		s.PendingScan = append(s.PendingScan, RayPoint{
			RobotX:     estPose.X,
			RobotY:     estPose.Y,
			RobotTheta: estPose.Theta,
			Angle:      sc[0],
			Dist:       sc[1],
		})
	}

	// Check if moved enough to process a scan update
	distMoved := math.Hypot(estPose.X-s.LastIcpPose.X, estPose.Y-s.LastIcpPose.Y)
	angleMoved := math.Abs(NormalizeAngle(estPose.Theta - s.LastIcpPose.Theta))

	if distMoved >= s.DistThresh || angleMoved >= s.AngleThresh {
		// 1. Gather all local point clouds from the pending string of scans
		var localHits []Pose

		for _, item := range s.PendingScan {
			if item.Dist < 0.1 || item.Dist > 2.95 {
				continue // Filter noise and max bounds
			}
			lx := item.Dist * math.Cos(item.Angle)
			ly := item.Dist * math.Sin(item.Angle)
			localHits = append(localHits, Pose{X: lx, Y: ly})
		}

		// 2. Map-to-Map Preferred: Skip the single-scan matching!
		// Just trust the highly accurate Odometry to build the single submap.
		correctedPose := estPose

		// 2.5 Update the active tracking pose with the mathematically aligned matched pose
		s.CurrentPose = correctedPose

		// Overwrite the final visual position with the corrected one
		if len(s.Poses) > 0 {
			s.Poses[len(s.Poses)-1] = correctedPose
		}

		// 3. Process the adjusted raycasts for the occupancy grid
		var freeSpace []Pose
		var hitSpace []Pose

		c := math.Cos(correctedPose.Theta)
		sn := math.Sin(correctedPose.Theta)

		for _, item := range s.PendingScan {
			dist := item.Dist
			if dist < 0.1 || dist > 5.0 {
				continue
			}

			lx := dist * math.Cos(item.Angle)
			ly := dist * math.Sin(item.Angle)

			// Map local coords to global map coordinates using the *Corrected* Pose
			gx := (lx*c - ly*sn) + correctedPose.X
			gy := (lx*sn + ly*c) + correctedPose.Y

			if dist < 2.95 {
				hitSpace = append(hitSpace, Pose{X: gx, Y: gy})
			}

			// Raycast for free space using corrected origins
			steps := int(dist / s.GridRes)
			for i := 1; i < steps; i++ {
				t := float64(i) / float64(steps)
				fx := correctedPose.X + t*(gx-correctedPose.X)
				fy := correctedPose.Y + t*(gy-correctedPose.Y)
				freeSpace = append(freeSpace, Pose{X: fx, Y: fy})
			}
		}

		s.updateOccupancyGrid(freeSpace, hitSpace)

		// Reset pending scan and update anchor
		s.PendingScan = make([]RayPoint, 0)
		s.LastIcpPose = correctedPose
	}

	return s.renderGridImage()
}

func (s *GoSLAM) updateOccupancyGrid(freeSpace []Pose, hitSpace []Pose) {
	// Ensure we have an active submap
	if s.ActiveSubmap == nil {
		s.ActiveSubmap = NewSubmap(len(s.Submaps), s.LastIcpPose, 800) // 8m x 8m submap to fit 3m full sensor radius
	}

	// Update Log-Odds in Active Submap
	for _, p := range freeSpace {
		ix, iy, valid := s.ActiveSubmap.globalPosToLocalIdx(p.X, p.Y, s.GridRes)
		if valid {
			idx := iy*s.ActiveSubmap.GridSize + ix
			// Reduce the free-space penalty.
			// If it clears too fast, the sensor washes away good walls.
			s.ActiveSubmap.Grid[idx] -= 0.1
			if s.ActiveSubmap.Grid[idx] < -5.0 {
				s.ActiveSubmap.Grid[idx] = -5.0
			}
		}
	}

	for _, p := range hitSpace {
		ix, iy, valid := s.ActiveSubmap.globalPosToLocalIdx(p.X, p.Y, s.GridRes)
		if valid {
			// Make walls thicker (3x3 grid) to prevent sensor noise from washing them away
			for kx := -1; kx <= 1; kx++ {
				for ky := -1; ky <= 1; ky++ {
					nx, ny := ix+kx, iy+ky
					if nx >= 0 && nx < s.ActiveSubmap.GridSize && ny >= 0 && ny < s.ActiveSubmap.GridSize {
						idx := ny*s.ActiveSubmap.GridSize + nx
						val := 0.9
						if kx != 0 || ky != 0 {
							val = 0.5 // Neighbor cells get slightly less confidence
						}
						s.ActiveSubmap.Grid[idx] += val
						if s.ActiveSubmap.Grid[idx] > 5.0 {
							s.ActiveSubmap.Grid[idx] = 5.0
						}
					}
				}
			}
		}
	}

	s.ActiveSubmap.NumScans++

	// Check if submap is "full" and we need to start a new one
	dx := s.LastIcpPose.X - s.ActiveSubmap.OriginPose.X
	dy := s.LastIcpPose.Y - s.ActiveSubmap.OriginPose.Y
	distFromOrigin := math.Hypot(dx, dy)

	if distFromOrigin >= s.SubmapRadius || s.ActiveSubmap.NumScans > 150 {
		// Minimum Submap Health Check:
		// Ensure this submap actually saw enough geometry to be worth saving.
		// If the car drove through an empty field, don't save a blank map just because it traveled 2 meters.
		solidVoxelCount := 0
		for _, val := range s.ActiveSubmap.Grid {
			if val > 1.0 { // Has definite geometry
				solidVoxelCount++
			}
		}

		// Only finalize if we have a decent amount of definite wall structure (e.g. 200 solid pixels)
		// Otherwise, keep the submap open so it keeps collecting until it finds walls!
		if solidVoxelCount > 200 {

			// --- MAP-TO-MAP MATCHING: Snap the completed Submap to the PREVIOUS Submap ---
			if len(s.Submaps) > 0 {
				prevSm := s.Submaps[len(s.Submaps)-1]
				pseudoHits := s.ActiveSubmap.extractHitsLocal()

				if len(pseudoHits) > 20 {
					// Swap temporarily
					tempActive := s.ActiveSubmap
					s.ActiveSubmap = prevSm

					// Match the extracted walls against the previous submap
					bestPose, bestScore := s.wideMatchScan(tempActive.OriginPose, pseudoHits)

					s.ActiveSubmap = tempActive // restore

					// If we found a good alignment, shift the current map and Odometry
					if bestScore > float64(len(pseudoHits))*0.4 {
						driftX := bestPose.X - s.ActiveSubmap.OriginPose.X
						driftY := bestPose.Y - s.ActiveSubmap.OriginPose.Y
						driftTheta := NormalizeAngle(bestPose.Theta - s.ActiveSubmap.OriginPose.Theta)

						s.ActiveSubmap.OriginPose = bestPose
						s.LastIcpPose.X += driftX
						s.LastIcpPose.Y += driftY
						s.LastIcpPose.Theta = NormalizeAngle(s.LastIcpPose.Theta + driftTheta)
						s.CurrentPose.X += driftX
						s.CurrentPose.Y += driftY
						s.CurrentPose.Theta = NormalizeAngle(s.CurrentPose.Theta + driftTheta)
					}
				}
			}

			s.ActiveSubmap.Finished = true
			s.Submaps = append(s.Submaps, s.ActiveSubmap)

			// Attempt to detect and close loops every time we finish a submap!
			// Wait slightly to ensure it doesn't overcorrect immediately
			if len(s.Submaps) > 5 {
				s.DetectAndCloseLoops()
			}

			s.ActiveSubmap = NewSubmap(len(s.Submaps), s.LastIcpPose, 800) // 8m x 8m
		} else if s.ActiveSubmap.NumScans > 300 {
			// Fallback: If it has zero geometry but has been scanning for 300 cycles (dead end / empty space),
			// force a reset to prevent infinitely growing an un-matchable map.
			s.ActiveSubmap = NewSubmap(len(s.Submaps), s.LastIcpPose, 800)
		}
	}
}
