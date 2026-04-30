package slam

import (
	"math"

	util "golang-server/utilities"
)

// poseEdge is a relative-pose constraint between two submaps recorded at
// match time.  Sequential edges (weight 1.0) come from the EKF backbone;
// loop-closure edges (weight 3.0) come from scan matching.
type poseEdge struct {
	from, to       int
	dx, dy, dtheta float64
	weightXY       float64
	weightTheta    float64
}

// addSequentialEdge records the consecutive-submap relative pose as a backbone
// constraint (weightXY 1.0).
func (m *OccupancyMap) addSequentialEdge(fromIdx, toIdx int) {
	if fromIdx < 0 || toIdx >= len(m.submaps) || fromIdx >= toIdx {
		return
	}
	f := m.submaps[fromIdx]
	t := m.submaps[toIdx]

	locX, locY := f.Origin.GlobalToLocal(t.Origin.X, t.Origin.Y)

	m.edges = append(m.edges, poseEdge{
		from:        fromIdx,
		to:          toIdx,
		dx:          locX,
		dy:          locY,
		dtheta:      util.NormalizeAngle(t.Origin.Theta - f.Origin.Theta),
		weightXY:    1.0,
		weightTheta: 3.0,
	})
}

// updateLastSequentialEdge refreshes the most recent sequential edge whose
// destination belongs to robotID, after scan matching corrected that submap's
// origin.
func (m *OccupancyMap) updateLastSequentialEdge(robotID int) {
	for i := len(m.edges) - 1; i >= 0; i-- {
		e := &m.edges[i]
		if e.weightXY > 1.0 {
			continue // skip loop-closure edges
		}
		if m.submaps[e.to].RobotID != robotID {
			continue
		}
		f := m.submaps[e.from]
		t := m.submaps[e.to]
		locX, locY := f.Origin.GlobalToLocal(t.Origin.X, t.Origin.Y)
		e.dx = locX
		e.dy = locY
		e.dtheta = util.NormalizeAngle(t.Origin.Theta - f.Origin.Theta)
		return
	}
}

// applyLoopClosure adds the LC constraint, runs Ceres, and returns the pose
// correction for the closed submap.  It also computes approximate corrected
// poses for any other robot whose active submap was moved by the optimisation
// and stores them in pendingCorrections for pick-up on the next ProcessUpdate.
func (m *OccupancyMap) applyLoopClosure(anchorID, closedID int, shiftX, shiftY, shiftTheta float64) (float64, float64, float64) {
	anchor := m.submaps[anchorID]
	closed := m.submaps[closedID]
	closedRobotID := closed.RobotID

	oldX := closed.Origin.X
	oldY := closed.Origin.Y
	oldTheta := closed.Origin.Theta

	// Corrected ideal pose of the closed submap
	correctedX := oldX + shiftX
	correctedY := oldY + shiftY
	correctedTheta := util.NormalizeAngle(oldTheta + shiftTheta)

	// Express the constraint in the anchor's local frame
	locX, locY := anchor.Origin.GlobalToLocal(correctedX, correctedY)
	relDTheta := util.NormalizeAngle(correctedTheta - anchor.Origin.Theta)

	m.edges = append(m.edges, poseEdge{
		from: anchorID, to: closedID,
		dx: locX, dy: locY, dtheta: relDTheta,
		weightXY:    3.0,
		weightTheta: 10.0,
	})

	// Snapshot every OTHER robot's active-submap origin and its robot's local
	// position within that submap, so we can compute corrected poses after
	// Ceres runs.
	type crossSnap struct {
		submap *Submap
		localX float64
		localY float64
		dt     float64 // robot heading relative to submap origin theta
	}
	crossSnaps := make(map[int]crossSnap, len(m.activeSubmaps))
	for rid, sub := range m.activeSubmaps {
		if rid == closedRobotID || sub == nil {
			continue
		}
		rs := m.robots[rid]
		if rs == nil {
			continue
		}
		lx, ly := sub.Origin.GlobalToLocal(rs.CurrentPose.X, rs.CurrentPose.Y)
		dt := util.NormalizeAngle(rs.CurrentPose.Theta - sub.Origin.Theta)
		crossSnaps[rid] = crossSnap{submap: sub, localX: lx, localY: ly, dt: dt}
	}

	m.optimizePoseGraph()

	// For each other robot, re-project its position using the (possibly moved)
	// submap origin.  Store as a pending correction if the shift is non-trivial.
	for rid, snap := range crossSnaps {
		sub := snap.submap // same pointer; Origin may have changed
		newOt := sub.Origin.Theta
		cosNew := math.Cos(newOt)
		sinNew := math.Sin(newOt)
		newX := sub.Origin.X + snap.localX*cosNew - snap.localY*sinNew
		newY := sub.Origin.Y + snap.localX*sinNew + snap.localY*cosNew
		newTheta := util.NormalizeAngle(newOt + snap.dt)

		rs := m.robots[rid]
		if rs == nil {
			continue
		}
		dx := newX - rs.CurrentPose.X
		dy := newY - rs.CurrentPose.Y
		if math.Hypot(dx, dy) > 0.005 || math.Abs(util.NormalizeAngle(newTheta-rs.CurrentPose.Theta)) > 0.001 {
			m.pendingCorrections[rid] = Pose{X: newX, Y: newY, Theta: newTheta}
		}
	}

	finalX := m.submaps[closedID].Origin.X
	finalY := m.submaps[closedID].Origin.Y
	finalTheta := m.submaps[closedID].Origin.Theta

	return finalX - oldX, finalY - oldY, util.NormalizeAngle(finalTheta - oldTheta)
}
