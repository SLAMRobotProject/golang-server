package slam

import (
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

// addSequentialEdge records the current consecutive-submap relative pose as a
// backbone constraint (weight 1.0).
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

// updateLastSequentialEdge updates ONLY the most recently added sequential edge
// to reflect the scan-matched odometry, rather than the raw EKF odometry.
func (m *OccupancyMap) updateLastSequentialEdge() {
	if len(m.edges) == 0 {
		return
	}
	e := &m.edges[len(m.edges)-1]

	// Safety check: Only update if it is a sequential edge (weight 1.0)
	if e.weightXY > 1.0 {
		return
	}
	f := m.submaps[e.from]
	t := m.submaps[e.to]

	locX, locY := f.Origin.GlobalToLocal(t.Origin.X, t.Origin.Y)

	e.dx = locX
	e.dy = locY
	e.dtheta = util.NormalizeAngle(t.Origin.Theta - f.Origin.Theta)
}

func (m *OccupancyMap) applyLoopClosure(anchorID, closedID int, shiftX, shiftY, shiftTheta float64) (float64, float64, float64) {
	anchor := m.submaps[anchorID]
	closed := m.submaps[closedID]

	oldX := closed.Origin.X
	oldY := closed.Origin.Y
	oldTheta := closed.Origin.Theta

	// 1. Calculate the globally corrected ideal pose of the new submap
	correctedX := oldX + shiftX
	correctedY := oldY + shiftY
	correctedTheta := util.NormalizeAngle(oldTheta + shiftTheta)

	// 2. Calculate the TRUE physical vector from the Anchor to this Corrected Pose
	locX, locY := anchor.Origin.GlobalToLocal(correctedX, correctedY)
	relDTheta := util.NormalizeAngle(correctedTheta - anchor.Origin.Theta)

	// 3. Add the true physical constraint to the Pose Graph
	m.edges = append(m.edges, poseEdge{
		from: anchorID, to: closedID,
		dx: locX, dy: locY, dtheta: relDTheta,
		weightXY:    3.0,
		weightTheta: 10.0,
	})

	m.optimizePoseGraph()

	// 4. Return the final shift Ceres applied so the robot's EKF can safely jump
	finalX := m.submaps[closedID].Origin.X
	finalY := m.submaps[closedID].Origin.Y
	finalTheta := m.submaps[closedID].Origin.Theta

	// Normalize the angle difference so the EKF never receives a 360+ degree jump
	return finalX - oldX, finalY - oldY, util.NormalizeAngle(finalTheta - oldTheta)
}
