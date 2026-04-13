package slam

// applyLoopClosure distributes a pose correction across a chain of submaps.
//
// When the robot returns to a previously mapped area, the accumulated drift
// between submap[anchorIdx] and submap[closeIdx] is (dx, dy, dTheta).
// Rather than snapping only the latest submap, we spread the correction
// linearly across every submap in the chain [anchorIdx+1 … closeIdx].
// This avoids sharp discontinuities in the global map.
//
//	anchorIdx: index of the old submap the loop was closed against (fixed, not moved)
//	closeIdx:  index of the outgoing submap where the closure was detected
//	dx, dy, dTheta: correction to apply at closeIdx (fraction applied to each intermediate)
func (m *OccupancyMap) applyLoopClosure(anchorIdx, closeIdx int, dx, dy, dTheta float64) {
	span := closeIdx - anchorIdx // number of steps to distribute over
	if span <= 0 {
		return
	}
	for i := anchorIdx + 1; i <= closeIdx; i++ {
		t := float64(i-anchorIdx) / float64(span)
		m.submaps[i].Origin.X += t * dx
		m.submaps[i].Origin.Y += t * dy
		m.submaps[i].Origin.Theta += t * dTheta
	}
}
