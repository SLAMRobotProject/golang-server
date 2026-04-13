package slam

import "math"

const (
	// Minimum wall cells the outgoing submap must have before attempting any match.
	matchMinWallCells = 250

	// Search window — wider than before to catch more drift.
	matchSearchXY    = 0.50 // ±50 cm
	matchSearchTheta = 0.26 // ±15°

	// Step sizes — fine enough for 5 cm cells; theta ~3°.
	matchStepXY    = GridRes // 5 cm
	matchStepTheta = 0.052   // ~3°

	// Gaussian sigma for the likelihood field (20 cm gives smooth gradients).
	likelihoodSigma = 0.20 // metres

	// How many finished submaps (before the outgoing one) to merge into the
	// sequential reference. More submaps = denser reference = better score signal.
	seqRefCount = 4

	// Accept a sequential match if average likelihood >= this.
	matchMinAvgScore = 0.15

	// Loop closure: skip submaps closer than this many index positions.
	loopMinSubmapGap = 5

	// Loop closure: only check submaps whose origin is within sensor range.
	loopMaxOriginDist = 3.0

	// Accept a loop closure if average likelihood >= this (stricter than seq).
	loopMinAvgScore = 0.25
)

// -- Merged world-space likelihood field ----------------------------------
//
// All reference submap wall cells are projected into world-grid coordinates
// (the same GridWidth × GridHeight grid used by renderGlobal).  A single
// Gaussian is splatted per wall cell.  This lets any number of submaps
// contribute to one coherent reference without per-submap coordinate changes.

type worldLF [GridHeight][GridWidth]float32

// buildWorldLF creates a merged likelihood field in world-grid space from
// the supplied reference submaps.
func buildWorldLF(refs []*Submap) *worldLF {
	sigma := likelihoodSigma
	radiusCells := int(math.Ceil(3 * sigma / GridRes))
	sigSq := float32(2 * sigma * sigma)

	lf := new(worldLF)

	for _, s := range refs {
		for cy := 0; cy < SubSize; cy++ {
			for cx := 0; cx < SubSize; cx++ {
				if s.grid[cy][cx] <= 0.2 {
					continue
				}
				// World-grid position of this wall cell
				worldX := s.Origin.X + float64(cx-SubHalf)*GridRes
				worldY := s.Origin.Y - float64(cy-SubHalf)*GridRes
				wx := int(math.Round(worldX/GridRes)) + GridOffX
				wy := GridOffY - int(math.Round(worldY/GridRes))

				// Splat Gaussian around (wx, wy)
				for dy := -radiusCells; dy <= radiusCells; dy++ {
					ny := wy + dy
					if ny < 0 || ny >= GridHeight {
						continue
					}
					for dx := -radiusCells; dx <= radiusCells; dx++ {
						nx := wx + dx
						if nx < 0 || nx >= GridWidth {
							continue
						}
						d2 := float32(dx*dx+dy*dy) * float32(GridRes*GridRes)
						v := float32(math.Exp(float64(-d2 / sigSq)))
						if v > lf[ny][nx] {
							lf[ny][nx] = v
						}
					}
				}
			}
		}
	}
	return lf
}

// -- Candidate wall cells -------------------------------------------------

type wallCell struct{ wx, wy float64 } // world-space centre

func collectWalls(s *Submap) []wallCell {
	var walls []wallCell
	for cy := 0; cy < SubSize; cy++ {
		for cx := 0; cx < SubSize; cx++ {
			if s.grid[cy][cx] > 0.5 {
				wx := s.Origin.X + float64(cx-SubHalf)*GridRes
				wy := s.Origin.Y - float64(cy-SubHalf)*GridRes
				walls = append(walls, wallCell{wx, wy})
			}
		}
	}
	return walls
}

// -- Score a single (dx, dy, dTheta) candidate ---------------------------

func scoreCandidate(walls []wallCell, lf *worldLF,
	originX, originY, dx, dy, dt float64,
) float64 {
	cosT, sinT := math.Cos(dt), math.Sin(dt)
	ox := originX + dx
	oy := originY + dy
	var sum float64
	for _, w := range walls {
		// Rotate around candidate origin then translate
		rx := w.wx - ox
		ry := w.wy - oy
		worldX := ox + rx*cosT - ry*sinT
		worldY := oy + rx*sinT + ry*cosT

		// World-grid lookup
		gx := int(math.Round(worldX/GridRes)) + GridOffX
		gy := GridOffY - int(math.Round(worldY/GridRes))
		if gx < 0 || gx >= GridWidth || gy < 0 || gy >= GridHeight {
			continue
		}
		sum += float64(lf[gy][gx])
	}
	return sum / float64(len(walls))
}

// -- Exhaustive search ----------------------------------------------------

func search(walls []wallCell, lf *worldLF, originX, originY float64) (bestDX, bestDY, bestDT, bestScore float64) {
	bestScore = -1
	for dt := -matchSearchTheta; dt <= matchSearchTheta+1e-9; dt += matchStepTheta {
		for dy := -matchSearchXY; dy <= matchSearchXY+1e-9; dy += matchStepXY {
			for dx := -matchSearchXY; dx <= matchSearchXY+1e-9; dx += matchStepXY {
				s := scoreCandidate(walls, lf, originX, originY, dx, dy, dt)
				if s > bestScore {
					bestScore = s
					bestDX, bestDY, bestDT = dx, dy, dt
				}
			}
		}
	}
	return
}

// -- Sequential match -----------------------------------------------------

// tryMatchToPrev aligns the outgoing (last) submap against a merged reference
// built from the seqRefCount finished submaps before it.
// Returns the correction (dx, dy, dTheta) applied to the outgoing submap's origin,
// or (0,0,0) if no confident match was found.
func (m *OccupancyMap) tryMatchToPrev() (dx, dy, dTheta float64) {
	n := len(m.submaps)
	if n < 2 {
		return
	}
	cur := m.submaps[n-1]
	walls := collectWalls(cur)
	if len(walls) < matchMinWallCells {
		return
	}

	start := max(n-1-seqRefCount, 0)
	refs := m.submaps[start : n-1]
	lf := buildWorldLF(refs)

	bx, by, bt, score := search(walls, lf, cur.Origin.X, cur.Origin.Y)
	if score < matchMinAvgScore {
		return
	}
	cur.Origin.X += bx
	cur.Origin.Y += by
	cur.Origin.Theta += bt
	return bx, by, bt
}

// -- Loop closure ---------------------------------------------------------

// tryLoopClosure checks all non-adjacent submaps within sensor range for a
// high-quality match.  On success it distributes the correction linearly
// across the entire pose chain via applyLoopClosure.
func (m *OccupancyMap) tryLoopClosure() (dx, dy, dTheta float64) {
	n := len(m.submaps)
	if n < loopMinSubmapGap+1 {
		return
	}
	cur := m.submaps[n-1]
	walls := collectWalls(cur)
	if len(walls) < matchMinWallCells {
		return
	}

	bestScore := loopMinAvgScore
	bestAnchor := -1
	bestDX, bestDY, bestDT := 0.0, 0.0, 0.0

	for ai := 0; ai <= n-1-loopMinSubmapGap; ai++ {
		anchor := m.submaps[ai]

		// Spatial filter
		if math.Hypot(cur.Origin.X-anchor.Origin.X, cur.Origin.Y-anchor.Origin.Y) > loopMaxOriginDist {
			continue
		}

		// Merge anchor and its immediate neighbours for a denser reference
		lo := max(ai-2, 0)
		hi := min(ai+3, n-1)
		lf := buildWorldLF(m.submaps[lo:hi])

		dx, dy, dt, score := search(walls, lf, cur.Origin.X, cur.Origin.Y)
		if score > bestScore {
			bestScore = score
			bestAnchor = ai
			bestDX, bestDY, bestDT = dx, dy, dt
		}
	}

	if bestAnchor < 0 {
		return
	}
	m.applyLoopClosure(bestAnchor, n-1, bestDX, bestDY, bestDT)
	return bestDX, bestDY, bestDT
}
