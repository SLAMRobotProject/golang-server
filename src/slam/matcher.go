package slam

import (
	"fmt"
	util "golang-server/utilities"
	"math"

	"gonum.org/v1/gonum/optimize"
)

const (
	// Core gates
	// Minimum wall cells required before attempting any match.
	matchMinWallCells = 120
	// Number of finished submaps merged into the sequential reference.
	seqRefCount = 2
	// Minimum acceptance scores.
	matchMinAvgScore = 0.40
	loopMinAvgScore  = 0.35

	// Loop candidate filtering
	// Skip near-adjacent submaps to avoid redundant closures.
	loopMinSubmapGap = 3
	// Only test anchors close enough to plausibly overlap.
	loopMaxOriginDist = 3.5

	// Ambiguity rejection (top-2 hypotheses).
	loopHypoMinScoreGap    = 0.01
	loopHypoMinScoreRatio  = 1.05
	loopHypoPoseCloseXY    = 0.20  // metres
	loopHypoPoseCloseTheta = 0.070 // radians (~4°)

	// Geometric validation sampling budget.
	loopGeomMaxWalls = 220

	// NDT core model
	ndtGridCellSize   = 0.25 // metres
	ndtMinPointsCell  = 3
	ndtMaxIterations  = 20
	ndtConvergeTol    = 1e-4
	ndtRegularization = 1e-3

	// Sequential NDT search (default + retry/wide).
	ndtSeqCoarseXY        = 0.45
	ndtSeqCoarseTheta     = 0.20
	ndtSeqCoarseStepXY    = 0.10
	ndtSeqCoarseStepTheta = 0.035
	ndtSeqCoarseXYWide    = 1.20
	ndtSeqCoarseThetaWide = 0.52

	// Loop NDT search and drift-based expansion limits.
	ndtLoopCoarseXY        = 1.20
	ndtLoopCoarseTheta     = 0.52
	ndtLoopCoarseStepXY    = 0.20
	ndtLoopCoarseStepTheta = 0.087
	ndtLoopCoarseThetaMax  = 0.78

	loopDriftExpandThreshold = 1.0
	loopDriftMaxCoarseXY     = 2.50

	// Coarse search penalties (prefer smaller corrections when scores are close).
	ndtCoarseDistPenalty  = 0.06
	ndtCoarseThetaPenalty = 0.03

	// Geometry gates on matched support.
	seqGeomMinSupportRatio = 0.10
	seqGeomMinPCARatio     = 0.01
	seqGeomMinSpanMinor    = 0.18

	loopGeomMinSupportRatio = 0.12
	loopGeomMinPCARatio     = 0.05
	loopGeomMinSpanMinor    = 0.60
)

// -- NDT Structures -------------------------------------------------------

// NDTPoint2D is a 2D point with mean and covariance collected from occupied cells.
type NDTPoint struct {
	mean [2]float64    // [x, y]
	cov  [2][2]float64 // covariance matrix
}

// NDTGrid represents a grid of Normal Distributions Transform cells.
type NDTGrid struct {
	cells                  map[int64]*NDTPoint // (gridIdx) → *NDTPoint
	cellSize               float64
	minX, minY, maxX, maxY float64 // bounds
}

// gridKey returns a spatial hash key for (gx, gy) cell coordinates.
func gridKey(gx, gy int) int64 {
	return (int64(gx) << 32) | int64(uint32(gy))
}

// buildNDTGrid constructs a Normal Distributions Transform grid from reference submaps.
// Each non-empty cell in the NDT grid stores mean and covariance of wall points.
func buildNDTGrid(refs []*Submap, cellSize float64) *NDTGrid {
	grid := &NDTGrid{
		cells:    make(map[int64]*NDTPoint),
		cellSize: cellSize,
		minX:     math.MaxFloat64,
		minY:     math.MaxFloat64,
		maxX:     -math.MaxFloat64,
		maxY:     -math.MaxFloat64,
	}
	ndt := grid

	// Collect all wall points from reference submaps
	var allWalls []wallCell
	for _, s := range refs {
		walls := collectWalls(s)
		for _, w := range walls {
			allWalls = append(allWalls, w)

			// Update NDT grid bounds
			if w.wx < ndt.minX {
				ndt.minX = w.wx
			}
			if w.wx > ndt.maxX {
				ndt.maxX = w.wx
			}
			if w.wy < ndt.minY {
				ndt.minY = w.wy
			}
			if w.wy > ndt.maxY {
				ndt.maxY = w.wy
			}
		}
	}

	// Group points into cells by spatial hash
	cellPoints := make(map[int64][]wallCell)
	for _, w := range allWalls {
		gx := int(math.Floor(w.wx / cellSize))
		gy := int(math.Floor(w.wy / cellSize))
		key := gridKey(gx, gy)
		cellPoints[key] = append(cellPoints[key], w)
	}

	// Compute mean and covariance for each cell
	for key, pts := range cellPoints {
		if len(pts) < ndtMinPointsCell {
			continue
		}

		// Compute mean
		var meanX, meanY float64
		for _, p := range pts {
			meanX += p.wx
			meanY += p.wy
		}
		n := float64(len(pts))
		meanX /= n
		meanY /= n

		// Compute covariance
		var cov [2][2]float64
		for _, p := range pts {
			dx := p.wx - meanX
			dy := p.wy - meanY
			cov[0][0] += dx * dx
			cov[0][1] += dx * dy
			cov[1][0] += dy * dx
			cov[1][1] += dy * dy
		}
		cov[0][0] /= n
		cov[0][1] /= n
		cov[1][0] /= n
		cov[1][1] /= n

		// Add regularization for numerical stability
		cov[0][0] += ndtRegularization
		cov[1][1] += ndtRegularization

		ndt.cells[key] = &NDTPoint{
			mean: [2]float64{meanX, meanY},
			cov:  cov,
		}
	}

	return ndt
}

// inverseMatrix2x2 computes the inverse of a 2x2 matrix.
func inverseMatrix2x2(m [2][2]float64) ([2][2]float64, bool) {
	det := m[0][0]*m[1][1] - m[0][1]*m[1][0]
	if math.Abs(det) < 1e-10 {
		return [2][2]float64{}, false
	}
	invDet := 1.0 / det
	return [2][2]float64{
		{m[1][1] * invDet, -m[0][1] * invDet},
		{-m[1][0] * invDet, m[0][0] * invDet},
	}, true
}

// computeNDTLikelihood computes the Gaussian probability score for current walls
// under the reference NDT distribution at a given pose (dx, dy, dtheta).
// Returns: (score, gradient_dx, gradient_dy, gradient_dtheta, hessian)
func computeNDTLikelihood(walls []wallCell, ndt *NDTGrid, originX, originY, dx, dy, dtheta float64) (score, gradDX, gradDY, gradTheta float64, hessian [3][3]float64, count int) {
	cosT := math.Cos(dtheta)
	sinT := math.Sin(dtheta)
	var sumScore float64
	count = 0
	hessian = [3][3]float64{}

	for _, w := range walls {
		// Transform wall point to candidate pose
		rx := w.wx - originX
		ry := w.wy - originY
		tx := originX + dx + rx*cosT - ry*sinT
		ty := originY + dy + rx*sinT + ry*cosT

		// Find nearest NDT cell
		gx := int(math.Floor(tx / ndt.cellSize))
		gy := int(math.Floor(ty / ndt.cellSize))
		key := gridKey(gx, gy)

		cell, ok := ndt.cells[key]
		if !ok {
			continue // point outside ND grid
		}

		// Compute Mahalanobis distance and likelihood
		dx_maha := tx - cell.mean[0]
		dy_maha := ty - cell.mean[1]

		invCov, ok := inverseMatrix2x2(cell.cov)
		if !ok {
			continue
		}

		// Mahalanobis distance squared
		d2 := dx_maha*(invCov[0][0]*dx_maha+invCov[0][1]*dy_maha) +
			dy_maha*(invCov[1][0]*dx_maha+invCov[1][1]*dy_maha)

		// Gaussian likelihood: exp(-0.5 * d2)
		likelihood := math.Exp(-0.5 * d2)
		sumScore += likelihood
		count++

		// Compute Jacobian of transformation: d(tx,ty)/d(dx,dy,dtheta)
		jtx_ddx := 1.0
		jtx_ddt := -rx*sinT - ry*cosT

		jty_ddy := 1.0
		jty_ddt := rx*cosT - ry*sinT

		// Gradient of likelihood w.r.t. pose: d(likelihood)/d(x,y,theta)
		// = -likelihood * M^{-1} * d(x,y) where M^{-1} is invCov
		factor := -likelihood * d2 // Simplified gradient direction
		gradDX += factor * (invCov[0][0]*dx_maha + invCov[0][1]*dy_maha) * jtx_ddx
		gradDY += factor * (invCov[1][0]*dx_maha + invCov[1][1]*dy_maha) * jty_ddy
		gradTheta += factor * (invCov[0][0]*dx_maha + invCov[0][1]*dy_maha) * jtx_ddt
		gradTheta += factor * (invCov[1][0]*dx_maha + invCov[1][1]*dy_maha) * jty_ddt
	}

	score = sumScore
	return
}

// ndtObjective returns negative score (for minimization) with mild regularization
// so quasi-Newton steps stay in a physically reasonable local basin.
func ndtObjective(walls []wallCell, ndt *NDTGrid, originX, originY float64, x []float64) float64 {
	dx, dy, dt := x[0], x[1], util.NormalizeAngle(x[2])
	score, _, _, _, _, count := computeNDTLikelihood(walls, ndt, originX, originY, dx, dy, dt)
	if count < 10 {
		return 1e3
	}
	reg := 1e-3*(dx*dx+dy*dy) + 1e-2*(dt*dt)
	return -score + reg
}

func ndtNumericGrad(dst []float64, walls []wallCell, ndt *NDTGrid, originX, originY float64, x []float64) {
	if len(dst) < 3 {
		return
	}
	step := [3]float64{1e-3, 1e-3, 5e-4}
	for i := 0; i < 3; i++ {
		xp := []float64{x[0], x[1], x[2]}
		xm := []float64{x[0], x[1], x[2]}
		xp[i] += step[i]
		xm[i] -= step[i]
		if i == 2 {
			xp[i] = util.NormalizeAngle(xp[i])
			xm[i] = util.NormalizeAngle(xm[i])
		}
		fp := ndtObjective(walls, ndt, originX, originY, xp)
		fm := ndtObjective(walls, ndt, originX, originY, xm)
		dst[i] = (fp - fm) / (2 * step[i])
	}
}

// optimizeNDTPose uses quasi-Newton (BFGS) to refine pose (dx, dy, dtheta)
// from a coarse-search initialization.
func optimizeNDTPose(walls []wallCell, ndt *NDTGrid, originX, originY, initDX, initDY, initTheta float64) (bestDX, bestDY, bestTheta, bestScore float64) {
	init := []float64{initDX, initDY, util.NormalizeAngle(initTheta)}
	problem := optimize.Problem{
		Func: func(x []float64) float64 {
			return ndtObjective(walls, ndt, originX, originY, x)
		},
		Grad: func(grad, x []float64) {
			ndtNumericGrad(grad, walls, ndt, originX, originY, x)
		},
	}
	settings := optimize.Settings{
		MajorIterations:   ndtMaxIterations,
		FuncEvaluations:   ndtMaxIterations * 20,
		GradientThreshold: ndtConvergeTol,
	}
	res, err := optimize.Minimize(problem, init, &settings, &optimize.BFGS{})
	if err == nil && len(res.X) >= 3 {
		bestDX = res.X[0]
		bestDY = res.X[1]
		bestTheta = util.NormalizeAngle(res.X[2])
		bestScore, _, _, _, _, _ = computeNDTLikelihood(walls, ndt, originX, originY, bestDX, bestDY, bestTheta)
		return
	}

	// Fallback to initial coarse candidate if BFGS fails.
	bestDX, bestDY, bestTheta = init[0], init[1], init[2]
	bestScore, _, _, _, _, _ = computeNDTLikelihood(walls, ndt, originX, originY, bestDX, bestDY, bestTheta)
	return
}

func ndtSupportAtPose(walls []wallCell, ndt *NDTGrid, originX, originY, dx, dy, dt float64) (score float64, support, total int, supportRatio float64) {
	score, _, _, _, _, support = computeNDTLikelihood(walls, ndt, originX, originY, dx, dy, dt)
	total = len(walls)
	if total > 0 {
		supportRatio = float64(support) / float64(total)
	}
	return
}

// -- Geometric angular diversity -----------------------------------------

// scanGeometricDiversity computes the Principal Components (Covariance) of the wall points.
// Returns the ratio of the smallest eigenvalue to the largest.
// Corridor ≈ 0.0, Corner/Room ≈ 0.3 - 1.0.
func scanGeometricDiversity(walls []wallCell) float64 {
	if len(walls) < 5 {
		return 0.0
	}

	// 1. Find the centroid of the walls
	var cx, cy float64
	for _, w := range walls {
		cx += w.wx
		cy += w.wy
	}
	cx /= float64(len(walls))
	cy /= float64(len(walls))

	// 2. Compute the 2x2 covariance matrix
	var c00, c01, c11 float64
	for _, w := range walls {
		dx := w.wx - cx
		dy := w.wy - cy
		c00 += dx * dx
		c01 += dx * dy
		c11 += dy * dy
	}

	// 3. Calculate the eigenvalues
	trace := c00 + c11
	det := c00*c11 - c01*c01
	discriminant := math.Sqrt(math.Max(0, trace*trace-4*det))

	eig1 := (trace + discriminant) / 2.0 // Largest variance (along the wall axis)
	eig2 := (trace - discriminant) / 2.0 // Smallest variance (perpendicular to wall)

	if eig1 < 1e-6 {
		return 0.0
	}
	// Return the ratio. A low ratio (< 0.05) means it is a straight line.
	return eig2 / eig1
}

type ndtGeomStats struct {
	Support  int
	Total    int
	Ratio    float64
	PCARatio float64
	SpanX    float64
	SpanY    float64
}

func ndtMatchGeometry(walls []wallCell, ndt *NDTGrid, originX, originY, dx, dy, dt float64) ndtGeomStats {
	stats := ndtGeomStats{Total: len(walls)}
	if len(walls) == 0 {
		return stats
	}

	cosT, sinT := math.Cos(dt), math.Sin(dt)
	minX, minY := math.MaxFloat64, math.MaxFloat64
	maxX, maxY := -math.MaxFloat64, -math.MaxFloat64

	// We store the transformed points that successfully found support in the NDT grid.
	// We only want to run PCA on the walls that actually match!
	supportedPoints := make([]wallCell, 0, len(walls))

	for _, w := range walls {
		rx := w.wx - originX
		ry := w.wy - originY
		tx := originX + dx + rx*cosT - ry*sinT
		ty := originY + dy + rx*sinT + ry*cosT

		gx := int(math.Floor(tx / ndt.cellSize))
		gy := int(math.Floor(ty / ndt.cellSize))

		if _, ok := ndt.cells[gridKey(gx, gy)]; !ok {
			continue
		}

		stats.Support++

		// Save the transformed coordinates for the PCA calculation
		supportedPoints = append(supportedPoints, wallCell{wx: tx, wy: ty})

		// Track bounds for SpanX / SpanY
		if tx < minX {
			minX = tx
		}
		if tx > maxX {
			maxX = tx
		}
		if ty < minY {
			minY = ty
		}
		if ty > maxY {
			maxY = ty
		}
	}

	if stats.Total > 0 {
		stats.Ratio = float64(stats.Support) / float64(stats.Total)
	}

	// Calculate basic spans if we have at least a line segment
	if stats.Support >= 2 {
		stats.SpanX = maxX - minX
		stats.SpanY = maxY - minY
	}

	// Calculate PCA Ratio (requires at least 5 points to be mathematically stable)
	if stats.Support >= 5 {
		// 1. Calculate Centroid
		var cx, cy float64
		for _, p := range supportedPoints {
			cx += p.wx
			cy += p.wy
		}
		cx /= float64(stats.Support)
		cy /= float64(stats.Support)

		// 2. Compute Covariance Matrix
		var c00, c01, c11 float64
		for _, p := range supportedPoints {
			diffX := p.wx - cx
			diffY := p.wy - cy
			c00 += diffX * diffX
			c01 += diffX * diffY
			c11 += diffY * diffY
		}

		// 3. Extract Eigenvalues
		trace := c00 + c11
		det := c00*c11 - c01*c01
		discriminant := math.Sqrt(math.Max(0, trace*trace-4*det))

		eig1 := (trace + discriminant) / 2.0 // Largest variance (Length)
		eig2 := (trace - discriminant) / 2.0 // Smallest variance (Width)

		if eig1 > 1e-6 {
			stats.PCARatio = eig2 / eig1
		} else {
			stats.PCARatio = 0.0
		}
	} else {
		stats.PCARatio = 0.0 // Not enough points to prove it isn't a corridor
	}

	return stats
}

func passesGeomValidator(stats ndtGeomStats, minRatio, minPCARatio, minMinorSpan float64) (bool, string) {
	if stats.Total < 30 || stats.Support < 30 {
		return false, "few-inliers"
	}
	if stats.Ratio < minRatio {
		return false, "low-support"
	}
	if stats.PCARatio < minPCARatio {
		return false, "low-pca-ratio"
	}
	if math.Min(stats.SpanX, stats.SpanY) < minMinorSpan {
		return false, "low-2d-span"
	}
	return true, "ok"
}

func searchNDTWindow(walls []wallCell, ndt *NDTGrid, originX, originY, rangeXY, stepXY, rangeTheta, stepTheta float64) (bestDX, bestDY, bestDT, bestScore float64) {
	bestScore = -1
	minSupport := max(8, len(walls)/12)
	for dt := -rangeTheta; dt <= rangeTheta+1e-9; dt += stepTheta {
		for dy := -rangeXY; dy <= rangeXY+1e-9; dy += stepXY {
			for dx := -rangeXY; dx <= rangeXY+1e-9; dx += stepXY {
				s, _, count, ratio := ndtSupportAtPose(walls, ndt, originX, originY, dx, dy, dt)
				if count < minSupport || ratio < 0.08 {
					continue
				}
				penalty := float64(count) * (ndtCoarseDistPenalty*math.Hypot(dx, dy) + ndtCoarseThetaPenalty*math.Abs(dt))
				adj := s - penalty
				if adj > bestScore {
					bestScore = adj
					bestDX, bestDY, bestDT = dx, dy, dt
				}
			}
		}
	}
	return
}

// searchNDT performs multi-scale coarse-to-fine pose estimation using NDT.
// Coarse pass uses 4-point cross search, fine pass optimizes around best coarse result.
func searchNDTAdaptive(walls []wallCell, ndt *NDTGrid, originX, originY, coarseXY, coarseTheta float64) (bestDX, bestDY, bestDT, bestScore float64) {
	coarseWalls := downsampleWalls(walls, 220)
	bestDX, bestDY, bestDT, bestScore = searchNDTWindow(
		coarseWalls,
		ndt,
		originX,
		originY,
		coarseXY,
		ndtSeqCoarseStepXY,
		coarseTheta,
		ndtSeqCoarseStepTheta,
	)

	// Fine pass: optimize around best coarse result
	if bestScore > 0 {
		bestDX, bestDY, bestDT, bestScore = optimizeNDTPose(walls, ndt, originX, originY, bestDX, bestDY, bestDT)
	}

	return
}

func searchNDT(walls []wallCell, ndt *NDTGrid, originX, originY float64) (bestDX, bestDY, bestDT, bestScore float64) {
	return searchNDTAdaptive(walls, ndt, originX, originY, ndtSeqCoarseXY, ndtSeqCoarseTheta)
}

// searchLoopNDT performs loop-closure pose estimation with wider search space.
// coarseXY overrides ndtLoopCoarseXY when > 0, allowing the caller to expand
// the search range when accumulated drift is large.
func searchLoopNDT(walls []wallCell, ndt *NDTGrid, originX, originY, coarseXY, coarseTheta float64) (bestDX, bestDY, bestDT, bestScore float64) {
	if coarseXY <= 0 {
		coarseXY = ndtLoopCoarseXY
	}
	if coarseTheta <= 0 {
		coarseTheta = ndtLoopCoarseTheta
	}
	coarseWalls := downsampleWalls(walls, 240)
	bestDX, bestDY, bestDT, bestScore = searchNDTWindow(
		coarseWalls,
		ndt,
		originX,
		originY,
		coarseXY,
		ndtLoopCoarseStepXY,
		coarseTheta,
		ndtLoopCoarseStepTheta,
	)

	// Fine pass: optimize around best coarse result
	if bestScore > 0 {
		bestDX, bestDY, bestDT, bestScore = optimizeNDTPose(walls, ndt, originX, originY, bestDX, bestDY, bestDT)
	}

	return
}

// -- Candidate wall cells -------------------------------------------------

type wallCell struct{ wx, wy float64 } // world-space centre

func collectWalls(s *Submap) []wallCell {
	var walls []wallCell
	for key, v := range s.grid {
		if v > 0.5 {
			lx := float64(key.X) * GridRes
			ly := -float64(key.Y) * GridRes
			wx, wy := s.Origin.LocalToGlobal(lx, ly)
			walls = append(walls, wallCell{wx, wy})
		}
	}
	return walls
}

func downsampleWalls(walls []wallCell, maxCount int) []wallCell {
	if len(walls) <= maxCount || maxCount <= 0 {
		return walls
	}
	stride := int(math.Ceil(float64(len(walls)) / float64(maxCount)))
	if stride < 1 {
		stride = 1
	}
	out := make([]wallCell, 0, maxCount)
	for i := 0; i < len(walls); i += stride {
		out = append(out, walls[i])
	}
	return out
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
	cur := m.active
	walls := collectWalls(cur)
	if len(walls) < matchMinWallCells {
		return
	}

	start := max(n-1-seqRefCount, 0)
	refs := m.submaps[start : n-1]
	ndt := buildNDTGrid(refs, ndtGridCellSize)

	bx, by, bt, sumScore := searchNDT(walls, ndt, cur.Origin.X, cur.Origin.Y)
	stats := ndtMatchGeometry(walls, ndt, cur.Origin.X, cur.Origin.Y, bx, by, bt)

	qualityScore := 0.0
	if stats.Support > 0 {
		qualityScore = sumScore / float64(stats.Support)
	}

	okGeom, geomReason := passesGeomValidator(stats, seqGeomMinSupportRatio, seqGeomMinPCARatio, seqGeomMinSpanMinor)

	if qualityScore < matchMinAvgScore || !okGeom {
		wbx, wby, wbt, wscore := searchNDTAdaptive(walls, ndt, cur.Origin.X, cur.Origin.Y, ndtSeqCoarseXYWide, ndtSeqCoarseThetaWide)
		wstats := ndtMatchGeometry(walls, ndt, cur.Origin.X, cur.Origin.Y, wbx, wby, wbt)
		wokGeom, _ := passesGeomValidator(wstats, seqGeomMinSupportRatio, seqGeomMinPCARatio, seqGeomMinSpanMinor)

		if wscore > sumScore && wokGeom {
			bx, by, bt, sumScore = wbx, wby, wbt, wscore
			stats, okGeom = wstats, true
			if stats.Support > 0 {
				qualityScore = sumScore / float64(stats.Support)
			}
		}
	}

	if qualityScore < matchMinAvgScore {
		fmt.Printf("[SLAM SEQ] submap %d: rejected (quality=%.3f < %.2f, sum=%.1f)\n", n-1, qualityScore, matchMinAvgScore, sumScore)
		return
	}
	if !okGeom {
		fmt.Printf("[SLAM SEQ] submap %d: rejected geom (%s support=%.2f pcaRatio=%.3f span=%.2fm×%.2fm)\n",
			n-1, geomReason, stats.Ratio, stats.PCARatio, stats.SpanX, stats.SpanY)
		return
	}

	shapeRatio := scanGeometricDiversity(walls)

	if shapeRatio < 0.02 { // Highly linear (Corridor)
		fmt.Printf("[SLAM SEQ] submap %d: low diversity (PCA ratio=%.3f) — rejected match\n", n-1, shapeRatio)
		return 0, 0, 0
	}

	fmt.Printf("[SLAM SEQ] submap %d: dx=%.3f dy=%.3f dθ=%.1f° score=%.3f shapeRatio=%.3f inlierPCA=%.3f span=%.2fm×%.2fm\n",
		n-1, bx, by, bt*180/math.Pi, qualityScore, shapeRatio, stats.PCARatio, stats.SpanX, stats.SpanY)

	cur.Origin.X += bx
	cur.Origin.Y += by
	cur.Origin.Theta += bt

	m.updateLastSequentialEdge()

	return bx, by, bt
}

// -- Loop closure ---------------------------------------------------------

// tryLoopClosure checks all non-adjacent submaps within sensor range for a
// high-quality match.  On success it distributes the correction linearly
// across the entire pose chain via applyLoopClosure.
// fired is true when a closure was accepted, regardless of correction magnitude.
func (m *OccupancyMap) tryLoopClosure() (dx, dy, dTheta float64, fired bool) {
	n := len(m.submaps)
	if n <= loopMinSubmapGap {
		return
	}

	cur := m.active
	walls := collectWalls(cur)
	if len(walls) < matchMinWallCells {
		return
	}

	bestScore := -math.MaxFloat64
	bestAnchor := -1
	bestDX, bestDY, bestDT := 0.0, 0.0, 0.0

	// Track second-best for multi-hypothesis ambiguity check.
	secondScore := -math.MaxFloat64
	secondDX, secondDY, secondDT := 0.0, 0.0, 0.0

	for ai := 0; ai <= n-loopMinSubmapGap; ai++ {
		anchor := m.submaps[ai]

		if !anchor.IsKeyframe {
			continue
		}

		if math.Hypot(cur.Origin.X-anchor.Origin.X, cur.Origin.Y-anchor.Origin.Y) > loopMaxOriginDist {
			continue
		}

		lo := max(ai-2, 0)
		hi := min(ai+3, n-1)
		ndt := buildNDTGrid(m.submaps[lo:hi], ndtGridCellSize)

		// Expand coarse search range proportionally when drift to anchor is large.
		drift := math.Hypot(cur.Origin.X-anchor.Origin.X, cur.Origin.Y-anchor.Origin.Y)
		coarseXY := ndtLoopCoarseXY
		coarseTheta := ndtLoopCoarseTheta
		if drift > loopDriftExpandThreshold {
			coarseXY = math.Min(drift*0.9, loopDriftMaxCoarseXY)
			t := ndtLoopCoarseTheta * (1.0 + 0.35*(drift-loopDriftExpandThreshold))
			if t > ndtLoopCoarseThetaMax {
				t = ndtLoopCoarseThetaMax
			}
			coarseTheta = t
		}
		ldx, ldy, ldt, score := searchLoopNDT(walls, ndt, cur.Origin.X, cur.Origin.Y, coarseXY, coarseTheta)
		if score > bestScore {
			// Demote current best to second before updating.
			secondScore = bestScore
			secondDX, secondDY, secondDT = bestDX, bestDY, bestDT
			bestScore = score
			bestAnchor = ai
			bestDX, bestDY, bestDT = ldx, ldy, ldt
		} else if score > secondScore {
			secondScore = score
			secondDX, secondDY, secondDT = ldx, ldy, ldt
		}
	}

	if bestAnchor < 0 {
		return 0, 0, 0, false
	}

	// Multi-hypothesis ambiguity check
	if secondScore >= 0 {
		scoreSimilar := secondScore*loopHypoMinScoreRatio >= bestScore
		poseFar := math.Hypot(bestDX-secondDX, bestDY-secondDY) > loopHypoPoseCloseXY ||
			math.Abs(normalizeAngle(bestDT-secondDT)) > loopHypoPoseCloseTheta
		if scoreSimilar && poseFar {
			fmt.Printf("[SLAM LC] submap %d: rejected (ambiguous — bestSum=%.1f secondSum=%.1f poseDist=%.2fm)\n",
				n-1, bestScore, secondScore, math.Hypot(bestDX-secondDX, bestDY-secondDY))
			return 0, 0, 0, false
		}
	}

	shapeRatio := scanGeometricDiversity(walls)
	if shapeRatio < 0.05 {
		fmt.Printf("[SLAM LC] submap %d: rejected (geometric diversity=%.3f < %.3f, single-wall)\n",
			n-1, shapeRatio, 0.05)
		return 0, 0, 0, false
	}

	lo := max(bestAnchor-2, 0)
	hi := min(bestAnchor+3, n-1)
	bestNDT := buildNDTGrid(m.submaps[lo:hi], ndtGridCellSize)

	exactSum, _, _, _, _, exactSupport := computeNDTLikelihood(walls, bestNDT, cur.Origin.X, cur.Origin.Y, bestDX, bestDY, bestDT)
	qualityScore := 0.0
	if exactSupport > 0 {
		qualityScore = exactSum / float64(exactSupport)
	}

	if qualityScore < loopMinAvgScore {
		fmt.Printf("[SLAM LC] submap %d: no closure (quality=%.3f < %.2f)\n", n-1, qualityScore, loopMinAvgScore)
		return 0, 0, 0, false
	}

	geomWalls := downsampleWalls(walls, loopGeomMaxWalls)
	stats := ndtMatchGeometry(geomWalls, bestNDT, cur.Origin.X, cur.Origin.Y, bestDX, bestDY, bestDT)
	okGeom, geomReason := passesGeomValidator(stats, loopGeomMinSupportRatio, loopGeomMinPCARatio, loopGeomMinSpanMinor)
	if !okGeom {
		fmt.Printf("[SLAM LC] submap %d: rejected geom (%s support=%.2f pcaRatio=%.3f span=%.2fm×%.2fm)\n",
			n-1, geomReason, stats.Ratio, stats.PCARatio, stats.SpanX, stats.SpanY)
		return 0, 0, 0, false
	}

	fmt.Printf("[SLAM LC] CLOSED: submap %d → anchor %d  dx=%.3f dy=%.3f dθ=%.1f°  score=%.3f  shapeRatio=%.3f inlierPCA=%.3f span=%.2fm×%.2fm\n",
		n-1, bestAnchor, bestDX, bestDY, bestDT*180/math.Pi, bestScore, shapeRatio, stats.PCARatio, stats.SpanX, stats.SpanY)

	optDX, optDY, optDT := m.applyLoopClosure(bestAnchor, n-1, bestDX, bestDY, bestDT)
	return optDX, optDY, optDT, true
}
