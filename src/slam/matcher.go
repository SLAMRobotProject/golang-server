package slam

import (
	"fmt"
	util "golang-server/utilities"
	"math"

	"gonum.org/v1/gonum/optimize"
)

const (
	// ==========================================
	// Core gates
	// ==========================================
	// Minimum number of wall cells a submap needs before attempting to match it.
	matchMinWallCells = 50
	// How many historic submaps to stitch together to form the "reference map" for sequential matching.
	seqRefCount = 2
	// The minimum average likelihood score (0.0 to 1.0) needed to accept a sequential frame-to-frame match.
	matchMinAvgScore = 0.40
	// The minimum average likelihood score (0.0 to 1.0) needed to accept a Loop Closure match.
	loopMinAvgScore = 0.38

	// ==========================================
	// Loop candidate filtering
	// ==========================================
	// The minimum number of submaps that must have been created between the 'anchor' and the current submap to attempt a loop closure. (Also acts as cooldown length).
	loopMinSubmapGap = 4
	// Maximum physical distance (metres) between the current submap origin and historic anchor origin to even test it.
	loopMaxOriginDist = 2.0

	// ==========================================
	// Ambiguity rejection (Checking the top-2 hypotheses to prevent false positive match)
	// ==========================================
	// If the difference between the best and second-best NDT score is smaller than this, the match might be ambiguous.
	loopHypoMinScoreGap = 0.01
	// The ratio check for comparing the best score to the second-best score.
	loopHypoMinScoreRatio = 1.05
	// Distance physically separating two confusing hypotheses (metres). If they are further apart than this and score is close, REJECT.
	loopHypoPoseCloseXY = 0.20
	// Angle separating two confusing hypotheses (radians).
	loopHypoPoseCloseTheta = 0.070

	// Maximum points used when checking the geometry of a loop closure to keep CPU load low.
	loopGeomMaxWalls = 220

	// ==========================================
	// NDT (Normal Distributions Transform) model settings
	// ==========================================
	// The bin size of the probability grid in metres. A smaller grid means higher precision but tighter search requirements.
	ndtGridCellSize = 0.25
	// Minimum number of lidar points required in a grid cell to calculate standard deviation and mean.
	ndtMinPointsCell = 3
	// Optimizer (BFGS) limits for fine-tuning a match locally.
	ndtMaxIterations = 20
	// Stopping criteria for the BFGS optimizer (gradient tolerance).
	ndtConvergeTol = 1e-4
	// Small value added to the Diagonal of covariance matrices to avoid dividing by zero on perfectly straight walls.
	ndtRegularization = 1e-3

	// ==========================================
	// Sequential NDT parameter grid search (Coarse search before optimizer)
	// ==========================================
	// Search radius in X and Y (± metres).
	ndtSeqCoarseXY = 0.45
	// Search radius in Theta (± radians).
	ndtSeqCoarseTheta = 0.14
	// Search steps in X and Y (metres).
	ndtSeqCoarseStepXY = 0.10
	// Search steps in Theta (radians).
	ndtSeqCoarseStepTheta = 0.035

	// ==========================================
	// Loop Closure NDT parameter grid search
	// ==========================================
	// Initial search radius (± metres) for Loop Closure.
	ndtLoopCoarseXY = 1.20
	// Initial search radius (± radians) for Loop Closure.
	ndtLoopCoarseTheta = 0.52
	// Step size for XY search.
	ndtLoopCoarseStepXY = 0.20
	// Step size for Theta search.
	ndtLoopCoarseStepTheta = 0.087
	// Hard cap for how huge of an angle sweep to allow Loop Closure to do.
	ndtLoopCoarseThetaMax = 0.78

	// If the current submap is structurally further than this limit from the anchor (due to pure drift over time), the Coarse bounds expand dynamically.
	loopDriftExpandThreshold = 1.0
	// Hard cap on the expanded XY search radius to prevent out-of-memory/CPU explosions.
	loopDriftMaxCoarseXY = 2.50

	// ==========================================
	// Coarse search penalties (Forces the system to prefer smaller jumps unless the score is overwhelmingly better)
	// ==========================================
	// Penalty multiplicator per swept metre for sequential matches.
	ndtSeqCoarseDistPenalty = 0.06
	// High penalty multiper mapped per swept radian; prevents the submap jumping angles wildly on straight walls.
	ndtSeqCoarseThetaPenalty = 0.09

	// Loop closures get lower penalties, allowing the map to snap larger distances and angles safely.
	ndtLoopCoarseDistPenalty  = 0.06
	ndtLoopCoarseThetaPenalty = 0.03

	// ==========================================
	// Geometry gates on matched support points (Prevents the "Corridor/Aperture Problem")
	// ==========================================
	// Sequential Geometry settings
	// % of points that must have cleanly fallen into an ND-Grid cell
	seqGeomMinSupportRatio = 0.10
	// PCA (Principal Component Analysis) bound. A ratio measuring Point-Cloud-Width / Point-Cloud-Length. >0.03 prevents slipping on totally flat walls.
	seqGeomMinPCARatio = 0.02
	// Absolute minimum physical spread (metre) of the walls in the shortest direction.
	seqGeomMinSpanMinor = 0.18

	// Loop Closure Geometry Settings (Stricter bounds because teleporting across the map needs higher confidence)
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

func searchNDTWindow(walls []wallCell, ndt *NDTGrid, originX, originY, rangeXY, stepXY, rangeTheta, stepTheta, distPenalty, thetaPenalty float64) (bestDX, bestDY, bestDT, bestScore float64) {
	bestScore = -1
	minSupport := max(8, len(walls)/12)
	for dt := -rangeTheta; dt <= rangeTheta+1e-9; dt += stepTheta {
		for dy := -rangeXY; dy <= rangeXY+1e-9; dy += stepXY {
			for dx := -rangeXY; dx <= rangeXY+1e-9; dx += stepXY {
				s, _, count, ratio := ndtSupportAtPose(walls, ndt, originX, originY, dx, dy, dt)
				if count < minSupport || ratio < 0.08 {
					continue
				}
				penalty := float64(count) * (distPenalty*math.Hypot(dx, dy) + thetaPenalty*math.Abs(dt))
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
		ndtSeqCoarseDistPenalty,
		ndtSeqCoarseThetaPenalty,
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
		ndtLoopCoarseDistPenalty,
		ndtLoopCoarseThetaPenalty,
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

// tryMatchToPrev aligns the outgoing submap for robotID against a merged reference
// built from the seqRefCount most-recent same-robot finalized submaps before it.
// Returns the correction (dx, dy, dTheta) applied to the outgoing submap's origin,
// or (0,0,0) if no confident match was found.
func (m *OccupancyMap) tryMatchToPrev(robotID int) (dx, dy, dTheta float64) {
	n := len(m.submaps)
	cur := m.activeSubmaps[robotID]
	if cur == nil || n < 2 {
		return
	}
	walls := collectWalls(cur)
	if len(walls) < matchMinWallCells {
		return
	}

	// Collect the last seqRefCount finalized submaps from the same robot (excluding cur).
	var refs []*Submap
	for i := n - 2; i >= 0 && len(refs) < seqRefCount; i-- {
		if m.submaps[i].RobotID == robotID {
			refs = append([]*Submap{m.submaps[i]}, refs...)
		}
	}
	if len(refs) == 0 {
		return
	}
	ndt := buildNDTGrid(refs, ndtGridCellSize)

	bx, by, bt, sumScore := searchNDT(walls, ndt, cur.Origin.X, cur.Origin.Y)
	stats := ndtMatchGeometry(walls, ndt, cur.Origin.X, cur.Origin.Y, bx, by, bt)

	qualityScore := 0.0
	if stats.Support > 0 {
		qualityScore = sumScore / float64(stats.Support)
	}

	okGeom, geomReason := passesGeomValidator(stats, seqGeomMinSupportRatio, seqGeomMinPCARatio, seqGeomMinSpanMinor)

	if qualityScore < matchMinAvgScore {
		fmt.Printf("[SLAM SEQ] submap %d: rejected (quality=%.3f < %.2f, sum=%.1f)\n", cur.ID, qualityScore, matchMinAvgScore, sumScore)
		return
	}
	if !okGeom {
		fmt.Printf("[SLAM SEQ] submap %d: rejected geom (%s support=%.2f pcaRatio=%.3f span=%.2fm×%.2fm)\n",
			cur.ID, geomReason, stats.Ratio, stats.PCARatio, stats.SpanX, stats.SpanY)
		return
	}

	shapeRatio := scanGeometricDiversity(walls)

	if shapeRatio < 0.02 { // Highly linear (Corridor)
		fmt.Printf("[SLAM SEQ] submap %d: low diversity (PCA ratio=%.3f) — rejected match\n", cur.ID, shapeRatio)
		return 0, 0, 0
	}

	fmt.Printf("[SLAM SEQ] submap %d: dx=%.3f dy=%.3f dθ=%.1f° score=%.3f shapeRatio=%.3f inlierPCA=%.3f span=%.2fm×%.2fm\n",
		cur.ID, bx, by, bt*180/math.Pi, qualityScore, shapeRatio, stats.PCARatio, stats.SpanX, stats.SpanY)

	cur.Origin.X += bx
	cur.Origin.Y += by
	cur.Origin.Theta += bt

	m.updateLastSequentialEdge(robotID)

	return bx, by, bt
}

// -- Loop closure ---------------------------------------------------------

// tryLoopClosure searches all keyframe submaps (including other robots) for a
// high-quality match against robotID's outgoing submap.  On success it applies
// the pose-graph correction via applyLoopClosure, which also stores pending
// corrections for any other robots whose submaps were moved by Ceres.
// fired is true when a closure was accepted, regardless of correction magnitude.
func (m *OccupancyMap) tryLoopClosure(robotID int) (dx, dy, dTheta float64, fired bool) {
	n := len(m.submaps)
	if n <= loopMinSubmapGap {
		return
	}

	// Global cooldown between loop-closure attempts.
	if n-m.lastLCAttempt < loopMinSubmapGap {
		return 0, 0, 0, false
	}
	m.lastLCAttempt = n

	cur := m.activeSubmaps[robotID]
	if cur == nil {
		return
	}
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

	// Search ALL keyframe submaps — cross-robot loop closures are allowed.
	// For same-robot anchors we still enforce the minimum submap-gap guard to
	// prevent trivially short self-loops.
	for ai := 0; ai < n-1; ai++ {
		anchor := m.submaps[ai]

		if !anchor.IsKeyframe {
			continue
		}

		// Same-robot anchors must be at least loopMinSubmapGap steps back.
		if anchor.RobotID == robotID && (n-1-ai) <= loopMinSubmapGap {
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
				cur.ID, bestScore, secondScore, math.Hypot(bestDX-secondDX, bestDY-secondDY))
			return 0, 0, 0, false
		}
	}

	shapeRatio := scanGeometricDiversity(walls)
	if shapeRatio < 0.05 {
		fmt.Printf("[SLAM LC] submap %d: rejected (geometric diversity=%.3f < %.3f, single-wall)\n",
			cur.ID, shapeRatio, 0.05)
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
		fmt.Printf("[SLAM LC] submap %d: no closure (quality=%.3f < %.2f)\n", cur.ID, qualityScore, loopMinAvgScore)
		return 0, 0, 0, false
	}

	geomWalls := downsampleWalls(walls, loopGeomMaxWalls)
	stats := ndtMatchGeometry(geomWalls, bestNDT, cur.Origin.X, cur.Origin.Y, bestDX, bestDY, bestDT)
	okGeom, geomReason := passesGeomValidator(stats, loopGeomMinSupportRatio, loopGeomMinPCARatio, loopGeomMinSpanMinor)
	if !okGeom {
		fmt.Printf("[SLAM LC] submap %d: rejected geom (%s support=%.2f pcaRatio=%.3f span=%.2fm×%.2fm)\n",
			cur.ID, geomReason, stats.Ratio, stats.PCARatio, stats.SpanX, stats.SpanY)
		return 0, 0, 0, false
	}

	fmt.Printf("[SLAM LC] CLOSED: submap %d → anchor %d  dx=%.3f dy=%.3f dθ=%.1f°  score=%.3f  shapeRatio=%.3f inlierPCA=%.3f span=%.2fm×%.2fm\n",
		cur.ID, bestAnchor, bestDX, bestDY, bestDT*180/math.Pi, qualityScore, shapeRatio, stats.PCARatio, stats.SpanX, stats.SpanY)

	optDX, optDY, optDT := m.applyLoopClosure(bestAnchor, n-1, bestDX, bestDY, bestDT)
	return optDX, optDY, optDT, true
}
