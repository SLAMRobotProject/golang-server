package slam

import (
	"image"
	"image/color"
	"math"
)

const (
	pixelsPerCell = 4 // render each grid cell as a 4×4 pixel block

	// The Local UI camera window (e.g. ±3.0m around the robot)
	ViewHalf     = int(LocalViewHalfWidthM / GridRes)
	ViewSize     = 2 * ViewHalf
	localImgSize = ViewSize * pixelsPerCell

	tickSpacingMeters = 1.0 // metres between tick marks
	tickLen           = 1   // half-length of tick mark in pixels
)

var (
	clrGray     = color.RGBA{128, 128, 128, 255} // unknown
	clrWhite    = color.RGBA{255, 255, 255, 255} // free space
	clrBlack    = color.RGBA{0, 0, 0, 255}       // wall
	clrRed      = color.RGBA{255, 0, 0, 255}     // robot dot
	clrOutside  = color.RGBA{64, 64, 64, 255}    // outside submap bounds
	clrAxis     = color.RGBA{0, 180, 255, 255}   // axis lines (cyan-blue)
	clrTick     = color.RGBA{0, 230, 255, 255}   // tick marks (brighter cyan)
	clrArrow    = color.RGBA{255, 220, 0, 255}   // direction shaft (yellow)
	clrTrail    = color.RGBA{255, 170, 0, 255}   // tail trail (orange)
	clrSeqMatch = color.RGBA{0, 175, 200, 255}   // seq overlap (turquoise-blue)
	clrLCMatch  = color.RGBA{0, 245, 210, 255}   // lc overlap (bright turquoise)

	globalViewportPaddingCells = 20
	globalViewportMinSpanCells = 120
)

func logOddsToColor(v float32, free, wall color.RGBA) color.RGBA {
	if v > 0.2 {
		return wall
	} else if v < -0.1 {
		return free
	}
	return clrGray
}

// renderLocal draws the active submap centered on the robot position (rxF, ryF in submap cell space).
// worldTheta is the robot's heading in the world frame for the direction arrow.
// The display is always world-aligned (North-up): counter-rotating by -Origin.Theta so the
// apparent orientation never jumps when a new submap starts with a corrected heading.
func (m *OccupancyMap) renderLocal(s *Submap, rxF, ryF, worldTheta float64) *image.RGBA {
	img := image.NewRGBA(image.Rect(0, 0, localImgSize, localImgSize))

	cosOt := math.Cos(-s.Origin.Theta)
	sinOt := math.Sin(-s.Origin.Theta)

	for cy := 0; cy < ViewSize; cy++ {
		for cx := 0; cx < ViewSize; cx++ {
			dxW := float64(cx - ViewHalf)
			dyW := -float64(cy - ViewHalf)

			ldx := dxW*cosOt - dyW*sinOt
			ldy := dxW*sinOt + dyW*cosOt

			cellX := int(math.Round(rxF + ldx))
			cellY := int(math.Round(ryF - ldy))

			key := CellKey{X: cellX, Y: cellY}
			v, exists := s.grid[key]

			var clr color.RGBA
			if !exists {
				clr = clrGray
			} else {
				clr = logOddsToColor(v, clrWhite, clrBlack)
			}

			baseX, baseY := cx*pixelsPerCell, cy*pixelsPerCell
			for dy := 0; dy < pixelsPerCell; dy++ {
				for dx := 0; dx < pixelsPerCell; dx++ {
					img.Set(baseX+dx, baseY+dy, clr)
				}
			}
		}
	}

	const ctr = ViewHalf * pixelsPerCell

	// Axis lines
	for p := 0; p < localImgSize; p++ {
		img.Set(p, ctr, clrAxis)
		img.Set(ctr, p, clrAxis)
	}

	// 1-metre tick marks
	tScale := int(math.Round(tickSpacingMeters/GridRes)) * pixelsPerCell
	const tLen = tickLen * pixelsPerCell
	for t := tScale; t < ViewHalf*pixelsPerCell; t += tScale {
		for d := -tLen; d <= tLen; d++ {
			img.Set(ctr+t, ctr+d, clrTick)
			img.Set(ctr-t, ctr+d, clrTick)
			img.Set(ctr+d, ctr+t, clrTick)
			img.Set(ctr+d, ctr-t, clrTick)
		}
	}

	imgDx := math.Cos(worldTheta)
	imgDy := -math.Sin(worldTheta)

	const shaftPx = 12 * pixelsPerCell
	const dotR = 2 * pixelsPerCell
	for i := dotR + pixelsPerCell; i <= shaftPx; i++ {
		px := ctr + int(math.Round(float64(i)*imgDx))
		py := ctr + int(math.Round(float64(i)*imgDy))
		if px >= 0 && px < localImgSize && py >= 0 && py < localImgSize {
			img.Set(px, py, clrArrow)
		}
	}

	// Robot dot
	for dy := -dotR; dy <= dotR; dy++ {
		for dx := -dotR; dx <= dotR; dx++ {
			img.Set(ctr+dx, ctr+dy, clrRed)
		}
	}
	return img
}

// renderGlobal composites all submaps from all robots into one world occupancy grid.
const globalImgSize = GridWidth

// buildWorldComposite stamps every finalized submap and every active submap
// (one per robot) onto a shared world grid and returns the occupied bounds.
func buildWorldComposite(m *OccupancyMap) (
	grid [GridHeight][GridWidth]float32,
	seen [GridHeight][GridWidth]bool,
	minX, minY, maxX, maxY int,
	ok bool,
) {
	minX, minY = GridWidth, GridHeight
	maxX, maxY = -1, -1

	stamp := func(s *Submap) {
		if s == nil {
			return
		}
		for key, v := range s.grid {
			if v == 0 {
				continue
			}
			lx := float64(key.X) * GridRes
			ly := -float64(key.Y) * GridRes

			wx, wy := s.Origin.LocalToGlobal(lx, ly)

			gridX := int(math.Round(wx/GridRes)) + GridOffX
			gridY := GridOffY - int(math.Round(wy/GridRes))

			if gridX < 0 || gridX >= GridWidth || gridY < 0 || gridY >= GridHeight {
				continue
			}

			acc := grid[gridY][gridX] + v
			if acc > ismMaxOdds {
				acc = ismMaxOdds
			} else if acc < ismMinOdds {
				acc = ismMinOdds
			}
			grid[gridY][gridX] = acc
			seen[gridY][gridX] = true

			if gridX < minX {
				minX = gridX
			}
			if gridY < minY {
				minY = gridY
			}
			if gridX > maxX {
				maxX = gridX
			}
			if gridY > maxY {
				maxY = gridY
			}
		}
	}

	for _, s := range m.submaps {
		stamp(s)
	}
	for _, s := range m.activeSubmaps {
		stamp(s)
	}

	if maxX >= minX && maxY >= minY {
		ok = true
	}
	return
}

func (m *OccupancyMap) renderGlobal(showMatches bool) *image.RGBA {
	img := image.NewRGBA(image.Rect(0, 0, globalImgSize, globalImgSize))

	for y := 0; y < globalImgSize; y++ {
		for x := 0; x < globalImgSize; x++ {
			img.Set(x, y, clrGray)
		}
	}

	grid, seen, minX, minY, maxX, maxY, ok := buildWorldComposite(m)
	if !ok {
		return img
	}

	minX -= globalViewportPaddingCells
	minY -= globalViewportPaddingCells
	maxX += globalViewportPaddingCells
	maxY += globalViewportPaddingCells
	if minX < 0 {
		minX = 0
	}
	if minY < 0 {
		minY = 0
	}
	if maxX >= GridWidth {
		maxX = GridWidth - 1
	}
	if maxY >= GridHeight {
		maxY = GridHeight - 1
	}

	spanX := maxX - minX + 1
	spanY := maxY - minY + 1
	if spanX < globalViewportMinSpanCells {
		pad := (globalViewportMinSpanCells - spanX) / 2
		minX -= pad
		maxX += pad
	}
	if spanY < globalViewportMinSpanCells {
		pad := (globalViewportMinSpanCells - spanY) / 2
		minY -= pad
		maxY += pad
	}
	if minX < 0 {
		minX = 0
	}
	if minY < 0 {
		minY = 0
	}
	if maxX >= GridWidth {
		maxX = GridWidth - 1
	}
	if maxY >= GridHeight {
		maxY = GridHeight - 1
	}

	spanX = maxX - minX + 1
	spanY = maxY - minY + 1
	if spanX <= 0 || spanY <= 0 {
		return img
	}

	scaleX := float64(globalImgSize-1) / float64(spanX)
	scaleY := float64(globalImgSize-1) / float64(spanY)
	scale := math.Min(scaleX, scaleY)
	if scale < 1 {
		scale = 1
	}
	if scale > 10 {
		scale = 10
	}

	usedW := int(math.Round(float64(spanX) * scale))
	usedH := int(math.Round(float64(spanY) * scale))
	offsetX := (globalImgSize - usedW) / 2
	offsetY := (globalImgSize - usedH) / 2
	worldToPixel := func(wx, wy float64) (int, int) {
		gx := wx/GridRes + float64(GridOffX)
		gy := float64(GridOffY) - wy/GridRes
		px := offsetX + int(math.Round((gx-float64(minX))*scale))
		py := offsetY + int(math.Round((gy-float64(minY))*scale))
		return px, py
	}

	for y := 0; y < spanY; y++ {
		wy := minY + y
		for x := 0; x < spanX; x++ {
			wx := minX + x
			var clr color.RGBA
			if !seen[wy][wx] {
				clr = clrGray
			} else {
				v := grid[wy][wx]
				clr = logOddsToColor(v, clrWhite, clrBlack)
			}

			px0 := offsetX + int(math.Round(float64(x)*scale))
			py0 := offsetY + int(math.Round(float64(y)*scale))
			px1 := offsetX + int(math.Round(float64(x+1)*scale))
			py1 := offsetY + int(math.Round(float64(y+1)*scale))
			if px1 <= px0 {
				px1 = px0 + 1
			}
			if py1 <= py0 {
				py1 = py0 + 1
			}
			for py := py0; py < py1 && py < globalImgSize; py++ {
				if py < 0 {
					continue
				}
				for px := px0; px < px1 && px < globalImgSize; px++ {
					if px < 0 {
						continue
					}
					img.Set(px, py, clr)
				}
			}
		}
	}

	// Draw all robots' trails and match overlays before robot icons.
	for _, rs := range m.robots {
		if len(rs.trail) >= 2 {
			for i := 1; i < len(rs.trail); i++ {
				x0, y0 := worldToPixel(rs.trail[i-1].x, rs.trail[i-1].y)
				x1, y1 := worldToPixel(rs.trail[i].x, rs.trail[i].y)
				drawDottedLine(img, x0, y0, x1, y1, clrTrail, 4, 3)
			}
		}
		if showMatches {
			drawMatchOverlays(img, rs.seqMatchOverlays, worldToPixel, clrSeqMatch)
			drawMatchOverlays(img, rs.lcMatchOverlays, worldToPixel, clrLCMatch)
		}
	}

	// Draw each robot's car icon.
	for robotID, rs := range m.robots {
		if m.activeSubmaps[robotID] == nil {
			continue
		}
		rpx, rpy := worldToPixel(rs.CurrentPose.X, rs.CurrentPose.Y)
		drawOrientedCar(img, rpx, rpy, rs.CurrentPose.Theta, scale)
	}

	return img
}

func drawMatchOverlays(img *image.RGBA, overlays []matchOverlay, worldToPixel func(wx, wy float64) (int, int), clr color.RGBA) {
	for _, ov := range overlays {
		for _, p := range ov.points {
			px, py := worldToPixel(p.wx, p.wy)
			drawMarkerDot(img, px, py, 1, clr)
		}
	}
}

func drawMarkerDot(img *image.RGBA, cx, cy, radius int, clr color.RGBA) {
	r2 := radius * radius
	for y := -radius; y <= radius; y++ {
		for x := -radius; x <= radius; x++ {
			if x*x+y*y > r2 {
				continue
			}
			px := cx + x
			py := cy + y
			if px >= 0 && px < globalImgSize && py >= 0 && py < globalImgSize {
				img.Set(px, py, clr)
			}
		}
	}
}

func drawOrientedCar(img *image.RGBA, cx, cy int, theta, scale float64) {
	imgDx := math.Cos(theta)
	imgDy := -math.Sin(theta)
	sideX := -imgDy
	sideY := imgDx

	lenPx := int(math.Round(7 * scale / 2))
	if lenPx < 6 {
		lenPx = 6
	}
	if lenPx > 14 {
		lenPx = 14
	}
	widPx := int(math.Round(float64(lenPx) * 0.55))
	if widPx < 3 {
		widPx = 3
	}

	rearX := cx - int(math.Round(0.6*float64(lenPx)*imgDx))
	rearY := cy - int(math.Round(0.6*float64(lenPx)*imgDy))
	frontX := cx + int(math.Round(0.8*float64(lenPx)*imgDx))
	frontY := cy + int(math.Round(0.8*float64(lenPx)*imgDy))

	leftRearX := rearX + int(math.Round(0.7*float64(widPx)*sideX))
	leftRearY := rearY + int(math.Round(0.7*float64(widPx)*sideY))
	rightRearX := rearX - int(math.Round(0.7*float64(widPx)*sideX))
	rightRearY := rearY - int(math.Round(0.7*float64(widPx)*sideY))
	leftFrontX := frontX + int(math.Round(float64(widPx)*sideX))
	leftFrontY := frontY + int(math.Round(float64(widPx)*sideY))
	rightFrontX := frontX - int(math.Round(float64(widPx)*sideX))
	rightFrontY := frontY - int(math.Round(float64(widPx)*sideY))

	drawLine(img, leftRearX, leftRearY, leftFrontX, leftFrontY, clrRed, 2)
	drawLine(img, rightRearX, rightRearY, rightFrontX, rightFrontY, clrRed, 2)
	drawLine(img, leftRearX, leftRearY, rightRearX, rightRearY, clrRed, 2)
	drawLine(img, leftFrontX, leftFrontY, rightFrontX, rightFrontY, clrRed, 2)
	drawLine(img, cx, cy, frontX, frontY, clrRed, 2)
}

func drawLine(img *image.RGBA, x0, y0, x1, y1 int, clr color.RGBA, thickness int) {
	dx := x1 - x0
	if dx < 0 {
		dx = -dx
	}
	dy := y1 - y0
	if dy < 0 {
		dy = -dy
	}
	steps := dx
	if dy > steps {
		steps = dy
	}
	if steps == 0 {
		if x0 >= 0 && x0 < globalImgSize && y0 >= 0 && y0 < globalImgSize {
			img.Set(x0, y0, clr)
		}
		return
	}

	xInc := float64(x1-x0) / float64(steps)
	yInc := float64(y1-y0) / float64(steps)
	x := float64(x0)
	y := float64(y0)
	r := thickness / 2
	for i := 0; i <= steps; i++ {
		px := int(math.Round(x))
		py := int(math.Round(y))
		for oy := -r; oy <= r; oy++ {
			for ox := -r; ox <= r; ox++ {
				nx, ny := px+ox, py+oy
				if nx >= 0 && nx < globalImgSize && ny >= 0 && ny < globalImgSize {
					img.Set(nx, ny, clr)
				}
			}
		}
		x += xInc
		y += yInc
	}
}

func drawDottedLine(img *image.RGBA, x0, y0, x1, y1 int, clr color.RGBA, onPx, offPx int) {
	dx := x1 - x0
	if dx < 0 {
		dx = -dx
	}
	dy := y1 - y0
	if dy < 0 {
		dy = -dy
	}
	steps := dx
	if dy > steps {
		steps = dy
	}
	if steps == 0 {
		if x0 >= 0 && x0 < globalImgSize && y0 >= 0 && y0 < globalImgSize {
			img.Set(x0, y0, clr)
		}
		return
	}
	xInc := float64(x1-x0) / float64(steps)
	yInc := float64(y1-y0) / float64(steps)
	cycle := onPx + offPx
	if cycle <= 0 {
		cycle = 1
	}
	x := float64(x0)
	y := float64(y0)
	for i := 0; i <= steps; i++ {
		if i%cycle < onPx {
			px := int(math.Round(x))
			py := int(math.Round(y))
			if px >= 0 && px < globalImgSize && py >= 0 && py < globalImgSize {
				img.Set(px, py, clr)
			}
		}
		x += xInc
		y += yInc
	}
}
