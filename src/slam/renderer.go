package slam

import (
	"image"
	"image/color"
	"math"
)

const (
	pixelsPerCell = 4 // render each grid cell as a 4×4 pixel block

	// Local view: SubHalf cells radius → matches the submap extent exactly
	localImgSize = SubSize * pixelsPerCell

	tickSpacing = 20 // cells between tick marks = 1.0 m
	tickLen     = 1  // half-length of tick mark in pixels
)

var (
	clrGray    = color.RGBA{128, 128, 128, 255} // unknown
	clrWhite   = color.RGBA{255, 255, 255, 255} // free space
	clrBlack   = color.RGBA{0, 0, 0, 255}       // wall
	clrRed     = color.RGBA{255, 0, 0, 255}     // robot dot
	clrOutside = color.RGBA{64, 64, 64, 255}    // outside submap bounds
	clrAxis    = color.RGBA{0, 180, 255, 255}   // axis lines (cyan-blue)
	clrTick    = color.RGBA{0, 230, 255, 255}   // tick marks (brighter cyan)
	clrArrow   = color.RGBA{255, 220, 0, 255}   // direction shaft (yellow)

	// One distinct tint per submap for the global view (cycles if > len)
	submapTints = []color.RGBA{
		{200, 230, 255, 255}, // light blue
		{255, 230, 200, 255}, // light orange
		{200, 255, 200, 255}, // light green
		{255, 200, 255, 255}, // light pink
		{255, 255, 200, 255}, // light yellow
		{200, 255, 255, 255}, // light cyan
	}
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
// The robot dot is always at the image center.
func (m *OccupancyMap) renderLocal(s *Submap, rxF, ryF float64) *image.RGBA {
	rx := int(math.Round(rxF))
	ry := int(math.Round(ryF))

	img := image.NewRGBA(image.Rect(0, 0, localImgSize, localImgSize))

	for cy := 0; cy < SubSize; cy++ {
		for cx := 0; cx < SubSize; cx++ {
			// cx/cy are pixel-grid offsets from the robot cell
			wx := rx - SubHalf + cx
			wy := ry - SubHalf + cy
			var clr color.RGBA
			if wx < 0 || wx >= SubSize || wy < 0 || wy >= SubSize {
				clr = clrOutside
			} else {
				clr = logOddsToColor(s.grid[wy][wx], clrWhite, clrBlack)
			}
			baseX, baseY := cx*pixelsPerCell, cy*pixelsPerCell
			for dy := 0; dy < pixelsPerCell; dy++ {
				for dx := 0; dx < pixelsPerCell; dx++ {
					img.Set(baseX+dx, baseY+dy, clr)
				}
			}
		}
	}

	const ctr = SubHalf * pixelsPerCell

	// Axis lines
	for p := 0; p < localImgSize; p++ {
		img.Set(p, ctr, clrAxis)
		img.Set(ctr, p, clrAxis)
	}

	// 1-metre tick marks
	const tScale = tickSpacing * pixelsPerCell
	const tLen = tickLen * pixelsPerCell
	for t := tScale; t < SubHalf*pixelsPerCell; t += tScale {
		for d := -tLen; d <= tLen; d++ {
			img.Set(ctr+t, ctr+d, clrTick)
			img.Set(ctr-t, ctr+d, clrTick)
			img.Set(ctr+d, ctr+t, clrTick)
			img.Set(ctr+d, ctr-t, clrTick)
		}
	}

	// Direction shaft
	theta := m.CurrentPose.Theta
	imgDx := math.Cos(theta)
	imgDy := -math.Sin(theta) // Y flipped in image space

	const shaftPx = 12 * pixelsPerCell
	const dotR = 2 * pixelsPerCell
	for i := dotR + pixelsPerCell; i <= shaftPx; i++ {
		px := ctr + int(math.Round(float64(i)*imgDx))
		py := ctr + int(math.Round(float64(i)*imgDy))
		if px >= 0 && px < localImgSize && py >= 0 && py < localImgSize {
			img.Set(px, py, clrArrow)
		}
	}

	// Robot dot (drawn last, on top)
	for dy := -dotR; dy <= dotR; dy++ {
		for dx := -dotR; dx <= dotR; dx++ {
			img.Set(ctr+dx, ctr+dy, clrRed)
		}
	}
	return img
}

// renderGlobal composites all submaps onto a world-sized image.
// Each submap is tinted a distinct colour so boundaries are visible.
// World grid is GridWidth×GridHeight cells at GridRes m/cell.
const globalImgSize = GridWidth // 1 pixel per world cell (no upscale — it's large already)

func (m *OccupancyMap) renderGlobal() *image.RGBA {
	img := image.NewRGBA(image.Rect(0, 0, globalImgSize, globalImgSize))

	// Fill with gray (unknown)
	for y := 0; y < globalImgSize; y++ {
		for x := 0; x < globalImgSize; x++ {
			img.Set(x, y, clrGray)
		}
	}

	for i, s := range m.submaps {
		tint := submapTints[i%len(submapTints)]
		ox, oy := s.Origin.X, s.Origin.Y

		for cy := 0; cy < SubSize; cy++ {
			for cx := 0; cx < SubSize; cx++ {
				v := s.grid[cy][cx]
				if v == 0 {
					continue // never observed — leave gray from base
				}

				// Convert submap cell → world cell
				// submap cx=SubHalf → world X = ox/GridRes + GridOffX
				wx := int(math.Round((ox/GridRes)+float64(GridOffX))) + (cx - SubHalf)
				wy := GridOffY - int(math.Round(oy/GridRes)) + (cy - SubHalf)

				if wx < 0 || wx >= GridWidth || wy < 0 || wy >= GridHeight {
					continue
				}

				clr := logOddsToColor(v, tint, clrBlack)
				img.Set(wx, wy, clr)
			}
		}
	}

	// Draw robot position
	rpx := int(math.Round(m.CurrentPose.X/GridRes)) + GridOffX
	rpy := GridOffY - int(math.Round(m.CurrentPose.Y/GridRes))
	const dotR = 3
	for dy := -dotR; dy <= dotR; dy++ {
		for dx := -dotR; dx <= dotR; dx++ {
			x, y := rpx+dx, rpy+dy
			if x >= 0 && x < globalImgSize && y >= 0 && y < globalImgSize {
				img.Set(x, y, clrRed)
			}
		}
	}

	return img
}
