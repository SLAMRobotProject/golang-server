package slam

import (
	"image"
	"image/color"
	"math"
)

// Coordinates to Grid Index for the viewing window
func (s *GoSLAM) posToGlobalIdx(x, y float64) (int, int, bool) {
	ix := int(x/s.GridRes) + s.GlobalSize/2
	iy := s.GlobalSize/2 - int(y/s.GridRes)

	if ix >= 0 && ix < s.GlobalSize && iy >= 0 && iy < s.GlobalSize {
		return ix, iy, true
	}
	return 0, 0, false
}

// computeGlobalGrid projects all submaps into a unified 2D float64 array for rendering.
func (s *GoSLAM) computeGlobalGrid() []float64 {
	compositeGrid := make([]float64, s.GlobalSize*s.GlobalSize)

	// Create a slice that contains completed submaps + the active one
	allMaps := append(s.Submaps, s.ActiveSubmap)

	for _, sm := range allMaps {
		if sm == nil {
			continue
		}

		// Iterate through every cell in the submap and add it to the global composite
		for ly := 0; ly < sm.GridSize; ly++ {
			for lx := 0; lx < sm.GridSize; lx++ {
				val := sm.Grid[ly*sm.GridSize+lx]
				if math.Abs(val) < 0.1 {
					continue // Ignore nearly empty log-odds blocks
				}

				// Convert local indices back to global coordinates based on the submap's OriginPose
				dx := float64(lx-sm.GridSize/2) * s.GridRes
				dy := float64(sm.GridSize/2-ly) * s.GridRes // Y is inverted in indices

				gx := sm.OriginPose.X + dx
				gy := sm.OriginPose.Y + dy

				// Project onto composite map
				gix, giy, valid := s.posToGlobalIdx(gx, gy)
				if valid {
					gIdx := giy*s.GlobalSize + gix
					compositeGrid[gIdx] += val
				}
			}
		}
	}
	return compositeGrid
}

// renderGridImage turns the log-odds back into a grayscale image byte array
func (s *GoSLAM) renderGridImage() *image.RGBA {
	rect := image.Rect(0, 0, s.GlobalSize, s.GlobalSize)
	img := image.NewRGBA(rect)

	// Combine all submaps into one flattened grid
	logOddsGrid := s.computeGlobalGrid()

	gridColor := color.RGBA{R: 200, G: 200, B: 200, A: 255}

	for y := 0; y < s.GlobalSize; y++ {
		for x := 0; x < s.GlobalSize; x++ {
			logOdds := logOddsGrid[y*s.GlobalSize+x]

			p := 1.0 - (1.0 / (1.0 + math.Exp(logOdds)))
			val := uint8((1.0 - p) * 255.0)

			img.Set(x, y, color.RGBA{R: val, G: val, B: val, A: 255})

			if (x-s.GlobalSize/2)%100 == 0 || (y-s.GlobalSize/2)%100 == 0 {
				img.Set(x, y, gridColor)
			}
		}
	}

	// Plot the robot trajectory on the map in a transparent RED color
	red := color.RGBA{R: 255, G: 0, B: 0, A: 255}
	for i, pose := range s.Poses {
		ix, iy, valid := s.posToGlobalIdx(pose.X, pose.Y)
		if valid {
			if i > 0 {
				prevPose := s.Poses[i-1]
				pix, piy, pValid := s.posToGlobalIdx(prevPose.X, prevPose.Y)
				if pValid {
					distPixels := math.Hypot(float64(ix-pix), float64(iy-piy))
					steps := int(distPixels) + 1
					for step := 0; step <= steps; step++ {
						t := float64(step) / float64(steps)
						hx := int(float64(pix) + t*float64(ix-pix))
						hy := int(float64(piy) + t*float64(iy-piy))

						for dy := -1; dy <= 1; dy++ {
							for dx := -1; dx <= 1; dx++ {
								if hx+dx >= 0 && hx+dx < s.GlobalSize && hy+dy >= 0 && hy+dy < s.GlobalSize {
									img.Set(hx+dx, hy+dy, red)
								}
							}
						}
					}
				}
			} else {
				for dy := -1; dy <= 1; dy++ {
					for dx := -1; dx <= 1; dx++ {
						if ix+dx >= 0 && ix+dx < s.GlobalSize && iy+dy >= 0 && iy+dy < s.GlobalSize {
							img.Set(ix+dx, iy+dy, red)
						}
					}
				}
			}

			if i == len(s.Poses)-1 {
				blue := color.RGBA{R: 0, G: 0, B: 255, A: 255}
				for dy := -3; dy <= 3; dy++ {
					for dx := -3; dx <= 3; dx++ {
						if dx*dx+dy*dy <= 9 && ix+dx >= 0 && ix+dx < s.GlobalSize && iy+dy >= 0 && iy+dy < s.GlobalSize {
							img.Set(ix+dx, iy+dy, blue)
						}
					}
				}

				green := color.RGBA{R: 0, G: 255, B: 0, A: 255}
				lineLength := 15.0
				for step := 0.0; step <= lineLength; step += 0.5 {
					hx := int(float64(ix) + math.Cos(pose.Theta)*step)
					hy := int(float64(iy) - math.Sin(pose.Theta)*step)
					if hx >= 0 && hx < s.GlobalSize && hy >= 0 && hy < s.GlobalSize {
						img.Set(hx, hy, green)
					}
				}
			}
		}
	}

	return img
}
