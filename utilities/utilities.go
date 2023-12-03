package utilities

import "math"

//Generic rotate function used in multiple files.
func Rotate(inputX, inputY, theta float64) (float64, float64) {
	//rotate the point around origo. Theta is given in degrees.
	thetaRad := theta * math.Pi / 180
	outputX := inputX*math.Cos(thetaRad) - inputY*math.Sin(thetaRad)
	outputY := inputX*math.Sin(thetaRad) + inputY*math.Cos(thetaRad)
	return outputX, outputY
}

//Bresenham's line algorithm. Used to find all pixels to form a line between two points.
func BresenhamAlgorithm(x0, y0, x1, y1 int) [][]int {
	dx := math.Abs(float64(x1 - x0))
	sx := 1
	if x0 > x1 {
		sx = -1
	}
	dy := -math.Abs(float64(y1 - y0))
	sy := 1
	if y0 > y1 {
		sy = -1
	}

	err := dx + dy
	points := make([][]int, 0)

	for {
		points = append(points, []int{x0, y0})
		if x0 == x1 && y0 == y1 {
			break
		}
		e2 := 2 * err
		if e2 >= dy {
			if x0 == x1 {
				break
			}
			err = err + dy
			x0 = x0 + sx
		}
		if e2 <= dx {
			if y0 == y1 {
				break
			}
			err = err + dx
			y0 = y0 + sy
		}
	}
	return points
}
