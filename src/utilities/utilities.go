package utilities

import (
	"fmt"
	"math"
	"strings"
)

func Rotate(inputX, inputY, theta float64) (float64, float64) {
	//rotate the point around origo. Theta is given in degrees.
	thetaRad := theta * math.Pi / 180
	outputX := inputX*math.Cos(thetaRad) - inputY*math.Sin(thetaRad)
	outputY := inputX*math.Sin(thetaRad) + inputY*math.Cos(thetaRad)
	return outputX, outputY
}

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

// Normalize the angle to be within the range of -π to π
func NormalizeAngle(angle float64) float64 {

	for angle > math.Pi {
		angle -= 2 * math.Pi
	}
	for angle < -math.Pi {
		angle += 2 * math.Pi
	}
	return angle
}

// Unit conversion helpers for clean handling of millimetres, centimetres, metres, degrees, and radians.

// MmToMetres converts millimetres to metres.
func MmToMetres(mm int) float64 {
	return float64(mm) / 1000.0
}

// MetresToMm converts metres back to millimetres.
func MetresToMm(m float64) int {
	return int(math.Round(m * 1000.0))
}

// CmToMetres converts centimetres to metres.
func CmToMetres(cm int) float64 {
	return float64(cm) / 100.0
}

// MetresToCm converts metres back to centimetres.
func MetresToCm(m float64) int {
	return int(math.Round(m * 100.0))
}

// DegreesToRadians converts degrees (int) to radians.
func DegreesToRadians(deg int) float64 {
	return float64(deg) * math.Pi / 180.0
}

// RadiansToDegrees converts radians back to degrees (int).
func RadiansToDegrees(rad float64) int {
	return int(math.Round(rad * 180.0 / math.Pi))
}

// FormatCovarianceMatrix formats a 5x5 covariance matrix as comma-separated float values.
func FormatCovarianceMatrix(matrix [5 * 5]float32) string {
	var sb strings.Builder
	for i, value := range matrix {
		if i > 0 {
			sb.WriteString(",")
		}
		sb.WriteString(fmt.Sprintf("%f", value))
	}
	return sb.String()
}
