package slam

import "math"

// Pose represents a 2D pose (x, y, theta)
type Pose struct {
	X, Y, Theta float64
}

// PolarPoint is a single lidar/camera scan measurement
type PolarPoint struct {
	Angle float64
	Dist  float64
}

// RayPoint stores the exact pose and ray data to process later
type RayPoint struct {
	RobotX, RobotY, RobotTheta float64
	Angle, Dist                float64
}

// NormalizeAngle normalizes an angle to [-pi, pi]
func NormalizeAngle(angle float64) float64 {
	for angle > math.Pi {
		angle -= 2 * math.Pi
	}
	for angle < -math.Pi {
		angle += 2 * math.Pi
	}
	return angle
}
