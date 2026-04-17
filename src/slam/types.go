package slam

import (
	"golang-server/utilities"
	"math"
)

// Pose represents a 2D robot pose in meters and radians.
type Pose struct {
	X, Y, Theta float64
}

func normalizeAngle(angle float64) float64 {
	return utilities.NormalizeAngle(angle)
}

func (p *Pose) GlobalToLocal(globalX, globalY float64) (localX, localY float64) {
	dx := globalX - p.X
	dy := globalY - p.Y
	cosT := math.Cos(-p.Theta)
	sinT := math.Sin(-p.Theta)

	localX = dx*cosT - dy*sinT
	localY = dx*sinT + dy*cosT
	return
}

func (p *Pose) LocalToGlobal(localX, localY float64) (globalX, globalY float64) {
	cosT := math.Cos(p.Theta)
	sinT := math.Sin(p.Theta)

	globalX = p.X + localX*cosT - localY*sinT
	globalY = p.Y + localX*sinT + localY*cosT
	return
}
