package backend

import (
	"golang-server/config"
	util "golang-server/utilities"
)

// GridMap centralizes conversions between pose-space and discrete map-space.
type GridMap struct{}

func NewGridMap() GridMap {
	return GridMap{}
}

func (g GridMap) PoseToCoordinates(p Pose) (int, int) {
	return util.MetresToCm(p.X), util.MetresToCm(p.Y)
}

func (g GridMap) SensorPointToCoordinates(p Pose, xSensorMM, ySensorMM int) (int, int) {
	globalX, globalY := p.LocalToGlobal(util.MmToMetres(xSensorMM), util.MmToMetres(ySensorMM))
	return util.MetresToCm(globalX), util.MetresToCm(globalY)
}

func (g GridMap) CoordinatesToPose(x, y int) Pose {
	return Pose{X: util.CmToMetres(x), Y: util.CmToMetres(y), Theta: 0}
}

func (g GridMap) MapIndex(x, y int) (int, int) {
	return config.MapCenterX + x, config.MapCenterY - y
}

func (g GridMap) ClampIndex(xIndex, yIndex int) (int, int) {
	return min(max(xIndex, 0), config.MapSize-1), min(max(yIndex, 0), config.MapSize-1)
}
