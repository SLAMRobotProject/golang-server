package backend

import (
	"golang-server/config"
	util "golang-server/utilities"
	"math"
)

func (s *BackendRuntimeState) setMapValue(x, y int, value uint8) {
	s.areaMap[x][y] = value
	switch value {
	case mapOpen:
		s.newOpen = append(s.newOpen, [2]int{x, y})
	case mapObstacle:
		s.newObstacle = append(s.newObstacle, [2]int{x, y})
	}
}

func (s *BackendRuntimeState) addIrSensorData(id, irX, irY int) {
	xMap, yMap := s.transformIrSensorData(id, irX, irY)
	s.addLineToMap(id, xMap, yMap)
}

func (s *BackendRuntimeState) transformIrSensorData(id, xSensorMM, ySensorMM int) (int, int) {
	// IR data is given in mm and it is relative to the body, so it must be scaled, rotated and translated to the map.
	ySensorMM = -ySensorMM // axis is flipped in robot code
	pose, ok := s.getCurrentPose(id)
	if !ok {
		return 0, 0
	}
	return s.grid.SensorPointToCoordinates(pose, xSensorMM, ySensorMM)
}

func (s *BackendRuntimeState) addLineToMap(id, x1, y1 int) {
	// x0, y0, x1, y1 are map coordinates in cm.
	pose, ok := s.getCurrentPose(id)
	if !ok {
		return
	}
	x0, y0 := s.grid.PoseToCoordinates(pose)

	lineLength := math.Sqrt(math.Pow(float64(x0-x1), 2) + math.Pow(float64(y0-y1), 2))

	var obstruction bool
	if lineLength < config.IrSensorMaxDistance {
		obstruction = true
	} else {
		obstruction = false

		// shorten the line to max sensor distance for Bresenham
		scale := config.IrSensorMaxDistance / lineLength
		x1 = x0 + int(scale*float64(x1-x0))
		y1 = y0 + int(scale*float64(y1-y0))
	}

	x0Index, y0Index := s.grid.MapIndex(x0, y0)
	x1Index, y1Index := s.grid.MapIndex(x1, y1)
	x1Index, y1Index = s.grid.ClampIndex(x1Index, y1Index)
	x0Index, y0Index = s.grid.ClampIndex(x0Index, y0Index)

	indexPoints := util.BresenhamAlgorithm(x0Index, y0Index, x1Index, y1Index)
	for i := 0; i < len(indexPoints); i++ {
		x := indexPoints[i][0]
		y := indexPoints[i][1]
		s.setMapValue(x, y, mapOpen)
	}
	if obstruction {
		s.setMapValue(x1Index, y1Index, mapObstacle)
	}
}

func calculateMapIndex(x, y int) (int, int) {
	// Kept as a compatibility wrapper for tests and legacy call sites.
	return NewGridMap().MapIndex(x, y)
}
