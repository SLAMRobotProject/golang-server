package backend

import (
	"golang-server/backend/grid"
	"golang-server/config"
	"golang-server/slam"
	"golang-server/types"
	util "golang-server/utilities"
	"image"
	"math"
)

// using binary flags to represent the map to allow bitwise operations
const (
	mapOpen     uint8 = 1 << iota //1
	mapUnknown                    //2
	mapObstacle                   //4
)

type MapRuntimeState struct {
	areaMap           [config.MapSize][config.MapSize]uint8
	newObstacle       [][2]int
	newOpen           [][2]int
	grid              grid.GridMap
	visualLines       [][2]types.Point
	slamMapper        *slam.OccupancyMap     // single global map shared by all robots
	slamRobotIDs      map[int]struct{}       // set of registered robot IDs
	slamMapImgs       map[int]image.Image    // per-robot local SLAM view
	slamGlobalImgs    map[int]image.Image    // per-robot slot for the shared global image
	slamGlobalDbgImgs map[int]image.Image    // per-robot slot for the shared debug image
}

func initMapRuntimeState() MapRuntimeState {
	m := MapRuntimeState{}
	for i := 0; i < config.MapSize; i++ {
		for j := 0; j < config.MapSize; j++ {
			m.areaMap[i][j] = mapUnknown
		}
	}
	m.grid = grid.NewGridMap()
	m.slamMapper = slam.NewOccupancyMap()
	m.slamRobotIDs = make(map[int]struct{})
	m.slamMapImgs = make(map[int]image.Image)
	m.slamGlobalImgs = make(map[int]image.Image)
	m.slamGlobalDbgImgs = make(map[int]image.Image)
	return m
}

func (s *BackendRuntimeState) setMapValue(x, y int, value uint8) {
	s.runtimeMap.areaMap[x][y] = value
	switch value {
	case mapOpen:
		s.runtimeMap.newOpen = append(s.runtimeMap.newOpen, [2]int{x, y})
	case mapObstacle:
		s.runtimeMap.newObstacle = append(s.runtimeMap.newObstacle, [2]int{x, y})
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
	return s.runtimeMap.grid.SensorPointToCoordinates(pose, xSensorMM, ySensorMM)
}

func (s *BackendRuntimeState) addLineToMap(id, x1, y1 int) {
	// x0, y0, x1, y1 are map coordinates in cm.
	pose, ok := s.getCurrentPose(id)
	if !ok {
		return
	}
	x0, y0 := s.runtimeMap.grid.PoseToCoordinates(pose)

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

	x0Index, y0Index := s.runtimeMap.grid.MapIndex(x0, y0)
	x1Index, y1Index := s.runtimeMap.grid.MapIndex(x1, y1)
	x1Index, y1Index = s.runtimeMap.grid.ClampIndex(x1Index, y1Index)
	x0Index, y0Index = s.runtimeMap.grid.ClampIndex(x0Index, y0Index)

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

// addCameraSegment converts a camera segment (mm in robot body frame)
// into map indices and marks obstacle/free cells.
func (s *BackendRuntimeState) addCameraSegment(id, startMM, widthMM, distanceMM int) {
	adjDist := distanceMM + config.CameraMountOffsetMM

	x1Map, y1Map := int(startMM/10), int(adjDist/10)
	x2Map, y2Map := int((startMM+widthMM)/10), int(adjDist/10)

	pose, ok := s.getCurrentPose(id)
	if !ok {
		return
	}
	xPos, yPos := s.runtimeMap.grid.PoseToCoordinates(pose)
	theta := util.RadiansToDegrees(pose.Theta)

	x1Rotated, y1Rotated := util.Rotate(float64(x1Map), float64(y1Map), float64(theta-90))
	x2Rotated, y2Rotated := util.Rotate(float64(x2Map), float64(y2Map), float64(theta-90))

	x1Map = int(math.Round(x1Rotated)) + xPos
	y1Map = int(math.Round(y1Rotated)) + yPos
	x2Map = int(math.Round(x2Rotated)) + xPos
	y2Map = int(math.Round(y2Rotated)) + yPos

	x1Index, y1Index := s.runtimeMap.grid.MapIndex(x1Map, y1Map)
	x2Index, y2Index := s.runtimeMap.grid.MapIndex(x2Map, y2Map)
	x1Index, y1Index = s.runtimeMap.grid.ClampIndex(x1Index, y1Index)
	x2Index, y2Index = s.runtimeMap.grid.ClampIndex(x2Index, y2Index)

	segmentPoints := util.BresenhamAlgorithm(x1Index, y1Index, x2Index, y2Index)

	rxIndex, ryIndex := s.runtimeMap.grid.MapIndex(xPos, yPos)
	rxIndex, ryIndex = s.runtimeMap.grid.ClampIndex(rxIndex, ryIndex)

	for _, p := range segmentPoints {
		sx := p[0]
		sy := p[1]

		if adjDist < 2900 {
			s.setMapValue(sx, sy, mapObstacle)
		}
		lineToObs := util.BresenhamAlgorithm(rxIndex, ryIndex, sx, sy)
		if len(lineToObs) > 1 {
			for i := 0; i < len(lineToObs)-1; i++ {
				lx := lineToObs[i][0]
				ly := lineToObs[i][1]
				s.setMapValue(lx, ly, mapOpen)
			}
		}
	}
}
