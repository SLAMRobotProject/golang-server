package backend

import (
	"golang-server/config"
	util "golang-server/utilities"
	"math"
)

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
	xPos, yPos := s.grid.PoseToCoordinates(pose)
	theta := util.RadiansToDegrees(pose.Theta)

	x1Rotated, y1Rotated := util.Rotate(float64(x1Map), float64(y1Map), float64(theta-90))
	x2Rotated, y2Rotated := util.Rotate(float64(x2Map), float64(y2Map), float64(theta-90))

	x1Map = int(math.Round(x1Rotated)) + xPos
	y1Map = int(math.Round(y1Rotated)) + yPos
	x2Map = int(math.Round(x2Rotated)) + xPos
	y2Map = int(math.Round(y2Rotated)) + yPos

	x1Index, y1Index := s.grid.MapIndex(x1Map, y1Map)
	x2Index, y2Index := s.grid.MapIndex(x2Map, y2Map)
	x1Index, y1Index = s.grid.ClampIndex(x1Index, y1Index)
	x2Index, y2Index = s.grid.ClampIndex(x2Index, y2Index)

	segmentPoints := util.BresenhamAlgorithm(x1Index, y1Index, x2Index, y2Index)

	rxIndex, ryIndex := s.grid.MapIndex(xPos, yPos)
	rxIndex, ryIndex = s.grid.ClampIndex(rxIndex, ryIndex)

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
