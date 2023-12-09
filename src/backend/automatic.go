package backend

import (
	"golang-server/log"
	"math"
)

func (s *fullSlamState) findClosestRobot(x, y int) int {
	//find the robot closest to the given point
	minDistance := math.MaxFloat64
	closestRobot := -1
	for id, index := range s.id2index {
		distance := math.Sqrt(math.Pow(float64(s.multiRobot[index].X-x), 2) + math.Pow(float64(s.multiRobot[index].Y-y), 2))
		if distance < minDistance {
			minDistance = distance
			closestRobot = id
		}
	}
	if closestRobot == -1 {
		log.GGeneralLogger.Println("Tried to find closest robot, but no robots were found.")
	}
	return closestRobot
}
