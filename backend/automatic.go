package backend

import (
	"golang-server/log"
	"math"
)

func (s *fullSlamState) find_closest_robot(x, y int) int {
	//find the robot closest to the given point
	min_distance := math.MaxFloat64
	closest_robot := -1
	for id, index := range s.id2index {
		distance := math.Sqrt(math.Pow(float64(s.multi_robot[index].X-x), 2) + math.Pow(float64(s.multi_robot[index].Y-y), 2))
		if distance < min_distance {
			min_distance = distance
			closest_robot = id
		}
	}
	if closest_robot == -1 {
		log.G_generalLogger.Println("Tried to find closest robot, but no robots were found.")
	}
	return closest_robot
}
