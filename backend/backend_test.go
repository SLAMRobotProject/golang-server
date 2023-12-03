package backend

import (
	"golang-server/config"
	"golang-server/utilities"
	"math"
	"testing"
)

func confirm_bresenham_output(known_points, calculated_points [][]int) bool {
	success := false
	for i := 0; i < len(known_points); i++ {
		if known_points[i][0] == calculated_points[i][0] && known_points[i][1] == calculated_points[i][1] {
			success = true
		} else {
			success = false
			break
		}
	}
	if success != true { //bresenham algorithm might have failed, because of wrong order of points
		for i := len(known_points) - 1; i >= 0; i-- {
			if known_points[i][0] == calculated_points[i][0] && known_points[i][1] == calculated_points[i][1] {
				success = true
			} else {
				success = false
				break
			}
		}
	}
	return success
}

func baseTest__bresenham_algorithm(t *testing.T, known_points [][]int, x0, y0, x1, y1 int) bool {
	success := true
	calculated_points := utilities.BresenhamAlgorithm(x0, y0, x1, y1)
	if len(calculated_points) != len(known_points) {
		success = false
	} else {
		success = confirm_bresenham_output(known_points, calculated_points)
	}
	if success != true {
		t.Errorf("Test of bresenham_algorithm failed. Expected: %v. Got: %v", known_points, calculated_points)
	}
	return success
}

func Test__bresenham_algorithm(t *testing.T) {
	//Testing multiple directions

	known_points := [][]int{{0, 0}, {1, 1}, {2, 2}, {3, 3}, {4, 4}}
	baseTest__bresenham_algorithm(t, known_points, 0, 0, 4, 4)

	known_points = [][]int{{0, 0}, {-1, -1}, {-2, -2}, {-3, -3}, {-4, -4}}
	baseTest__bresenham_algorithm(t, known_points, 0, 0, -4, -4)

	known_points = [][]int{{0, 0}, {-1, 1}, {-2, 1}, {-3, 2}, {-4, 2}, {-5, 3}}
	baseTest__bresenham_algorithm(t, known_points, 0, 0, -5, 3)
}

func Test__add_lineToMap(t *testing.T) {
	s := initFullSlamState()
	id := 2
	s.id2index[id] = len(s.multiRobot)
	s.multiRobot = append(s.multiRobot, *initRobotState(0, 0, 90))

	x1, y1 := 20, 20
	x1_idx, y1_idx := calculateMapIndex(x1, y1)
	s.addLineToMap(id, x1, y1)
	if s.areaMap[x1_idx][y1_idx] != MAP_OBSTACLE {
		t.Errorf("Function addLineToMap did not add obstacle to map correctly.")
	}

	x1, y1 = -20, -20
	x1_mod_idx, y1_mod_idx := calculateMapIndex(x1+1, y1+1) //modified to test the point before the obstacle
	s.addLineToMap(id, x1, y1)
	if s.areaMap[x1_mod_idx][y1_mod_idx] != MAP_OPEN {
		t.Errorf("Function addLineToMap did not add line to map correctly.")
	}

	x1, y1 = 40, 40
	x1_idx, y1_idx = calculateMapIndex(x1, y1)
	if s.areaMap[x1_idx][y1_idx] == MAP_UNKNOWN {
		s.addLineToMap(id, x1, y1)
		x1_mod_idx, y1_mod_idx = calculateMapIndex(21, 21) //modified to respect a max distance of 30
		if math.Sqrt(float64(x1*x1+y1*y1)) > config.IR_SENSOR_MAX_DISTANCE && s.areaMap[x1_idx][y1_idx] != MAP_UNKNOWN {
			t.Errorf("Function addLineToMap did not respect the max distance.")
		} else if s.areaMap[x1_mod_idx][y1_mod_idx] != MAP_OPEN {
			t.Errorf("Function addLineToMap did not add line to map correctly.")
		}
	} else {
		t.Errorf("Could not test max distance, because the point is already known.")
	}
}

func Test__irSensorData_add(t *testing.T) {
	s := initFullSlamState()
	id := 2
	s.id2index[id] = len(s.multiRobot)
	s.multiRobot = append(s.multiRobot, *initRobotState(0, 0, 90))

	irX, irY := 500, 500 //written in milimeters (because of the robot code), while the map is in centimeters
	s.addIrSensorData(id, irX, irY)
	if s.areaMap[config.MAP_CENTER_X+21][config.MAP_CENTER_Y-21] != MAP_OPEN {
		t.Errorf("Function addIrSensorData did not add line to map correctly.#1")
	}

	irX, irY = -200, -200
	s.addIrSensorData(id, irX, irY)
	if s.areaMap[config.MAP_CENTER_X-19][config.MAP_CENTER_Y+19] != MAP_OPEN {
		t.Errorf("Function addIrSensorData did not add line to map correctly.#2")
	}
	if s.areaMap[config.MAP_CENTER_X-20][config.MAP_CENTER_Y+20] != MAP_OBSTACLE {
		print(s.areaMap[config.MAP_CENTER_X-20][config.MAP_CENTER_Y+20])
		t.Errorf("Function addIrSensorData did not add obstruction.")
	}

	irX, irY = 1000, 1000
	s.multiRobot[s.id2index[id]].X = -150
	s.addIrSensorData(id, irX, irY)
	if s.areaMap[config.MAP_CENTER_X+21][config.MAP_CENTER_Y+150-21] == MAP_OPEN {
		t.Errorf("Function addIrSensorData did not respect the max distance. #3")
	}
}
