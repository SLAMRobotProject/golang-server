package main

import (
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
	calculated_points := bresenham_algorithm(x0, y0, x1, y1)
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

func Test__add_line(t *testing.T) {
	s := init_fullSlamState()
	id := 2
	s.id2index[id] = len(s.multi_robot)
	s.multi_robot = append(s.multi_robot, *init_robotState(0, 0, 90))

	x1, y1 := 20, 20
	x1_idx, y1_idx := get_mapIndex(x1, y1)
	s.add_line(id, x1, y1)
	if s.Map[x1_idx][y1_idx] != MAP_OBSTACLE {
		t.Errorf("Function add_line did not add obstacle to map correctly.")
	}

	x1, y1 = -20, -20
	x1_mod_idx, y1_mod_idx := get_mapIndex(x1+1, y1+1) //modified to test the point before the obstacle
	s.add_line(id, x1, y1)
	if s.Map[x1_mod_idx][y1_mod_idx] != MAP_OPEN {
		t.Errorf("Function add_line did not add line to map correctly.")
	}

	x1, y1 = 40, 40
	x1_idx, y1_idx = get_mapIndex(x1, y1)
	if s.Map[x1_idx][y1_idx] == MAP_UNKNOWN {
		s.add_line(id, x1, y1)
		x1_mod_idx, y1_mod_idx = get_mapIndex(21, 21) //modified to respect a max distance of 30
		if math.Sqrt(float64(x1*x1+y1*y1)) > IR_SENSOR_MAX_DISTANCE && s.Map[x1_idx][y1_idx] != MAP_UNKNOWN {
			t.Errorf("Function add_line did not respect the max distance.")
		} else if s.Map[x1_mod_idx][y1_mod_idx] != MAP_OPEN {
			t.Errorf("Function add_line did not add line to map correctly.")
		}
	} else {
		t.Errorf("Could not test max distance, because the point is already known.")
	}
}

func Test__irSensorData_add(t *testing.T) {
	s := init_fullSlamState()
	id := 2
	s.id2index[id] = len(s.multi_robot)
	s.multi_robot = append(s.multi_robot, *init_robotState(0, 0, 90))

	irX, irY := 500, 500 //written in milimeters (because of the robot code), while the map is in centimeters
	s.irSensorData_add(id, irX, irY)
	if s.Map[MAP_CENTER_X+21][MAP_CENTER_Y-21] != MAP_OPEN {
		t.Errorf("Function irSensorData_add did not add line to map correctly.#1")
	}

	irX, irY = -200, -200
	s.irSensorData_add(id, irX, irY)
	if s.Map[MAP_CENTER_X-19][MAP_CENTER_Y+19] != MAP_OPEN {
		t.Errorf("Function irSensorData_add did not add line to map correctly.#2")
	}
	if s.Map[MAP_CENTER_X-20][MAP_CENTER_Y+20] != MAP_OBSTACLE {
		print(s.Map[MAP_CENTER_X-20][MAP_CENTER_Y+20])
		t.Errorf("Function irSensorData_add did not add obstruction.")
	}

	irX, irY = 1000, 1000
	s.multi_robot[s.id2index[id]].x = -150
	s.irSensorData_add(id, irX, irY)
	if s.Map[MAP_CENTER_X+21][MAP_CENTER_Y+150-21] == MAP_OPEN {
		t.Errorf("Function irSensorData_add did not respect the max distance. #3")
	}
}
