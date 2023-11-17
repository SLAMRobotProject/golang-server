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
	b := NewBackend()
	id := 2
	b.id2index[id] = len(backend.multi_robot)
	b.multi_robot = append(backend.multi_robot, Robot{x: 0, y: 0, theta: 0})

	x1, y1 := 20, 20
	b.add_line(id, x1, y1)
	if b.Map[20+map_center_x][20+map_center_y] != map_obstacle {
		t.Errorf("Function add_line did not add obstacle to map correctly.")
	}

	x1, y1 = -20, -20
	b.add_line(id, x1, y1)
	if b.Map[map_center_x-19][map_center_y-19] != map_open {
		t.Errorf("Function add_line did not add line to map correctly.")
	}

	x1, y1 = 40, 40
	if b.Map[x1+map_center_x][y1+map_center_y] == map_unknown {
		b.add_line(id, x1, y1)
		if math.Sqrt(float64(x1*x1+y1*y1)) > irSensor_maxDistance && b.Map[x1+map_center_x][y1+map_center_y] != map_unknown {
			t.Errorf("Function add_line did not respect the max distance.")
		} else if b.Map[21+map_center_x][21+map_center_y] != map_open {
			t.Errorf("Function add_line did not add line to map correctly.")
		}
	} else {
		t.Errorf("Could not test max distance, because the point is already known.")
	}
}

func Test__add_irSensorData(t *testing.T) {
	b := NewBackend()
	id := 2
	b.id2index[id] = len(backend.multi_robot)
	b.multi_robot = append(backend.multi_robot, Robot{x: 0, y: 0, theta: 0})

	irX, irY := 500, 500 //written in milimeters, while the map is in centimeters
	b.add_irSensorData(id, irX, irY)
	if b.Map[map_center_x+21][map_center_y-21] != map_open {
		t.Errorf("Function add_irSensorData did not add line to map correctly.#1")
	}

	irX, irY = -200, -200
	b.add_irSensorData(id, irX, irY)
	if b.Map[map_center_x-19][map_center_y+19] != map_open {
		t.Errorf("Function add_irSensorData did not add line to map correctly.#2")
	}
	if b.Map[map_center_x-20][map_center_y+20] != map_obstacle {
		print(b.Map[map_center_x-20][map_center_y+20])
		t.Errorf("Function add_irSensorData did not add obstruction.")
	}

	irX, irY = 1000, 1000
	b.multi_robot[b.id2index[id]].x = 150
	b.add_irSensorData(id, irX, irY)
	if b.Map[map_center_x+150+21][map_center_y-21] == map_open {
		t.Errorf("Function add_irSensorData did not respect the max distance. #3")
	}
}
