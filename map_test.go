package main

import (
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
