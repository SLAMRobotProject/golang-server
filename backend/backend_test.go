package backend

import (
	"golang-server/config"
	"golang-server/utilities"
	"math"
	"testing"
)

func confirmBresenhamOutput(knownPoints, calculatedPoints [][]int) bool {
	success := false
	for i := 0; i < len(knownPoints); i++ {
		if knownPoints[i][0] == calculatedPoints[i][0] && knownPoints[i][1] == calculatedPoints[i][1] {
			success = true
		} else {
			success = false
			break
		}
	}
	if success != true { //bresenham algorithm might have failed, because of wrong order of points
		for i := len(knownPoints) - 1; i >= 0; i-- {
			if knownPoints[i][0] == calculatedPoints[i][0] && knownPoints[i][1] == calculatedPoints[i][1] {
				success = true
			} else {
				success = false
				break
			}
		}
	}
	return success
}

func baseTestBresenhamAlgorithm(t *testing.T, knownPoints [][]int, x0, y0, x1, y1 int) bool {
	success := true
	calculatedPoints := utilities.BresenhamAlgorithm(x0, y0, x1, y1)
	if len(calculatedPoints) != len(knownPoints) {
		success = false
	} else {
		success = confirmBresenhamOutput(knownPoints, calculatedPoints)
	}
	if success != true {
		t.Errorf("Test of bresenhamAlgorithm failed. Expected: %v. Got: %v", knownPoints, calculatedPoints)
	}
	return success
}

func TestBresenhamAlgorithm(t *testing.T) {
	//Testing multiple directions

	knownPoints := [][]int{{0, 0}, {1, 1}, {2, 2}, {3, 3}, {4, 4}}
	baseTestBresenhamAlgorithm(t, knownPoints, 0, 0, 4, 4)

	knownPoints = [][]int{{0, 0}, {-1, -1}, {-2, -2}, {-3, -3}, {-4, -4}}
	baseTestBresenhamAlgorithm(t, knownPoints, 0, 0, -4, -4)

	knownPoints = [][]int{{0, 0}, {-1, 1}, {-2, 1}, {-3, 2}, {-4, 2}, {-5, 3}}
	baseTestBresenhamAlgorithm(t, knownPoints, 0, 0, -5, 3)
}

func TestAddLineToMap(t *testing.T) {
	s := initFullSlamState()
	id := 2
	s.id2index[id] = len(s.multiRobot)
	s.multiRobot = append(s.multiRobot, *initRobotState(0, 0, 90))

	x1, y1 := 20, 20
	x1Index, y1Index := calculateMapIndex(x1, y1)
	s.addLineToMap(id, x1, y1)
	if s.areaMap[x1Index][y1Index] != mapObstacle {
		t.Errorf("Function addLineToMap did not add obstacle to map correctly.")
	}

	x1, y1 = -20, -20
	x1ModIdx, y1ModIdx := calculateMapIndex(x1+1, y1+1) //modified to test the point before the obstacle
	s.addLineToMap(id, x1, y1)
	if s.areaMap[x1ModIdx][y1ModIdx] != mapOpen {
		t.Errorf("Function addLineToMap did not add line to map correctly.")
	}

	x1, y1 = 40, 40
	x1Index, y1Index = calculateMapIndex(x1, y1)
	if s.areaMap[x1Index][y1Index] == mapUnknown {
		s.addLineToMap(id, x1, y1)
		x1ModIdx, y1ModIdx = calculateMapIndex(21, 21) //modified to respect a max distance of 30
		if math.Sqrt(float64(x1*x1+y1*y1)) > config.IrSensorMaxDistance && s.areaMap[x1Index][y1Index] != mapUnknown {
			t.Errorf("Function addLineToMap did not respect the max distance.")
		} else if s.areaMap[x1ModIdx][y1ModIdx] != mapOpen {
			t.Errorf("Function addLineToMap did not add line to map correctly.")
		}
	} else {
		t.Errorf("Could not test max distance, because the point is already known.")
	}
}

func TestIrSensorDataAdd(t *testing.T) {
	s := initFullSlamState()
	id := 2
	s.id2index[id] = len(s.multiRobot)
	s.multiRobot = append(s.multiRobot, *initRobotState(0, 0, 90))

	irX, irY := 500, 500 //written in milimeters (because of the robot code), while the map is in centimeters
	s.addIrSensorData(id, irX, irY)
	if s.areaMap[config.MapCenterX+21][config.MapCenterY-21] != mapOpen {
		t.Errorf("Function addIrSensorData did not add line to map correctly.#1")
	}

	irX, irY = -200, -200
	s.addIrSensorData(id, irX, irY)
	if s.areaMap[config.MapCenterX-19][config.MapCenterY+19] != mapOpen {
		t.Errorf("Function addIrSensorData did not add line to map correctly.#2")
	}
	if s.areaMap[config.MapCenterX-20][config.MapCenterY+20] != mapObstacle {
		print(s.areaMap[config.MapCenterX-20][config.MapCenterY+20])
		t.Errorf("Function addIrSensorData did not add obstruction.")
	}

	irX, irY = 1000, 1000
	s.multiRobot[s.id2index[id]].X = -150
	s.addIrSensorData(id, irX, irY)
	if s.areaMap[config.MapCenterX+21][config.MapCenterY+150-21] == mapOpen {
		t.Errorf("Function addIrSensorData did not respect the max distance. #3")
	}
}
