package pathfinding

import (
	"golang-server/config"
	"golang-server/backend"
	"testing"
)

func createEmptyGrid() [config.MapSize][config.MapSize]uint8 {
	var grid [config.MapSize][config.MapSize]uint8
	for i := 0; i < config.MapSize; i++ {
		for j := 0; j < config.MapSize; j++ {
			grid[i][j] = backend.MapOpen
		}
	}
	return grid
}

func TestAStar_SimplePath(t *testing.T) {
	grid := createEmptyGrid()
	start := Pair{X: 1, Y: 1}
	end := Pair{X: 5, Y: 5}

	path := A_Star(grid, start, end)

	if len(path) == 0 {
		t.Fatal("Expected path, got none")
	}
	if path[0] != start {
		t.Errorf("Expected path to start at %v, got %v", start, path[0])
	}
	if path[len(path)-1] != end {
		t.Errorf("Expected path to end at %v, got %v", end, path[len(path)-1])
	}
}

func TestAStar_NoPathDueToObstacle(t *testing.T) {
	grid := createEmptyGrid()

	// Blokker en hel linje i X-retning
	for i := 0; i < config.MapSize; i++ {
		grid[3][i] = backend.MapObstacle
	}

	start := Pair{X: 2, Y: 2}
	end := Pair{X: 4, Y: 2}

	path := A_Star(grid, start, end)

	if len(path) != 0 {
		t.Errorf("Expected no path due to wall, got %d steps", len(path))
	}
}

func TestAStar_StartEqualsEnd(t *testing.T) {
	grid := createEmptyGrid()
	start := Pair{X: 10, Y: 10}

	path := A_Star(grid, start, start)

	if len(path) != 0 {
		t.Errorf("Expected empty path when start equals end, got %d steps", len(path))
	}
}

func TestAStar_InvalidStart(t *testing.T) {
	grid := createEmptyGrid()
	grid[1][1] = backend.MapObstacle

	start := Pair{X: 1, Y: 1}
	end := Pair{X: 5, Y: 5}

	path := A_Star(grid, start, end)

	if len(path) != 0 {
		t.Errorf("Expected no path from invalid start, got %d steps", len(path))
	}
}

func TestAStar_InvalidEnd(t *testing.T) {
	grid := createEmptyGrid()
	grid[5][5] = backend.MapObstacle

	start := Pair{X: 1, Y: 1}
	end := Pair{X: 5, Y: 5}

	path := A_Star(grid, start, end)

	if len(path) != 0 {
		t.Errorf("Expected no path to invalid end, got %d steps", len(path))
	}
}
