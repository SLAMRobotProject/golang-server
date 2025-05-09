package pathfinding

import (
	"golang-server/config"
	"container/heap"
	"math"
)

const (
	NUM_PARENTS           = 4
	MIN_OBSTACLE_DISTANCE = 15
	TURN_PENALTY          = 2
)

type Pair struct {
	X, Y int
}

type pPair struct {
	point Pair
	f     int
	index int
}

type Cell struct {
	parents    [NUM_PARENTS]Pair
	numParents int
	f, g, h    int
}

type PriorityQueue []*pPair

func (pq PriorityQueue) Len() int {
	return len(pq)
}

func (pq PriorityQueue) Less(i, j int) bool {
	return pq[i].f < pq[j].f
}

func (pq PriorityQueue) Swap(i, j int) {
	pq[i], pq[j] = pq[j], pq[i]
	pq[i].index = i
	pq[j].index = j
}

func (pq *PriorityQueue) Push(x interface{}) {
	item := x.(*pPair)
	item.index = len(*pq)
	*pq = append(*pq, item)
}

func (pq *PriorityQueue) Pop() interface{} {
	old := *pq
	n := len(old)
	item := old[n-1]
	*pq = old[0 : n-1]
	return item
}

func (pq *PriorityQueue) Enqueue(p Pair, f int) {
	heap.Push(pq, &pPair{point: p, f: f})
}

func (pq *PriorityQueue) Dequeue() *pPair {
	return heap.Pop(pq).(*pPair)
}

func isValid(row, col int, grid [config.MapSize][config.MapSize]uint8) bool {
	for dx := -MIN_OBSTACLE_DISTANCE; dx <= MIN_OBSTACLE_DISTANCE; dx++ {
		for dy := -MIN_OBSTACLE_DISTANCE; dy <= MIN_OBSTACLE_DISTANCE; dy++ {
			r := row + dx
			c := col + dy
			if r < 0 || r >= config.MapSize || c < 0 || c >= config.MapSize || grid[r][c] == config.MapObstacle {
				return false
			}
		}
	}
	return true
}

func isDestination(row, col int, dest Pair) bool {
	return row == dest.X && col == dest.Y
}

func heuristic(row, col int, dest Pair) int {
	return int(math.Abs(float64(row-dest.X)) + math.Abs(float64(col-dest.Y)))
}

func tracePath(cellDetails [config.MapSize][config.MapSize]Cell, dest Pair) []Pair {
	row, col := dest.X, dest.Y
	var path []Pair

	for !(cellDetails[row][col].parents[0].X == row && cellDetails[row][col].parents[0].Y == col) {
		path = append(path, Pair{row, col})
		var tempRow, tempCol int
		if len(path) > 1 {
			for i := 0; i < cellDetails[row][col].numParents; i++ {
				prev := Pair{path[len(path)-2].X - path[len(path)-1].X, path[len(path)-2].Y - path[len(path)-1].Y}
				next := Pair{path[len(path)-1].X - cellDetails[row][col].parents[i].X, path[len(path)-1].Y - cellDetails[row][col].parents[i].Y}
				if prev == next {
					tempRow = cellDetails[row][col].parents[i].X
					tempCol = cellDetails[row][col].parents[i].Y
					break
				} else if i == cellDetails[row][col].numParents-1 {
					tempRow = cellDetails[row][col].parents[i].X
					tempCol = cellDetails[row][col].parents[i].Y
				}
			}
		} else {
			tempRow = cellDetails[row][col].parents[0].X
			tempCol = cellDetails[row][col].parents[0].Y
		}
		row, col = tempRow, tempCol
	}
	path = append(path, Pair{row, col})

	for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
		path[i], path[j] = path[j], path[i]
	}

	return path
}

func extractTurnPoints(fullPath []Pair) []Pair {
	if len(fullPath) < 2 {
		return fullPath
	}

	var turnPoints []Pair
	turnPoints = append(turnPoints, fullPath[0])

	for i := 1; i < len(fullPath)-1; i++ {
		dx1 := fullPath[i].X - fullPath[i-1].X
		dy1 := fullPath[i].Y - fullPath[i-1].Y
		dx2 := fullPath[i+1].X - fullPath[i].X
		dy2 := fullPath[i+1].Y - fullPath[i].Y

		if dx1 != dx2 || dy1 != dy2 {
			turnPoints = append(turnPoints, fullPath[i])
		}
	}

	turnPoints = append(turnPoints, fullPath[len(fullPath)-1])
	return turnPoints
}

func A_Star(grid [config.MapSize][config.MapSize]uint8, start, end Pair) []Pair {
	if !isValid(start.X, start.Y, grid) || !isValid(end.X, end.Y, grid) {
		return []Pair{}
	}
	if isDestination(start.X, start.Y, end) {
		return []Pair{}
	}

	var closedList [config.MapSize][config.MapSize]bool
	var cellDetails [config.MapSize][config.MapSize]Cell

	for i := 0; i < config.MapSize; i++ {
		for j := 0; j < config.MapSize; j++ {
			cellDetails[i][j].f = math.MaxInt32
			cellDetails[i][j].g = math.MaxInt32
			cellDetails[i][j].h = math.MaxInt32
			cellDetails[i][j].numParents = 0
			cellDetails[i][j].parents[0] = Pair{-1, -1}
		}
	}

	cellDetails[start.X][start.Y] = Cell{parents: [NUM_PARENTS]Pair{{start.X, start.Y}}, numParents: 1, f: 0, g: 0, h: 0}

	openList := &PriorityQueue{}
	heap.Init(openList)
	openList.Enqueue(start, 0)

	for openList.Len() > 0 {
		p := openList.Dequeue()
		row, col := p.point.X, p.point.Y
		closedList[row][col] = true

		if isDestination(row, col, end) {
			fullPath := tracePath(cellDetails, end)
			return extractTurnPoints(fullPath)
		}

		for i := 0; i < cellDetails[row][col].numParents; i++ {
			dRow := row - cellDetails[row][col].parents[i].X
			dCol := col - cellDetails[row][col].parents[i].Y
			for dx := -1; dx <= 1; dx++ {
				for dy := -1; dy <= 1; dy++ {
					if (dx == 0 && dy == 0) {
						continue
					}
					r := row + dx
					c := col + dy
					if r >= 0 && r < config.MapSize && c >= 0 && c < config.MapSize && isValid(r, c, grid) {
						newG := cellDetails[row][col].g + 1
						if dRow != dx || dCol != dy {
							newG += TURN_PENALTY
						}
						newH := heuristic(r, c, end)
						newF := newG + newH
						if cellDetails[r][c].f == math.MaxInt32 || cellDetails[r][c].f-TURN_PENALTY > newF {
							cellDetails[r][c] = Cell{f: newF, g: newG, h: newH, parents: [NUM_PARENTS]Pair{{row, col}}, numParents: 1}
							openList.Enqueue(Pair{r, c}, newF)
						} else if cellDetails[r][c].f+TURN_PENALTY > newF {
							n := cellDetails[r][c].numParents
							if n < NUM_PARENTS {
								cellDetails[r][c].parents[n] = Pair{row, col}
								cellDetails[r][c].numParents++
							}
							if cellDetails[r][c].f > newF {
								cellDetails[r][c].f = newF
								cellDetails[r][c].g = newG
							}
						}
					}
				}
			}
		}
	}
	return []Pair{}
}