package pathfinding

import (
	"container/heap"
	"golang-server/config"
	"math"
)

const (
	MIN_OBSTACLE_DISTANCE = 15
	TURN_PENALTY          = 10
)

type Pair struct {
	X, Y int
}

type State struct {
	point         Pair
	prevDirection Pair
	g, h, f       int
	parent        *State
}

type stateQueue []*State

func (pq stateQueue) Len() int           { return len(pq) }
func (pq stateQueue) Less(i, j int) bool { return pq[i].f < pq[j].f }
func (pq stateQueue) Swap(i, j int)      { pq[i], pq[j] = pq[j], pq[i] }

func (pq *stateQueue) Push(x interface{}) {
	*pq = append(*pq, x.(*State))
}

func (pq *stateQueue) Pop() interface{} {
	old := *pq
	n := len(old)
	item := old[n-1]
	*pq = old[0 : n-1]
	return item
}

func isValid(row, col int, grid [config.MapSize][config.MapSize]uint8) bool {
	for dx := -MIN_OBSTACLE_DISTANCE; dx <= MIN_OBSTACLE_DISTANCE; dx++ {
		for dy := -MIN_OBSTACLE_DISTANCE; dy <= MIN_OBSTACLE_DISTANCE; dy++ {
			if math.Abs(float64(dx))+math.Abs(float64(dy)) > MIN_OBSTACLE_DISTANCE*1.75 {
				continue
			}
			r := row + dx
			c := col + dy
			if (r < 0) || (r >= config.MapSize) || (c < 0) || (c >= config.MapSize) {
				return false
			} else if (grid[r][c] == config.MapObstacle) || (grid[r][c] == config.MapUnknown) {
				return false
			}
		}
	}
	return true
}

func isDestination(cur Pair, dest Pair) bool {
	return cur.X == dest.X && cur.Y == dest.Y
}

func heuristic(row, col int, dest Pair) int {
	dx := float64(row - dest.X)
	dy := float64(col - dest.Y)
	return 10*int(math.Sqrt(dx*dx + dy*dy))
}

func traceStatePath(endState *State) []Pair {
	var rev []Pair
	for s := endState; s != nil; s = s.parent {
		rev = append(rev, Pair{X: s.point.X, Y: s.point.Y})
	}

	for i, j := 0, len(rev)-1; i < j; i, j = i+1, j-1 {
		rev[i], rev[j] = rev[j], rev[i]
	}
	return rev
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

	var bestG [config.MapSize][config.MapSize][3][3]int
	for x := 0; x < config.MapSize; x++ {
		for y := 0; y < config.MapSize; y++ {
			for dx := 0; dx < 3; dx++ {
				for dy := 0; dy < 3; dy++ {
					bestG[x][y][dx][dy] = math.MaxInt32
				}
			}
		}
	}

	h0 := heuristic(start.X, start.Y, end)
	startState := &State{
		point:         start,
		prevDirection: Pair{0, 0},
		g:             0, h: h0, f: h0,
		parent: nil,
	}

	bestG[start.X][start.Y][1][1] = 0

	var openList stateQueue
	heap.Init(&openList)
	heap.Push(&openList, startState)

	for openList.Len() > 0 {
		cur := heap.Pop(&openList).(*State)

		if isDestination(cur.point, end) {
			fullPath := traceStatePath(cur)
			return extractTurnPoints(fullPath)
		}

		for dx := -1; dx <= 1; dx++ {
			for dy := -1; dy <= 1; dy++ {
				newX, newY := cur.point.X+dx, cur.point.Y+dy

				if !isValid(newX, newY, grid) {
					continue
				}

				stepCost := 10
				if math.Abs(float64(dx))+math.Abs(float64(dy)) > 1 {
					stepCost = 15
				}

				if cur.prevDirection.X != dx || cur.prevDirection.Y != dy {
					stepCost += TURN_PENALTY
				}
				newG := cur.g + stepCost

				idxDX, idxDY := dx+1, dy+1
				if newG >= bestG[newX][newY][idxDX][idxDY] {
					continue
				}
				bestG[newX][newY][idxDX][idxDY] = newG

				newH := heuristic(newX, newY, end)
				newPoint := &State{
					point:         Pair{newX, newY},
					prevDirection: Pair{dx, dy},
					g:             newG, h: newH, f: newG + newH,
					parent: cur,
				}
				heap.Push(&openList, newPoint)
			}
		}
	}
	return []Pair{}
}