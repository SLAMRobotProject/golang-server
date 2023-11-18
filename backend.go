package main

import (
	"math"
	"time"
)

// Global variable, because it is too large to be passed over channels.
// Writers: only thread_backend.
// Data races will only cause a short delay in the GUI update, so it is not a problem.
var g_fullSlamState *fullSlamState = init_fullSlamState()

// using binary flags to represent the map to allow bitwise operations
const (
	MAP_OPEN     uint8 = 1 << iota //1
	MAP_UNKNOWN                    //2
	MAP_OBSTACLE                   //4
)

type robotState struct {
	x, y, theta                int //cm, degrees
	x_init, y_init, theta_init int
}

func init_robotState(x, y, theta int) *robotState {
	return &robotState{x, y, theta, x, y, theta}
}

type fullSlamState struct {
	Map         [MAP_SIZE][MAP_SIZE]uint8
	multi_robot []robotState
	id2index    map[int]int
}

func init_fullSlamState() *fullSlamState {
	s := fullSlamState{}
	for i := 0; i < MAP_SIZE; i++ {
		for j := 0; j < MAP_SIZE; j++ {
			s.Map[i][j] = MAP_UNKNOWN
		}
	}
	s.id2index = make(map[int]int)

	return &s
}

func thread_backend(
	ch_publish chan<- [3]int,
	ch_robotPending chan<- int,
	ch_receive <-chan advMsg,
	ch_robotInit <-chan [4]int,
) {
	prev_msg := advMsg{}
	position_logger := init_positionLogger()
	pending_init := map[int]struct{}{} //simple and efficient way in golang to create a set to check values.
	for {
		select {
		case msg := <-ch_receive:
			if _, exist := pending_init[msg.id]; exist {
				//skip
			} else if _, exist := g_fullSlamState.id2index[msg.id]; !exist {
				pending_init[msg.id] = struct{}{}
				ch_robotPending <- msg.id //Buffered channel, so it will not block.
			} else {
				//robot update
				new_x, new_y := rotate(float64(msg.x/10), float64(msg.y/10), float64(get_robot(msg.id).theta_init))
				index := g_fullSlamState.id2index[msg.id]
				g_fullSlamState.multi_robot[index].x = int(new_x) + get_robot(msg.id).x_init
				g_fullSlamState.multi_robot[index].y = int(new_y) + get_robot(msg.id).y_init
				g_fullSlamState.multi_robot[index].theta = msg.theta + get_robot(msg.id).theta_init

				//map update, dependent upon an updated robot
				g_fullSlamState.irSensorData_add(msg.id, msg.ir1x, msg.ir1y)
				g_fullSlamState.irSensorData_add(msg.id, msg.ir2x, msg.ir2y)
				g_fullSlamState.irSensorData_add(msg.id, msg.ir3x, msg.ir3y)
				g_fullSlamState.irSensorData_add(msg.id, msg.ir4x, msg.ir4y)
				//log position
				if msg.x != prev_msg.x || msg.y != prev_msg.y || msg.theta != prev_msg.theta {
					position_logger.Printf("%d %d %d %d\n", msg.id, get_robot(msg.id).x, get_robot(msg.id).y, get_robot(msg.id).theta)
				}
			}
			prev_msg = msg
		case msg := <-ch_robotInit:
			id := msg[0]
			g_fullSlamState.id2index[id] = len(get_multiRobotState())
			g_fullSlamState.multi_robot = append(g_fullSlamState.multi_robot, *init_robotState(msg[1], msg[2], msg[3]))
			delete(pending_init, id)
			time.Sleep(time.Second * 10)
		}
	}
}

func find_closest_robot(x, y int) int {
	s := g_fullSlamState
	//find the robot closest to the given point
	min_distance := math.MaxFloat64
	closest_robot := -1
	for id, index := range g_fullSlamState.id2index {
		distance := math.Sqrt(math.Pow(float64(s.multi_robot[index].x-x), 2) + math.Pow(float64(s.multi_robot[index].y-y), 2))
		if distance < min_distance {
			min_distance = distance
			closest_robot = id
		}
	}
	if closest_robot == -1 {
		g_generalLogger.Println("Tried to find closest robot, but no robots were found.")
	}
	return closest_robot

}

func get_multiRobotState() []robotState {
	return g_fullSlamState.multi_robot
}
func get_robot(id int) robotState {
	return g_fullSlamState.multi_robot[g_fullSlamState.id2index[id]]
}
func get_map() [MAP_SIZE][MAP_SIZE]uint8 {
	return g_fullSlamState.Map
}
func get_id2index() map[int]int {
	return g_fullSlamState.id2index
}

func (s *fullSlamState) irSensorData_add(id, irX, irY int) {
	x_map, y_map := s.irSensorData_scaleRotateTranselate(id, irX, irY)
	s.add_line(id, x_map, y_map)
}

func (s *fullSlamState) irSensorData_scaleRotateTranselate(id, x_bodyFrame, y_bodyFrame int) (int, int) {
	// IR data is given in mm and it is relative to the body, so it must be scaled, rotated and transelated to the map.

	//Must flip y_bodyFrame, because the axis is flipped in the robot code.
	y_bodyFrame = -y_bodyFrame

	//rotate
	theta := s.multi_robot[s.id2index[id]].theta
	x_bodyFrame_rotated, y_bodyFrame_rotated := rotate(float64(x_bodyFrame), float64(y_bodyFrame), float64(theta))

	//scale and transelate
	x_map := math.Round(x_bodyFrame_rotated/10) + float64(s.multi_robot[s.id2index[id]].x)
	y_map := math.Round(y_bodyFrame_rotated/10) + float64(s.multi_robot[s.id2index[id]].y)

	return int(x_map), int(y_map)
}

func (s *fullSlamState) add_line(id, x1, y1 int) {
	//x0, y0, x1, y1 is given in map coordinates. With origo as defined in the config.

	x0 := get_robot(id).x
	y0 := get_robot(id).y

	line_length := math.Sqrt(math.Pow(float64(x0-x1), 2) + math.Pow(float64(y0-y1), 2))

	var obstruction bool
	if line_length < IR_SENSOR_MAX_DISTANCE {
		obstruction = true
	} else {
		obstruction = false

		//shorten the line to IR_SENSOR_MAX_DISTANCE, needed for bresenham algorithm
		scale := IR_SENSOR_MAX_DISTANCE / line_length
		x1 = x0 + int(scale*float64(x1-x0))
		y1 = y0 + int(scale*float64(y1-y0))

	}

	//get map index values
	x0_idx, y0_idx := get_mapIndex(x0, y0)
	x1_idx, y1_idx := get_mapIndex(x1, y1)
	//get values in map range
	x1_idx = min(max(x1_idx, 0), MAP_SIZE-1)
	y1_idx = min(max(y1_idx, 0), MAP_SIZE-1)
	x0_idx = min(max(x0_idx, 0), MAP_SIZE-1)
	y0_idx = min(max(y0_idx, 0), MAP_SIZE-1)

	idx_points := bresenham_algorithm(x0_idx, y0_idx, x1_idx, y1_idx)
	for i := 0; i < len(idx_points); i++ {
		x := idx_points[i][0]
		y := idx_points[i][1]
		s.Map[x][y] = MAP_OPEN
	}
	if obstruction {
		s.Map[x1_idx][y1_idx] = MAP_OBSTACLE
	}
}

func get_mapIndex(x, y int) (int, int) {
	//Input is given in map coordinates (i.e. robot positions) with normal axis and origo as defined in the config.
	return MAP_CENTER_X + x, MAP_CENTER_Y - y
}
