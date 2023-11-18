package main

import (
	"math"
	"time"
)

// Global variable, because it is too large to be passed over channels.
// Writers: only thread_backend.
// Data races will only cause a short delay in the GUI update, so it is not a problem.
var backend *Backend = NewBackend()

// using binary flags to represent the map to allow bitwise operations
const (
	map_open     uint8 = 1 << iota //1
	map_unknown                    //2
	map_obstacle                   //4
)

type Robot struct {
	x, y, theta                int //cm, degrees
	x_init, y_init, theta_init int
}

func NewRobot(x, y, theta int) *Robot {
	return &Robot{x, y, theta, x, y, theta}
}

type Backend struct {
	Map         [map_size][map_size]uint8
	multi_robot []Robot
	id2index    map[int]int
}

func NewBackend() *Backend {
	backend := Backend{}
	for i := 0; i < map_size; i++ {
		for j := 0; j < map_size; j++ {
			backend.Map[i][j] = map_unknown
		}
	}
	backend.id2index = make(map[int]int)

	return &backend
}

func thread_backend(
	ch_publish chan<- [3]int,
	ch_robotPending chan<- int,
	ch_receive <-chan adv_msg,
	ch_robotInit <-chan [4]int,
) {
	prev_msg := adv_msg{}
	position_logger := position_logger_init()
	pending_init := map[int]struct{}{} //simple and efficient way in golang to create a set to check values.
	for {
		select {
		case msg := <-ch_receive:
			if _, exist := pending_init[msg.id]; exist {
				//skip
			} else if _, exist := backend.id2index[msg.id]; !exist {
				pending_init[msg.id] = struct{}{}
				ch_robotPending <- msg.id //Buffered channel, so it will not block.
			} else {
				//robot update
				index := backend.id2index[msg.id]
				new_x, new_y := rotate(float64(msg.x/10), float64(msg.y/10), float64(backend.multi_robot[index].theta_init))
				backend.multi_robot[index].x = int(new_x) + backend.multi_robot[index].x_init
				backend.multi_robot[index].y = int(new_y) + backend.multi_robot[index].y_init
				backend.multi_robot[index].theta = msg.theta + backend.multi_robot[index].theta_init

				//map update, dependent upon an updated robot
				backend.add_irSensorData(msg.id, msg.ir1x, msg.ir1y)
				backend.add_irSensorData(msg.id, msg.ir2x, msg.ir2y)
				backend.add_irSensorData(msg.id, msg.ir3x, msg.ir3y)
				backend.add_irSensorData(msg.id, msg.ir4x, msg.ir4y)
				//log position
				if msg.x != prev_msg.x || msg.y != prev_msg.y || msg.theta != prev_msg.theta {
					position_logger.Printf("%d %d %d %d\n", msg.id, backend.get_x(msg.id), backend.get_y(msg.id), backend.get_theta(msg.id))
				}
			}
			prev_msg = msg
		case msg := <-ch_robotInit:
			id := msg[0]
			backend.id2index[id] = len(backend.multi_robot)
			backend.multi_robot = append(backend.multi_robot, *NewRobot(msg[1], msg[2], msg[3]))
			delete(pending_init, id)
			time.Sleep(time.Second * 10)
		}
	}
}

func (b *Backend) find_closest_robot(x, y int) int {
	//find the robot closest to the given point
	min_distance := math.MaxFloat64
	closest_robot := -1
	for id, index := range b.id2index {
		distance := math.Sqrt(math.Pow(float64(b.multi_robot[index].x-x), 2) + math.Pow(float64(b.multi_robot[index].y-y), 2))
		if distance < min_distance {
			min_distance = distance
			closest_robot = id
		}
	}
	if closest_robot == -1 {
		general_logger.Println("Tried to find closest robot, but no robots were found.")
	}
	return closest_robot

}

func (b *Backend) get_x(id int) int {
	return b.multi_robot[b.id2index[id]].x
}
func (b *Backend) get_y(id int) int {
	return b.multi_robot[b.id2index[id]].y
}
func (b *Backend) get_theta(id int) int {
	return b.multi_robot[b.id2index[id]].theta
}

func (b *Backend) add_irSensorData(id, irX, irY int) {
	x_map, y_map := b.irSensor_scaleRotateTranselate(id, irX, irY)
	b.add_line(id, x_map, y_map)
}

func (b *Backend) irSensor_scaleRotateTranselate(id, x_bodyFrame, y_bodyFrame int) (int, int) {
	// IR data is given in mm and it is relative to the body, so it must be scaled, rotated and transelated to the map.

	//Must flip y_bodyFrame, because the axis is flipped in the robot code.
	y_bodyFrame = -y_bodyFrame

	//rotate
	theta := b.multi_robot[b.id2index[id]].theta
	x_bodyFrame_rotated, y_bodyFrame_rotated := rotate(float64(x_bodyFrame), float64(y_bodyFrame), float64(theta))

	//scale and transelate
	x_map := math.Round(x_bodyFrame_rotated/10) + float64(b.multi_robot[b.id2index[id]].x)
	y_map := math.Round(y_bodyFrame_rotated/10) + float64(b.multi_robot[b.id2index[id]].y)

	return int(x_map), int(y_map)
}

func (b *Backend) add_line(id, x1, y1 int) {
	//x0, y0, x1, y1 is given in map coordinates. With origo as defined in the config.

	x0 := b.multi_robot[b.id2index[id]].x
	y0 := b.multi_robot[b.id2index[id]].y

	line_length := math.Sqrt(math.Pow(float64(x0-x1), 2) + math.Pow(float64(y0-y1), 2))

	var obstruction bool
	if line_length < irSensor_maxDistance {
		obstruction = true
	} else {
		obstruction = false

		//shorten the line to irSensor_maxDistance, needed for bresenham algorithm
		scale := irSensor_maxDistance / line_length
		x1 = x0 + int(scale*float64(x1-x0))
		y1 = y0 + int(scale*float64(y1-y0))

	}

	//get map index values
	x0_idx, y0_idx := get_map_index(x0, y0)
	x1_idx, y1_idx := get_map_index(x1, y1)
	//get values in map range
	x1_idx = min(max(x1_idx, 0), map_size-1)
	y1_idx = min(max(y1_idx, 0), map_size-1)
	x0_idx = min(max(x0_idx, 0), map_size-1)
	y0_idx = min(max(y0_idx, 0), map_size-1)

	idx_points := bresenham_algorithm(x0_idx, y0_idx, x1_idx, y1_idx)
	for i := 0; i < len(idx_points); i++ {
		x := idx_points[i][0]
		y := idx_points[i][1]
		b.Map[x][y] = map_open
	}
	if obstruction {
		b.Map[x1_idx][y1_idx] = map_obstacle
	}
}

func get_map_index(x, y int) (int, int) {
	//Input is given in map coordinates (i.e. robot positions) with normal axis and origo as defined in the config.
	return map_center_x + x, map_center_y - y
}
