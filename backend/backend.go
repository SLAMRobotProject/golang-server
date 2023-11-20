package backend

import (
	"golang-server/config"
	"golang-server/log"
	"golang-server/types"
	"golang-server/utilities"
	"math"
	"time"
)

// using binary flags to represent the map to allow bitwise operations
const (
	MAP_OPEN     uint8 = 1 << iota //1
	MAP_UNKNOWN                    //2
	MAP_OBSTACLE                   //4
)

func init_robotState(x, y, theta int) *types.RobotState {
	return &types.RobotState{X: x, Y: y, Theta: theta, X_init: x, Y_init: y, Theta_init: theta}
}

type fullSlamState struct {
	Map          [config.MAP_SIZE][config.MAP_SIZE]uint8
	new_obstacle [][2]int //new since last gui update
	new_open     [][2]int //new since last gui update
	multi_robot  []types.RobotState
	id2index     map[int]int
}

func init_fullSlamState() *fullSlamState {
	s := fullSlamState{}
	for i := 0; i < config.MAP_SIZE; i++ {
		for j := 0; j < config.MAP_SIZE; j++ {
			s.Map[i][j] = MAP_UNKNOWN
		}
	}
	s.id2index = make(map[int]int)

	return &s
}

// The map is very large and sending it gives a warning. This only sends updates.

func Thread_backend(
	ch_publish chan<- [3]int,
	ch_b2g_robotPendingInit chan<- int,
	ch_receive <-chan types.AdvMsg,
	ch_robotInit <-chan [4]int,
	ch_g2b_command <-chan types.Command,
	ch_b2g_update chan<- types.UpdateGui,
) {
	var state *fullSlamState = init_fullSlamState()

	prev_msg := types.AdvMsg{}
	position_logger := log.Init_positionLogger()
	pending_init := map[int]struct{}{} //simple and efficient way in golang to create a set to check values.
	ch_updateGui_ticker := time.Tick(time.Second / config.GUI_FRAME_RATE)
	for {
		select {
		case <-ch_updateGui_ticker:
			//update gui
			ch_b2g_update <- types.UpdateGui{
				Multi_robot:  state.multi_robot,
				Id2index:     state.id2index,
				New_open:     state.new_open,
				New_obstacle: state.new_obstacle,
			}
			//reset new_open and new_obstacle
			state.new_open = [][2]int{}
			state.new_obstacle = [][2]int{}
		case command := <-ch_g2b_command:
			switch command.Command_type {
			case types.AUTOMATIC_COMMAND:
				id := state.find_closest_robot(command.X, command.Y)
				if id == -1 {
					//already logged in find_closest_robot()
					return
				}
				//convert to mm because robot uses mm, and rotate back from init to get robot body coordinates
				robot := state.get_robot(id)
				x_robotBody, y_robotBody := utilities.Rotate(float64(command.X-robot.X_init)*10, float64(command.Y-robot.Y_init)*10, -float64(robot.Theta_init))
				ch_publish <- [3]int{id, int(x_robotBody), int(y_robotBody)}
				log.G_generalLogger.Println("Publishing automatic input to robot with ID: ", command.Id, " x: ", command.X, " y: ", command.Y, ".")
			case types.MANUAL_COMMAND:
				//convert to mm because robot uses mm, and rotate back from init to get robot body coordinates
				robot := state.get_robot(command.Id)
				x_robotBody, y_robotBody := utilities.Rotate(float64(command.X-robot.X_init)*10, float64(command.Y-robot.Y_init)*10, -float64(robot.Theta_init))
				ch_publish <- [3]int{command.Id, int(x_robotBody), int(y_robotBody)}
				log.G_generalLogger.Println("Publishing manual input to robot with ID: ", command.Id, " x: ", command.X, " y: ", command.Y, ".")
			}
		case msg := <-ch_receive:
			if _, exist := pending_init[msg.Id]; exist {
				//skip
			} else if _, exist := state.id2index[msg.Id]; !exist {
				pending_init[msg.Id] = struct{}{}
				ch_b2g_robotPendingInit <- msg.Id //Buffered channel, so it will not block.
			} else {
				//robot update
				new_x, new_y := utilities.Rotate(float64(msg.X/10), float64(msg.Y/10), float64(state.get_robot(msg.Id).Theta_init))
				index := state.id2index[msg.Id]
				state.multi_robot[index].X = int(new_x) + state.get_robot(msg.Id).X_init
				state.multi_robot[index].Y = int(new_y) + state.get_robot(msg.Id).Y_init
				state.multi_robot[index].Theta = msg.Theta + state.get_robot(msg.Id).Theta_init

				//map update, dependent upon an updated robot
				state.irSensorData_add(msg.Id, msg.Ir1x, msg.Ir1y)
				state.irSensorData_add(msg.Id, msg.Ir2x, msg.Ir2y)
				state.irSensorData_add(msg.Id, msg.Ir3x, msg.Ir3y)
				state.irSensorData_add(msg.Id, msg.Ir4x, msg.Ir4y)
				//log position
				if msg.X != prev_msg.X || msg.Y != prev_msg.Y || msg.Theta != prev_msg.Theta {
					position_logger.Printf("%d %d %d %d\n", msg.Id, state.get_robot(msg.Id).X, state.get_robot(msg.Id).Y, state.get_robot(msg.Id).Theta)
				}
			}
			prev_msg = msg
		case init := <-ch_robotInit:
			id := init[0]
			state.id2index[id] = len(state.multi_robot)
			state.multi_robot = append(state.multi_robot, *init_robotState(init[1], init[2], init[3]))
			delete(pending_init, id)
		}
	}
}

func (s *fullSlamState) find_closest_robot(x, y int) int {
	//find the robot closest to the given point
	min_distance := math.MaxFloat64
	closest_robot := -1
	for id, index := range s.id2index {
		distance := math.Sqrt(math.Pow(float64(s.multi_robot[index].X-x), 2) + math.Pow(float64(s.multi_robot[index].Y-y), 2))
		if distance < min_distance {
			min_distance = distance
			closest_robot = id
		}
	}
	if closest_robot == -1 {
		log.G_generalLogger.Println("Tried to find closest robot, but no robots were found.")
	}
	return closest_robot
}

func (s *fullSlamState) set_mapValue(x, y int, value uint8) {
	s.Map[x][y] = value
	switch value {
	case MAP_OPEN:
		s.new_open = append(s.new_open, [2]int{x, y})
	case MAP_OBSTACLE:
		s.new_obstacle = append(s.new_obstacle, [2]int{x, y})
	}
}

func (s *fullSlamState) get_robot(id int) types.RobotState {
	return s.multi_robot[s.id2index[id]]
}

func (s *fullSlamState) irSensorData_add(id, irX, irY int) {
	x_map, y_map := s.irSensorData_scaleRotateTranselate(id, irX, irY)
	s.add_lineToMap(id, x_map, y_map)
}

func (s *fullSlamState) irSensorData_scaleRotateTranselate(id, x_bodyFrame, y_bodyFrame int) (int, int) {
	// IR data is given in mm and it is relative to the body, so it must be scaled, rotated and transelated to the map.

	//Must flip y_bodyFrame, because the axis is flipped in the robot code.
	y_bodyFrame = -y_bodyFrame

	//rotate
	theta := s.multi_robot[s.id2index[id]].Theta
	x_bodyFrame_rotated, y_bodyFrame_rotated := utilities.Rotate(float64(x_bodyFrame), float64(y_bodyFrame), float64(theta))

	//scale and transelate
	x_map := math.Round(x_bodyFrame_rotated/10) + float64(s.multi_robot[s.id2index[id]].X)
	y_map := math.Round(y_bodyFrame_rotated/10) + float64(s.multi_robot[s.id2index[id]].Y)

	return int(x_map), int(y_map)
}

func (s *fullSlamState) add_lineToMap(id, x1, y1 int) {
	//x0, y0, x1, y1 is given in map coordinates. With origo as defined in the config.
	x0 := s.get_robot(id).X
	y0 := s.get_robot(id).Y

	line_length := math.Sqrt(math.Pow(float64(x0-x1), 2) + math.Pow(float64(y0-y1), 2))

	var obstruction bool
	if line_length < config.IR_SENSOR_MAX_DISTANCE {
		obstruction = true
	} else {
		obstruction = false

		//shorten the line to config.IR_SENSOR_MAX_DISTANCE, needed for bresenham algorithm
		scale := config.IR_SENSOR_MAX_DISTANCE / line_length
		x1 = x0 + int(scale*float64(x1-x0))
		y1 = y0 + int(scale*float64(y1-y0))

	}

	//get map index values
	x0_idx, y0_idx := calculate_mapIndex(x0, y0)
	x1_idx, y1_idx := calculate_mapIndex(x1, y1)
	//get values in map range
	x1_idx = min(max(x1_idx, 0), config.MAP_SIZE-1)
	y1_idx = min(max(y1_idx, 0), config.MAP_SIZE-1)
	x0_idx = min(max(x0_idx, 0), config.MAP_SIZE-1)
	y0_idx = min(max(y0_idx, 0), config.MAP_SIZE-1)

	idx_points := utilities.Bresenham_algorithm(x0_idx, y0_idx, x1_idx, y1_idx)
	for i := 0; i < len(idx_points); i++ {
		x := idx_points[i][0]
		y := idx_points[i][1]
		s.set_mapValue(x, y, MAP_OPEN)
	}
	if obstruction {
		s.set_mapValue(x1_idx, y1_idx, MAP_OBSTACLE)
	}
}

func calculate_mapIndex(x, y int) (int, int) {
	//Input is given in map coordinates (i.e. robot positions) with normal axis and origo as defined in the config.
	return config.MAP_CENTER_X + x, config.MAP_CENTER_Y - y
}
