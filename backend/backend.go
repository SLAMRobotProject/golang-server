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

func initRobotState(x, y, theta int) *types.RobotState {
	return &types.RobotState{X: x, Y: y, Theta: theta, XInit: x, YInit: y, ThetaInit: theta}
}

type fullSlamState struct {
	areaMap     [config.MAP_SIZE][config.MAP_SIZE]uint8
	newObstacle [][2]int //new since last gui update
	newOpen     [][2]int //new since last gui update
	multiRobot  []types.RobotState
	id2index    map[int]int
}

func initFullSlamState() *fullSlamState {
	s := fullSlamState{}
	for i := 0; i < config.MAP_SIZE; i++ {
		for j := 0; j < config.MAP_SIZE; j++ {
			s.areaMap[i][j] = MAP_UNKNOWN
		}
	}
	s.id2index = make(map[int]int)

	return &s
}

// The map is very large and sending it gives a warning. This only sends updates.

func ThreadBackend(
	chPublish chan<- [3]int,
	chReceive <-chan types.AdvMsg,
	chB2gRobotPendingInit chan<- int,
	chB2gUpdate chan<- types.UpdateGui,
	chG2bRobotInit <-chan [4]int,
	chG2bCommand <-chan types.Command,
) {
	var state *fullSlamState = initFullSlamState()

	prevMsg := types.AdvMsg{}
	positionLogger := log.InitPositionLogger()
	pendingInit := map[int]struct{}{} //simple and efficient way in golang to create a set to check values.
	guiUpdateTicker := time.NewTicker(time.Second / config.GUI_FRAME_RATE)
	for {
		select {
		case <-guiUpdateTicker.C:
			//update gui
			chB2gUpdate <- types.UpdateGui{
				MultiRobot:  state.multiRobot,
				Id2index:    state.id2index,
				NewOpen:     state.newOpen,
				NewObstacle: state.newObstacle,
			}
			//reset newOpen and newObstacle
			state.newOpen = [][2]int{}
			state.newObstacle = [][2]int{}
		case command := <-chG2bCommand:
			switch command.Command_type {
			case types.AUTOMATIC_COMMAND:
				id := state.findClosestRobot(command.X, command.Y)
				if id == -1 {
					//already logged in findClosestRobot()
					return
				}
				//convert to mm because robot uses mm, and rotate back from init to get robot body coordinates
				robot := state.getRobot(id)
				x_robotBody, y_robotBody := utilities.Rotate(float64(command.X-robot.XInit)*10, float64(command.Y-robot.YInit)*10, -float64(robot.ThetaInit))
				chPublish <- [3]int{id, int(x_robotBody), int(y_robotBody)}
				log.GGeneralLogger.Println("Publishing automatic input to robot with ID: ", command.Id, " x: ", command.X, " y: ", command.Y, ".")
			case types.MANUAL_COMMAND:
				//convert to mm because robot uses mm, and rotate back from init to get robot body coordinates
				robot := state.getRobot(command.Id)
				x_robotBody, y_robotBody := utilities.Rotate(float64(command.X-robot.XInit)*10, float64(command.Y-robot.YInit)*10, -float64(robot.ThetaInit))
				chPublish <- [3]int{command.Id, int(x_robotBody), int(y_robotBody)}
				log.GGeneralLogger.Println("Publishing manual input to robot with ID: ", command.Id, " x: ", command.X, " y: ", command.Y, ".")
			}
		case msg := <-chReceive:
			if _, exist := pendingInit[msg.Id]; exist {
				//skip
			} else if _, exist := state.id2index[msg.Id]; !exist {
				pendingInit[msg.Id] = struct{}{}
				chB2gRobotPendingInit <- msg.Id //Buffered channel, so it will not block.
			} else {
				//robot update
				new_x, new_y := utilities.Rotate(float64(msg.X/10), float64(msg.Y/10), float64(state.getRobot(msg.Id).ThetaInit))
				index := state.id2index[msg.Id]
				state.multiRobot[index].X = int(new_x) + state.getRobot(msg.Id).XInit
				state.multiRobot[index].Y = int(new_y) + state.getRobot(msg.Id).YInit
				state.multiRobot[index].Theta = msg.Theta + state.getRobot(msg.Id).ThetaInit

				//map update, dependent upon an updated robot
				state.addIrSensorData(msg.Id, msg.Ir1x, msg.Ir1y)
				state.addIrSensorData(msg.Id, msg.Ir2x, msg.Ir2y)
				state.addIrSensorData(msg.Id, msg.Ir3x, msg.Ir3y)
				state.addIrSensorData(msg.Id, msg.Ir4x, msg.Ir4y)
				//log position
				if msg.X != prevMsg.X || msg.Y != prevMsg.Y || msg.Theta != prevMsg.Theta {
					positionLogger.Printf("%d %d %d %d\n", msg.Id, state.getRobot(msg.Id).X, state.getRobot(msg.Id).Y, state.getRobot(msg.Id).Theta)
				}
			}
			prevMsg = msg
		case init := <-chG2bRobotInit:
			id := init[0]
			state.id2index[id] = len(state.multiRobot)
			state.multiRobot = append(state.multiRobot, *initRobotState(init[1], init[2], init[3]))
			delete(pendingInit, id)
		}
	}
}

func (s *fullSlamState) setMapValue(x, y int, value uint8) {
	s.areaMap[x][y] = value
	switch value {
	case MAP_OPEN:
		s.newOpen = append(s.newOpen, [2]int{x, y})
	case MAP_OBSTACLE:
		s.newObstacle = append(s.newObstacle, [2]int{x, y})
	}
}

func (s *fullSlamState) getRobot(id int) types.RobotState {
	return s.multiRobot[s.id2index[id]]
}

func (s *fullSlamState) addIrSensorData(id, irX, irY int) {
	x_map, y_map := s.transformIrSensorData(id, irX, irY)
	s.addLineToMap(id, x_map, y_map)
}

func (s *fullSlamState) transformIrSensorData(id, x_bodyFrame, y_bodyFrame int) (int, int) {
	// IR data is given in mm and it is relative to the body, so it must be scaled, rotated and transelated to the map.

	//Must flip y_bodyFrame, because the axis is flipped in the robot code.
	y_bodyFrame = -y_bodyFrame

	//rotate
	theta := s.multiRobot[s.id2index[id]].Theta
	x_bodyFrame_rotated, y_bodyFrame_rotated := utilities.Rotate(float64(x_bodyFrame), float64(y_bodyFrame), float64(theta))

	//scale and transelate
	x_map := math.Round(x_bodyFrame_rotated/10) + float64(s.multiRobot[s.id2index[id]].X)
	y_map := math.Round(y_bodyFrame_rotated/10) + float64(s.multiRobot[s.id2index[id]].Y)

	return int(x_map), int(y_map)
}

func (s *fullSlamState) addLineToMap(id, x1, y1 int) {
	//x0, y0, x1, y1 is given in map coordinates. With origo as defined in the config.
	x0 := s.getRobot(id).X
	y0 := s.getRobot(id).Y

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
	x0_idx, y0_idx := calculateMapIndex(x0, y0)
	x1_idx, y1_idx := calculateMapIndex(x1, y1)
	//get values in map range
	x1_idx = min(max(x1_idx, 0), config.MAP_SIZE-1)
	y1_idx = min(max(y1_idx, 0), config.MAP_SIZE-1)
	x0_idx = min(max(x0_idx, 0), config.MAP_SIZE-1)
	y0_idx = min(max(y0_idx, 0), config.MAP_SIZE-1)

	idx_points := utilities.BresenhamAlgorithm(x0_idx, y0_idx, x1_idx, y1_idx)
	for i := 0; i < len(idx_points); i++ {
		x := idx_points[i][0]
		y := idx_points[i][1]
		s.setMapValue(x, y, MAP_OPEN)
	}
	if obstruction {
		s.setMapValue(x1_idx, y1_idx, MAP_OBSTACLE)
	}
}

func calculateMapIndex(x, y int) (int, int) {
	//Input is given in map coordinates (i.e. robot positions) with normal axis and origo as defined in the config.
	return config.MAP_CENTER_X + x, config.MAP_CENTER_Y - y
}
