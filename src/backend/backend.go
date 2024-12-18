package backend

import (
	//"fmt"
	"golang-server/config"
	"golang-server/log"
	"golang-server/types"
	"golang-server/utilities"
	"math"
	"time"
	//"golang.org/x/text/cases"
)

// using binary flags to represent the map to allow bitwise operations
const (
	mapOpen     uint8 = 1 << iota //1
	mapUnknown                    //2
	mapObstacle                   //4
)

func initRobotState(x, y, theta int) *types.RobotState {
	return &types.RobotState{X: x, Y: y, Theta: theta, XInit: x, YInit: y, ThetaInit: theta}
}

type fullSlamState struct {
	areaMap     [config.MapSize][config.MapSize]uint8
	newObstacle [][2]int //new since last gui update
	newOpen     [][2]int //new since last gui update
	multiRobot  []types.RobotState
	id2index    map[int]int
}

func initFullSlamState() *fullSlamState {
	s := fullSlamState{}
	for i := 0; i < config.MapSize; i++ {
		for j := 0; j < config.MapSize; j++ {
			s.areaMap[i][j] = mapUnknown
		}
	}
	s.id2index = make(map[int]int)

	return &s
}

// The map is very large and sending it gives a warning. This only sends updates.

func ThreadBackend(
	chPublish chan<- [3]int,
	chPublishInit chan<-[4]int,
	chReceive <-chan types.AdvMsg,
	chB2gRobotPendingInit chan<- int,
	chB2gUpdate chan<- types.UpdateGui,
	chG2bRobotInit <-chan [4]int,
	chG2bCommand <-chan types.Command,
	chReceiveMapFromRobot <-chan types.RectangleMsg,
	chB2gMapRectangle chan<- types.RectangleMsg,
) {
	var state *fullSlamState = initFullSlamState()

	prevMsg := types.AdvMsg{}
	positionLogger := log.InitPositionLogger()
	pendingInit := map[int]struct{}{} //simple and efficient way in golang to create a set to check values.
	guiUpdateTicker := time.NewTicker(time.Second / config.GuiFrameRate)
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
			switch command.CommandType {
			case types.AutomaticCommand:
				id := state.findClosestRobot(command.X, command.Y)
				if id == -1 {
					//already logged in findClosestRobot()
					return
				}
				//convert to mm because robot uses mm, and rotate back from init to get robot body coordinates
				robot := state.getRobot(id)
				xRobotBody, yRobotBody := utilities.Rotate(float64(command.X-robot.XInit)*10, float64(command.Y-robot.YInit)*10, -float64(robot.ThetaInit))
				chPublish <- [3]int{id, int(xRobotBody), int(yRobotBody)}
				log.GGeneralLogger.Println("Publishing automatic input to robot with ID: ", command.Id, " x: ", command.X, " y: ", command.Y, ".")
			case types.ManualCommand:
				//convert to mm because robot uses mm, and rotate back from init to get robot body coordinates
				robot := state.getRobot(command.Id)
				xRobotBody, yRobotBody := utilities.Rotate(float64(command.X-robot.XInit)*10, float64(command.Y-robot.YInit)*10, -float64(robot.ThetaInit))
				chPublish <- [3]int{command.Id, int(xRobotBody), int(yRobotBody)}
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
				newX, newY := utilities.Rotate(float64(msg.X/10), float64(msg.Y/10), float64(state.getRobot(msg.Id).ThetaInit))
				index := state.id2index[msg.Id]
				state.multiRobot[index].X = int(newX) + state.getRobot(msg.Id).XInit
				state.multiRobot[index].Y = int(newY) + state.getRobot(msg.Id).YInit
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
		case msg := <-chReceiveMapFromRobot:
			//fmt.Print("\n", msg.X, msg.Y, msg.Height, msg.Width, msg.Obstacle, msg.Reachable)
			chB2gMapRectangle <- msg //Send the rectangle to mapping

		case init := <-chG2bRobotInit:
			id := init[0]
			state.id2index[id] = len(state.multiRobot)
			state.multiRobot = append(state.multiRobot, *initRobotState(init[1], init[2], init[3]))
			delete(pendingInit, id)
			chPublishInit<-init //Send init to communication
		}
	}
}

func (s *fullSlamState) setMapValue(x, y int, value uint8) {
	s.areaMap[x][y] = value
	switch value {
	case mapOpen:
		s.newOpen = append(s.newOpen, [2]int{x, y})
	case mapObstacle:
		s.newObstacle = append(s.newObstacle, [2]int{x, y})
	}
}

func (s *fullSlamState) getRobot(id int) types.RobotState {
	return s.multiRobot[s.id2index[id]]
}

func (s *fullSlamState) addIrSensorData(id, irX, irY int) {
	xMap, yMap := s.transformIrSensorData(id, irX, irY)
	s.addLineToMap(id, xMap, yMap)
}

func (s *fullSlamState) transformIrSensorData(id, xBodyFrame, yBodyFrame int) (int, int) {
	// IR data is given in mm and it is relative to the body, so it must be scaled, rotated and transelated to the map.

	//Must flip yBodyFrame, because the axis is flipped in the robot code.
	yBodyFrame = -yBodyFrame

	//rotate
	theta := s.multiRobot[s.id2index[id]].Theta
	xBodyFrameRotated, yBodyFrameRotated := utilities.Rotate(float64(xBodyFrame), float64(yBodyFrame), float64(theta))

	//scale and transelate
	xMap := math.Round(xBodyFrameRotated/10) + float64(s.multiRobot[s.id2index[id]].X)
	yMap := math.Round(yBodyFrameRotated/10) + float64(s.multiRobot[s.id2index[id]].Y)

	return int(xMap), int(yMap)
}

func (s *fullSlamState) addLineToMap(id, x1, y1 int) {
	//x0, y0, x1, y1 is given in map coordinates. With origo as defined in the config.
	x0 := s.getRobot(id).X
	y0 := s.getRobot(id).Y

	lineLength := math.Sqrt(math.Pow(float64(x0-x1), 2) + math.Pow(float64(y0-y1), 2))

	var obstruction bool
	if lineLength < config.IrSensorMaxDistance {
		obstruction = true
	} else {
		obstruction = false

		//shorten the line to config.IrSensorMaxDistance, needed for bresenham algorithm
		scale := config.IrSensorMaxDistance / lineLength
		x1 = x0 + int(scale*float64(x1-x0))
		y1 = y0 + int(scale*float64(y1-y0))

	}

	//get map index values
	x0Index, y0Index := calculateMapIndex(x0, y0)
	x1Index, y1Index := calculateMapIndex(x1, y1)
	//get values in map range
	x1Index = min(max(x1Index, 0), config.MapSize-1)
	y1Index = min(max(y1Index, 0), config.MapSize-1)
	x0Index = min(max(x0Index, 0), config.MapSize-1)
	y0Index = min(max(y0Index, 0), config.MapSize-1)

	indexPoints := utilities.BresenhamAlgorithm(x0Index, y0Index, x1Index, y1Index)
	for i := 0; i < len(indexPoints); i++ {
		x := indexPoints[i][0]
		y := indexPoints[i][1]
		s.setMapValue(x, y, mapOpen)
	}
	if obstruction {
		s.setMapValue(x1Index, y1Index, mapObstacle)
	}
}

func calculateMapIndex(x, y int) (int, int) {
	//Input is given in map coordinates (i.e. robot positions) with normal axis and origo as defined in the config.
	return config.MapCenterX + x, config.MapCenterY - y
}
