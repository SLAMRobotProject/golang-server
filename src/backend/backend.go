package backend

import (
	"fmt"
	"golang-server/config"
	"golang-server/log"
	"golang-server/slam"
	"golang-server/types"
	util "golang-server/utilities"
	"strings"
	"time"
)

// using binary flags to represent the map to allow bitwise operations
const (
	mapOpen     uint8 = 1 << iota //1
	mapUnknown                    //2
	mapObstacle                   //4
)

func initRobotState(pose Pose) *types.RobotState {
	x := util.MetresToCm(pose.X)
	y := util.MetresToCm(pose.Y)
	theta := util.RadiansToDegrees(pose.Theta)
	return &types.RobotState{X: x, Y: y, Theta: theta, XInit: x, YInit: y, ThetaInit: theta}
}

// The map is very large and sending it gives a warning. This only sends updates.

func ThreadBackend(
	chRobotCmd chan<- types.RobotCommand,
	chReceive <-chan types.AdvMsg,
	chCamera <-chan types.CameraMsg,
	chPoseUpdate chan<- types.PoseUpdateMsg,
	chB2gRobotPendingInit chan<- int,
	chB2gUpdate chan<- types.UpdateGui,
	chG2bRobotInit <-chan [4]int,
	chG2bCommand <-chan types.Command,
) {
	var state *BackendRuntimeState = initBackendRuntimeState()

	prevMsg := types.AdvMsg{}
	positionLogger := log.InitPositionLogger()
	pendingInit := map[int]struct{}{}
	guiUpdateTicker := time.NewTicker(time.Second / config.GuiFrameRate)
	handleGuiTick := func() {
		submapCounts := make(map[int]int, len(state.slamMappers))
		for id, mapper := range state.slamMappers {
			submapCounts[id] = mapper.SubmapCount()
		}
		chB2gUpdate <- types.UpdateGui{
			MultiRobot:        state.multiRobot,
			Id2index:          state.id2index,
			NewOpen:           state.newOpen,
			NewObstacle:       state.newObstacle,
			Lines:             state.visualLines,
			SlamMapImgs:       state.slamMapImgs,
			SlamGlobalImgs:    state.slamGlobalImgs,
			SlamGlobalDbgImgs: state.slamGlobalDbgImgs,
			SlamSubmapCount:   submapCounts,
		}
		state.newOpen = [][2]int{}
		state.newObstacle = [][2]int{}
		state.visualLines = [][2]types.Point{}
	}
	handleCommand := func(command types.Command) bool {
		id := command.Id
		commandName := "manual"
		if command.CommandType == types.AutomaticCommand {
			id = state.findClosestRobot(command.X, command.Y)
			if id == -1 {
				// already logged in findClosestRobot()
				return false
			}
			commandName = "automatic"
		}
		robot := state.getRobotObject(id)
		if robot == nil {
			return true
		}
		target := state.grid.CoordinatesToPose(command.X, command.Y)
		chRobotCmd <- robot.NewRobotCommand(id, target)
		log.GGeneralLogger.Println("Publishing ", commandName, " input to robot with ID: ", id, " x: ", command.X, " y: ", command.Y, ".")
		return true
	}
	handleAdvMsg := func(msg types.AdvMsg) {
		if _, exist := pendingInit[msg.Id]; exist {
			// skip
		} else if _, exist := state.id2index[msg.Id]; !exist {
			pendingInit[msg.Id] = struct{}{}
			chB2gRobotPendingInit <- msg.Id // Buffered channel, so it will not block.
		} else {
			index := state.id2index[msg.Id]
			robot := state.getRobotObject(msg.Id)
			if robot == nil {
				return
			}
			robot.UpdateOdom(poseFromAdvMsg(msg))
			state.multiRobot[index] = robot.ToState()

			state.addIrSensorData(msg.Id, msg.Ir1x, msg.Ir1y)
			state.addIrSensorData(msg.Id, msg.Ir2x, msg.Ir2y)
			state.addIrSensorData(msg.Id, msg.Ir3x, msg.Ir3y)
			state.addIrSensorData(msg.Id, msg.Ir4x, msg.Ir4y)

			if msg.X != prevMsg.X || msg.Y != prevMsg.Y || msg.Theta != prevMsg.Theta {
				current := robot.ToState()
				positionLogger.Printf("%d %d %d %d\n", msg.Id, current.X, current.Y, current.Theta)
			}
		}
		prevMsg = msg
	}
	handleCameraMsg := func(cam types.CameraMsg) {
		if _, exist := state.id2index[cam.Id]; !exist {
			return
		}
		mapper := state.slamMappers[cam.Id]
		if mapper == nil {
			state.visualLines = [][2]types.Point{}
			return
		}

		robot := state.getRobotObject(cam.Id)
		if robot == nil {
			return
		}
		gX := robot.Current.X
		gY := robot.Current.Y
		gTheta := robot.Current.Theta

		processSlamUpdate := func(sensorObservation types.CameraObject) {
			img, correction := mapper.ProcessUpdate(gX, gY, gTheta, sensorObservation)
			state.slamMapImgs[cam.Id] = img
			state.slamGlobalImgs[cam.Id] = mapper.RenderGlobal()
			state.slamGlobalDbgImgs[cam.Id] = mapper.RenderGlobalDebug()
			if correction != nil {
				if _, ok := state.applyRobotMapCorrection(cam.Id, Pose{X: correction.X, Y: correction.Y, Theta: correction.Theta}); ok {
					//chPoseUpdate <- poseUpdate
				}
			}
		}

		if !cam.IsViritual {
			state.addCameraSegment(cam.Id, cam.Obj.StartMM, cam.Obj.WidthMM, cam.Obj.DistMM)
		}

		var sensorObservation types.CameraObject
		if cam.IsViritual {
			localX, localY := robot.VirtualCameraWorldPointToLocal(cam.Obj.StartMM, cam.Obj.DistMM)
			if localY > slam.MinBuildDist && localY < slam.MaxBuildDist {
				halfWidthMM := util.MetresToMm(slam.GridRes)
				sensorObservation = types.CameraObject{
					DistMM:  util.MetresToMm(localY),
					StartMM: util.MetresToMm(localX) - halfWidthMM,
					WidthMM: 2 * halfWidthMM,
				}
			}
		} else {
			dist := cam.Obj.DistMM
			if dist < config.CameraNoHitRawMM {
				sensorObservation = types.CameraObject{
					DistMM:  dist + config.CameraMountOffsetMM,
					StartMM: cam.Obj.StartMM,
					WidthMM: cam.Obj.WidthMM,
				}
			}
		}
		processSlamUpdate(sensorObservation)
		state.visualLines = [][2]types.Point{}
	}
	handleRobotInit := func(init [4]int) {
		id := init[0]
		state.id2index[id] = len(state.multiRobot)
		initialPose := Pose{X: util.CmToMetres(init[1]), Y: util.CmToMetres(init[2]), Theta: util.DegreesToRadians(init[3])}
		state.robots[id] = NewRobot(initialPose)
		state.multiRobot = append(state.multiRobot, state.robots[id].ToState())
		state.slamMappers[id] = slam.NewOccupancyMap()
		delete(pendingInit, id)
	}

	for {
		select {
		case <-guiUpdateTicker.C:
			handleGuiTick()

		case command := <-chG2bCommand:
			if !handleCommand(command) {
				return
			}
		case msg := <-chReceive:
			handleAdvMsg(msg)
		case cam := <-chCamera:
			handleCameraMsg(cam)
		case init := <-chG2bRobotInit:
			handleRobotInit(init)
		}
	}
}

func formatCovarianceMatrix(matrix [5 * 5]float32) string {
	var sb strings.Builder
	for i, value := range matrix {
		if i > 0 {
			sb.WriteString(",")
		}
		sb.WriteString(fmt.Sprintf("%f", value))
	}
	return sb.String()
}
