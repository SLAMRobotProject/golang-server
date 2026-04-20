package backend

import (
	"golang-server/backend/pose"
	"golang-server/backend/robot"
	"golang-server/config"
	"golang-server/log"
	"golang-server/slam"
	"golang-server/types"
	util "golang-server/utilities"
	stdlog "log"
	"sync"
	"time"
)

// BackendEngine is the main struct for the backend, containing channels and state.
type BackendEngine struct {
	runtimeState *BackendRuntimeState
	runtimeMap   *MapRuntimeState

	chRobotCmd            chan<- types.RobotCommand
	chRobotTelemetry      <-chan types.RobotTelemetryMsg
	chCamera              <-chan types.CameraMsg
	chB2gRobotPendingInit chan<- int
	chB2gUpdate           chan<- types.UpdateGui
	chG2bRobotInit        <-chan [4]int
	chG2bCommand          <-chan types.Command
	chVirtualTarget       chan<- types.VirtualTargetMsg

	prevMsg         types.RobotTelemetryMsg
	pendingInit     map[int]struct{}
	positionLogger  *stdlog.Logger
	guiUpdateTicker *time.Ticker
}

// BackendRuntimeState stores mutable backend data shared by handlers.
type BackendRuntimeState struct {
	mu                 sync.RWMutex
	runtimeMultiRobots []*robot.Robot
	runtimeMap         MapRuntimeState
}

func initBackendRuntimeState() *BackendRuntimeState {
	return &BackendRuntimeState{
		runtimeMultiRobots: make([]*robot.Robot, 0),
		runtimeMap:         initMapRuntimeState(),
	}
}

// Main loop of the backend engine, handling all incoming messages and commands.
func (e *BackendEngine) Start() {
	for {
		select {
		case <-e.guiUpdateTicker.C:
			e.handleGuiTick()

		case command := <-e.chG2bCommand:
			if !e.handleCommand(command) {
				return
			}
		case msg := <-e.chRobotTelemetry:
			e.handleAdvMsg(msg)
		case cam := <-e.chCamera:
			e.handleCameraMsg(cam)
		case init := <-e.chG2bRobotInit:
			e.handleRobotInit(init)
		}
	}
}

func RunBackend(
	chRobotCmd chan<- types.RobotCommand,
	chRobotTelemetry <-chan types.RobotTelemetryMsg,
	chCamera <-chan types.CameraMsg,
	chB2gRobotPendingInit chan<- int,
	chB2gUpdate chan<- types.UpdateGui,
	chG2bRobotInit <-chan [4]int,
	chG2bCommand <-chan types.Command,
	chVirtualTarget chan<- types.VirtualTargetMsg,
	chBackendState chan<- *BackendRuntimeState,
) {
	runtimeState := initBackendRuntimeState()

	engine := &BackendEngine{
		runtimeState:          runtimeState,
		runtimeMap:            &runtimeState.runtimeMap,
		chRobotCmd:            chRobotCmd,
		chRobotTelemetry:      chRobotTelemetry,
		chCamera:              chCamera,
		chB2gRobotPendingInit: chB2gRobotPendingInit,
		chB2gUpdate:           chB2gUpdate,
		chG2bRobotInit:        chG2bRobotInit,
		chG2bCommand:          chG2bCommand,
		chVirtualTarget:       chVirtualTarget,
		pendingInit:           map[int]struct{}{},
		positionLogger:        log.InitPositionLogger(),
		guiUpdateTicker:       time.NewTicker(time.Second / config.GuiFrameRate),
	}

	chBackendState <- engine.runtimeState
	engine.Start()
}

// ================================================
// Handlers -
// ================================================

func (e *BackendEngine) handleGuiTick() {
	multiRobot, id2index := e.runtimeState.snapshotRobotStates()
	submapCounts := make(map[int]int, len(e.runtimeMap.slamMappers))
	for id, mapper := range e.runtimeMap.slamMappers {
		submapCounts[id] = mapper.SubmapCount()
	}
	e.chB2gUpdate <- types.UpdateGui{
		MultiRobot:        multiRobot,
		Id2index:          id2index,
		NewOpen:           e.runtimeMap.newOpen,
		NewObstacle:       e.runtimeMap.newObstacle,
		Lines:             e.runtimeMap.visualLines,
		SlamMapImgs:       e.runtimeMap.slamMapImgs,
		SlamGlobalImgs:    e.runtimeMap.slamGlobalImgs,
		SlamGlobalDbgImgs: e.runtimeMap.slamGlobalDbgImgs,
		SlamSubmapCount:   submapCounts,
	}
	e.runtimeMap.newOpen = [][2]int{}
	e.runtimeMap.newObstacle = [][2]int{}
	e.runtimeMap.visualLines = [][2]types.Point{}
}

func (e *BackendEngine) handleCommand(command types.Command) bool {
	id := command.Id
	commandName := "manual"
	if command.CommandType == types.AutomaticCommand {
		id = e.runtimeState.findClosestRobot(command.X, command.Y)
		if id == -1 {
			return false
		}
		commandName = "automatic"
	}
	r := e.runtimeState.getRobotObject(id)
	if r == nil {
		return true
	}
	target := e.runtimeMap.grid.CoordinatesToPose(command.X, command.Y)
	e.chRobotCmd <- r.NewRobotCommand(id, target)
	odomTarget := r.MapTargetToOdomTarget(target)
	select {
	case e.chVirtualTarget <- types.VirtualTargetMsg{Id: id, X: odomTarget.X, Y: odomTarget.Y}:
	default:
	}
	log.GGeneralLogger.Println("Publishing ", commandName, " input to robot with ID: ", id, " x: ", command.X, " y: ", command.Y, ".")
	return true
}

func (e *BackendEngine) handleAdvMsg(msg types.RobotTelemetryMsg) {
	if _, exist := e.pendingInit[msg.Id]; exist {
		// skip
	} else if _, exist := e.runtimeState.indexByID(msg.Id); !exist {
		e.pendingInit[msg.Id] = struct{}{}
		e.chB2gRobotPendingInit <- msg.Id
	} else {
		e.runtimeState.updateRobotOdom(msg.Id, pose.FromRobotTelemetry(msg))

		e.runtimeState.addIrSensorData(msg.Id, msg.Ir1x, msg.Ir1y)
		e.runtimeState.addIrSensorData(msg.Id, msg.Ir2x, msg.Ir2y)
		e.runtimeState.addIrSensorData(msg.Id, msg.Ir3x, msg.Ir3y)
		e.runtimeState.addIrSensorData(msg.Id, msg.Ir4x, msg.Ir4y)

		r := e.runtimeState.getRobotObject(msg.Id)
		if r != nil && (msg.X != e.prevMsg.X || msg.Y != e.prevMsg.Y || msg.Theta != e.prevMsg.Theta) {
			current := r.ToState()
			e.positionLogger.Printf("%d %d %d %d\n", msg.Id, current.X, current.Y, current.Theta)
		}
	}
	e.prevMsg = msg
}

func (e *BackendEngine) handleCameraMsg(cam types.CameraMsg) {
	if _, exist := e.runtimeState.indexByID(cam.Id); !exist {
		return
	}
	mapper := e.runtimeMap.slamMappers[cam.Id]
	if mapper == nil {
		e.runtimeMap.visualLines = [][2]types.Point{}
		return
	}

	r := e.runtimeState.getRobotObject(cam.Id)
	if r == nil {
		return
	}
	gX := r.Current.X
	gY := r.Current.Y
	gTheta := r.Current.Theta

	processSlamUpdate := func(sensorObservation types.CameraObject) {
		img, correction := mapper.ProcessUpdate(gX, gY, gTheta, sensorObservation)
		e.runtimeMap.slamMapImgs[cam.Id] = img
		e.runtimeMap.slamGlobalImgs[cam.Id] = mapper.RenderGlobal()
		e.runtimeMap.slamGlobalDbgImgs[cam.Id] = mapper.RenderGlobalDebug()
		if correction != nil {
			e.runtimeState.applyRobotMapCorrection(cam.Id, pose.Pose{X: correction.X, Y: correction.Y, Theta: correction.Theta})
		}
	}

	if !cam.IsViritual {
		e.runtimeState.addCameraSegment(cam.Id, cam.Obj.StartMM, cam.Obj.WidthMM, cam.Obj.DistMM)
	}

	var sensorObservation types.CameraObject
	if cam.IsViritual {
		localX, localY := r.VirtualCameraWorldPointToLocal(cam.Obj.StartMM, cam.Obj.DistMM)
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
	e.runtimeMap.visualLines = [][2]types.Point{}
}

func (e *BackendEngine) handleRobotInit(init [4]int) {
	id := init[0]
	initialPose := pose.Pose{X: util.CmToMetres(init[1]), Y: util.CmToMetres(init[2]), Theta: util.DegreesToRadians(init[3])}
	e.runtimeState.addRuntimeRobot(robot.NewRobot(id, initialPose))
	e.runtimeMap.slamMappers[id] = slam.NewOccupancyMap()
	delete(e.pendingInit, id)
}
