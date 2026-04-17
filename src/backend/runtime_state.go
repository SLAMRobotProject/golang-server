package backend

import (
	"golang-server/config"
	"golang-server/slam"
	"golang-server/types"
	"image"
)

// BackendRuntimeState holds the mutable runtime data used by the backend loop.
type BackendRuntimeState struct {
	areaMap           [config.MapSize][config.MapSize]uint8
	newObstacle       [][2]int // new since last gui update
	newOpen           [][2]int // new since last gui update
	grid              GridMap
	multiRobot        []types.RobotState
	robots            map[int]*Robot
	id2index          map[int]int
	visualLines       [][2]types.Point
	slamMappers       map[int]*slam.OccupancyMap // one OccupancyMap per robot ID
	slamMapImgs       map[int]image.Image        // latest local map image per robot ID
	slamGlobalImgs    map[int]image.Image        // latest global map image per robot ID
	slamGlobalDbgImgs map[int]image.Image        // latest global map image with debug overlays
}

func initBackendRuntimeState() *BackendRuntimeState {
	s := BackendRuntimeState{}
	for i := 0; i < config.MapSize; i++ {
		for j := 0; j < config.MapSize; j++ {
			s.areaMap[i][j] = mapUnknown
		}
	}
	s.id2index = make(map[int]int)
	s.grid = NewGridMap()
	s.robots = make(map[int]*Robot)
	s.slamMappers = make(map[int]*slam.OccupancyMap)
	s.slamMapImgs = make(map[int]image.Image)
	s.slamGlobalImgs = make(map[int]image.Image)
	s.slamGlobalDbgImgs = make(map[int]image.Image)

	return &s
}

// Compatibility shim for existing tests and call sites.
func initFullSlamState() *BackendRuntimeState {
	return initBackendRuntimeState()
}

func (s *BackendRuntimeState) getRobotObject(id int) *Robot {
	if robot, ok := s.robots[id]; ok {
		return robot
	}
	return nil
}

func (s *BackendRuntimeState) getCurrentPose(id int) (Pose, bool) {
	if robot := s.getRobotObject(id); robot != nil {
		return robot.Current, true
	}
	idx, ok := s.id2index[id]
	if !ok || idx < 0 || idx >= len(s.multiRobot) {
		return Pose{}, false
	}
	return poseFromRobotState(s.multiRobot[idx]), true
}

func (s *BackendRuntimeState) applyRobotMapCorrection(id int, mapPose Pose) (types.PoseUpdateMsg, bool) {
	robot := s.getRobotObject(id)
	if robot == nil {
		return types.PoseUpdateMsg{}, false
	}
	robot.ApplyMapCorrection(mapPose)
	if index, ok := s.id2index[id]; ok {
		s.multiRobot[index] = robot.ToState()
	}
	return types.PoseUpdateMsg{Id: id, X: mapPose.X, Y: mapPose.Y, Theta: mapPose.Theta}, true
}
