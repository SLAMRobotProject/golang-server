package backend

import (
	"golang-server/backend/pose"
	"golang-server/backend/robot"
	"golang-server/log"
	"golang-server/types"
	"math"
)

// GetRobotPose returns the robot state for the given ID (thread-safe).
func (s *BackendRuntimeState) GetRobotPose(id int) (types.RobotState, bool) {
	s.mu.RLock()
	defer s.mu.RUnlock()

	idx, ok := s.indexByID(id)
	if !ok {
		return types.RobotState{}, false
	}
	robot := s.runtimeMultiRobots[idx]
	if robot == nil {
		return types.RobotState{}, false
	}
	return robot.ToState(), true
}

// GetAnyRobotPose returns the first available robot state (thread-safe).
func (s *BackendRuntimeState) GetAnyRobotPose() (types.RobotState, bool) {
	s.mu.RLock()
	defer s.mu.RUnlock()

	for _, robot := range s.runtimeMultiRobots {
		if robot != nil {
			return robot.ToState(), true
		}
	}
	return types.RobotState{}, false
}

func (s *BackendRuntimeState) snapshotRobotStates() ([]types.RobotState, map[int]int) {
	states := make([]types.RobotState, 0, len(s.runtimeMultiRobots))
	id2index := make(map[int]int, len(s.runtimeMultiRobots))
	for _, robot := range s.runtimeMultiRobots {
		if robot == nil {
			continue
		}
		id2index[robot.ID] = len(states)
		states = append(states, robot.ToState())
	}
	return states, id2index
}

func (s *BackendRuntimeState) indexByID(id int) (int, bool) {
	for idx, robot := range s.runtimeMultiRobots {
		if robot != nil && robot.ID == id {
			return idx, true
		}
	}
	return -1, false
}

func (s *BackendRuntimeState) addRuntimeRobot(robot *robot.Robot) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.runtimeMultiRobots = append(s.runtimeMultiRobots, robot)
}

func (s *BackendRuntimeState) getRobotObject(id int) *robot.Robot {
	idx, ok := s.indexByID(id)
	if !ok {
		return nil
	}
	return s.runtimeMultiRobots[idx]
}

func (s *BackendRuntimeState) getCurrentPose(id int) (pose.Pose, bool) {
	s.mu.RLock()
	defer s.mu.RUnlock()

	robotState := s.getRobotObject(id)
	if robotState == nil {
		return pose.Pose{}, false
	}
	return robotState.Current, true
}

func (s *BackendRuntimeState) applyRobotMapCorrection(id int, mapPose pose.Pose) bool {
	s.mu.Lock()
	defer s.mu.Unlock()
	
	robotState := s.getRobotObject(id)
	if robotState == nil {
		return false
	}
	robotState.ApplyMapCorrection(mapPose)
	return true
}

func (s *BackendRuntimeState) findClosestRobot(x, y int) int {
	minDistance := math.MaxFloat64
	closestRobot := -1
	for _, robot := range s.runtimeMultiRobots {
		if robot == nil {
			continue
		}
		state := robot.ToState()
		distance := math.Sqrt(math.Pow(float64(state.X-x), 2) + math.Pow(float64(state.Y-y), 2))
		if distance < minDistance {
			minDistance = distance
			closestRobot = robot.ID
		}
	}
	if closestRobot == -1 {
		log.GGeneralLogger.Println("Tried to find closest robot, but no robots were found.")
	}
	return closestRobot
}

// updateRobotOdom safely updates robot odometry under write lock.
func (s *BackendRuntimeState) updateRobotOdom(id int, odomPose pose.Pose) bool {
	s.mu.Lock()
	defer s.mu.Unlock()

	robot := s.getRobotObject(id)
	if robot == nil {
		return false
	}
	robot.UpdateOdom(odomPose)
	return true
}
