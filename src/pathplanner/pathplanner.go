package pathplanner

import (
	"golang-server/types"
	"math"
	"time"
)

// Path contains a sequence of waypoints.
type Path struct {
	Waypoints []Waypoint
	Loop      bool
}

func NewPath(waypoints ...Waypoint) Path {
	out := make([]Waypoint, len(waypoints))
	copy(out, waypoints)
	return Path{Waypoints: out}
}

func (p Path) WithLoop(loop bool) Path {
	p.Loop = loop
	return p
}

func (p Path) IsEmpty() bool {
	return len(p.Waypoints) == 0
}

type ManualPoseProvider func(id int) (types.RobotState, bool)
type AutoPoseProvider func() (types.RobotState, bool)
type LoopStateProvider func() bool
type CancelStateProvider func() bool

type PathPlanner struct {
	waypoints []Waypoint
}

func NewPathPlanner() *PathPlanner {
	return &PathPlanner{waypoints: make([]Waypoint, 0)}
}

// New is kept as a compatibility wrapper.
func New() *PathPlanner {
	return NewPathPlanner()
}

func (p *PathPlanner) Reset() {
	p.waypoints = p.waypoints[:0]
}

func (p *PathPlanner) AddWaypoint(wp Waypoint) {
	p.waypoints = append(p.waypoints, wp)
}

func (p *PathPlanner) RemoveWaypoint(index int) bool {
	if index < 0 || index >= len(p.waypoints) {
		return false
	}
	p.waypoints = append(p.waypoints[:index], p.waypoints[index+1:]...)
	return true
}

func (p *PathPlanner) SwapWaypoints(i, j int) bool {
	if i < 0 || j < 0 || i >= len(p.waypoints) || j >= len(p.waypoints) || i == j {
		return false
	}
	p.waypoints[i], p.waypoints[j] = p.waypoints[j], p.waypoints[i]
	return true
}

func (p *PathPlanner) Waypoints() []Waypoint {
	out := make([]Waypoint, len(p.waypoints))
	copy(out, p.waypoints)
	return out
}

func (p *PathPlanner) PlannedPath() Path {
	return NewPath(p.waypoints...)
}

func RunManualPath(
	chG2bCommand chan<- types.Command,
	id int,
	path Path,
	poseProvider ManualPoseProvider,
	loopProvider LoopStateProvider,
	cancelProvider CancelStateProvider,
	onWaypoint func(Waypoint),
) {
	if path.IsEmpty() {
		return
	}

	const (
		acceptableRadius = 8
		stallThreshold   = 3 * time.Second
		settleDuration   = time.Second
		checkInterval    = 200 * time.Millisecond
		graceDuration    = 4 * time.Second // give the robot time to rotate/start moving
		minProgressCm    = 1.0
		nominalAccel     = 25.0 // cm/s^2 used in s=0.5*a*t^2 expected-time estimate
		minExpectedSec   = 2.0
		maxExpectedSec   = 12.0
		extraBufferSec   = 1.0
	)

	send := func(wp Waypoint) {
		chG2bCommand <- types.Command{CommandType: types.ManualCommand, Id: id, X: wp.X, Y: wp.Y}
	}

	expectedDuration := func(distance float64) time.Duration {
		// Newton kinematics: s = 1/2 * a * t^2  =>  t = sqrt(2s/a)
		seconds := math.Sqrt((2.0*distance)/nominalAccel) + extraBufferSec
		if seconds < minExpectedSec {
			seconds = minExpectedSec
		}
		if seconds > maxExpectedSec {
			seconds = maxExpectedSec
		}
		return time.Duration(seconds * float64(time.Second))
	}

	waitForTarget := func(target Waypoint) bool {
		time.Sleep(graceDuration)

		var (
			lastPose   types.RobotState
			lastUpdate = time.Now()
			havePose   = false
			lastSendAt = time.Now()
			expectBy   = 5 * time.Second
			lastDist   = 0.0
		)

		for {
			if cancelProvider != nil && cancelProvider() {
				return false
			}

			time.Sleep(checkInterval)

			pose, ok := poseProvider(id)
			if !ok {
				continue
			}
			if !havePose {
				lastPose = pose
				lastUpdate = time.Now()
				dist := math.Hypot(float64(pose.X-target.X), float64(pose.Y-target.Y))
				expectBy = expectedDuration(dist)
				lastSendAt = time.Now()
				lastDist = dist
				havePose = true
			}

			dist := math.Hypot(float64(pose.X-target.X), float64(pose.Y-target.Y))

			if pose.X != lastPose.X || pose.Y != lastPose.Y {
				lastPose = pose
				lastUpdate = time.Now()
			}

			if lastDist-dist >= minProgressCm {
				lastUpdate = time.Now()
				lastDist = dist
			}

			if dist > lastDist {
				lastDist = dist
			}

			if dist <= acceptableRadius && time.Since(lastUpdate) >= settleDuration {
				return true
			}

			if dist > acceptableRadius && time.Since(lastUpdate) > stallThreshold {
				send(target)
				lastUpdate = time.Now()
				lastSendAt = time.Now()
				expectBy = expectedDuration(dist)
			}

			if dist > acceptableRadius && time.Since(lastSendAt) > expectBy {
				send(target)
				lastSendAt = time.Now()
				expectBy = expectedDuration(dist)
			}
		}
	}

	for {
		if cancelProvider != nil && cancelProvider() {
			return
		}
		for _, wp := range path.Waypoints {
			if cancelProvider != nil && cancelProvider() {
				return
			}
			if onWaypoint != nil {
				onWaypoint(wp)
			}
			send(wp)
			if !waitForTarget(wp) {
				return
			}
		}
		loopEnabled := path.Loop
		if loopProvider != nil {
			loopEnabled = loopProvider()
		}
		if !loopEnabled {
			return
		}
	}
}

func RunAutomaticPath(
	chG2bCommand chan<- types.Command,
	path Path,
	poseProvider AutoPoseProvider,
	loopProvider LoopStateProvider,
	cancelProvider CancelStateProvider,
	onWaypoint func(Waypoint),
) {
	if path.IsEmpty() {
		return
	}

	const (
		acceptableRadius = 8
		stallThreshold   = 3 * time.Second
		settleDuration   = time.Second
		checkInterval    = 200 * time.Millisecond
		graceDuration    = 4 * time.Second
		minProgressCm    = 1.0
		resendInterval   = 6 * time.Second
	)

	send := func(wp Waypoint) {
		chG2bCommand <- types.Command{CommandType: types.AutomaticCommand, Id: -1, X: wp.X, Y: wp.Y}
	}

	waitForTarget := func(target Waypoint) bool {
		time.Sleep(graceDuration)
		var (
			lastPose   types.RobotState
			lastUpdate = time.Now()
			havePose   = false
			lastSendAt = time.Now()
			lastDist   = 0.0
		)

		for {
			if cancelProvider != nil && cancelProvider() {
				return false
			}

			time.Sleep(checkInterval)
			pose, ok := poseProvider()
			if !ok {
				continue
			}
			if !havePose {
				lastPose = pose
				lastUpdate = time.Now()
				lastDist = math.Hypot(float64(pose.X-target.X), float64(pose.Y-target.Y))
				lastSendAt = time.Now()
				havePose = true
			}

			dist := math.Hypot(float64(pose.X-target.X), float64(pose.Y-target.Y))
			if pose.X != lastPose.X || pose.Y != lastPose.Y {
				lastPose = pose
				lastUpdate = time.Now()
			}

			if lastDist-dist >= minProgressCm {
				lastUpdate = time.Now()
				lastDist = dist
			}

			if dist > lastDist {
				lastDist = dist
			}

			if dist <= acceptableRadius && time.Since(lastUpdate) >= settleDuration {
				return true
			}

			if dist > acceptableRadius && time.Since(lastUpdate) > stallThreshold {
				send(target)
				lastUpdate = time.Now()
				lastSendAt = time.Now()
			}

			if dist > acceptableRadius && time.Since(lastSendAt) > resendInterval {
				send(target)
				lastSendAt = time.Now()
			}
		}
	}

	for {
		if cancelProvider != nil && cancelProvider() {
			return
		}
		for _, wp := range path.Waypoints {
			if cancelProvider != nil && cancelProvider() {
				return
			}
			if onWaypoint != nil {
				onWaypoint(wp)
			}
			send(wp)
			if !waitForTarget(wp) {
				return
			}
		}
		loopEnabled := path.Loop
		if loopProvider != nil {
			loopEnabled = loopProvider()
		}
		if !loopEnabled {
			return
		}
	}
}
