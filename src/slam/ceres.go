//go:build ceres

package slam

// #cgo CXXFLAGS: -std=c++17 -DHAVE_CERES -Dj0=_j0 -Dj1=_j1 -Djn=_jn -DGLOG_USE_GLOG_EXPORT -DGLOG_NO_ABBREVIATED_SEVERITIES -IC:/msys64/mingw64/include/eigen3 -IC:/msys64/mingw64/include
// #cgo LDFLAGS: -LC:/msys64/mingw64/lib -lceres -lglog -lgflags
// #include "ceres_wrapper.h"
import "C"
import (
	"fmt"
	util "golang-server/utilities"
	"unsafe"
)

// optimizePoseGraph calls the Ceres Solver to find globally consistent submap
// origins that satisfy all recorded pose-graph constraints (sequential edges
// weight 1.0, loop-closure edges weight 3.0, Huber loss 0.10 m).
// Submap 0 is held fixed as the world anchor.
func (m *OccupancyMap) optimizePoseGraph() {
	n := len(m.submaps)
	if n < 2 || len(m.edges) == 0 {
		return
	}

	// Marshal submap origins into a flat C array.
	cposes := make([]C.CeresPose, n)
	for i, s := range m.submaps {
		cposes[i].x = C.double(s.Origin.X)
		cposes[i].y = C.double(s.Origin.Y)
		cposes[i].theta = C.double(s.Origin.Theta)
	}

	// Marshal edges.
	cedges := make([]C.CeresEdge, len(m.edges))
	for i, e := range m.edges {
		cedges[i].from = C.int(e.from)
		cedges[i].to = C.int(e.to)
		cedges[i].dx = C.double(e.dx)
		cedges[i].dy = C.double(e.dy)
		cedges[i].dtheta = C.double(e.dtheta)
		cedges[i].weightXY = C.double(e.weightXY)
		cedges[i].weightTheta = C.double(e.weightTheta)

	}

	fmt.Printf("[SLAM] Ceres optimizer: %d submaps, %d edges\n", n, len(m.edges))

	C.ceres_optimize_poses(
		(*C.CeresPose)(unsafe.Pointer(&cposes[0])),
		C.int(n),
		(*C.CeresEdge)(unsafe.Pointer(&cedges[0])),
		C.int(len(cedges)),
	)

	// Unmarshal optimised poses back into submap origins.
	for i, s := range m.submaps {
		s.Origin.X = float64(cposes[i].x)
		s.Origin.Y = float64(cposes[i].y)
		s.Origin.Theta = util.NormalizeAngle(float64(cposes[i].theta))
	}
}
