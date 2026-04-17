package backend

import (
	"golang-server/types"
	util "golang-server/utilities"
	"math"
)

type Pose struct {
	X, Y, Theta float64
}

func (p Pose) Compose(other Pose) Pose {
	ca := math.Cos(p.Theta)
	sa := math.Sin(p.Theta)
	return Pose{
		X:     p.X + ca*other.X - sa*other.Y,
		Y:     p.Y + sa*other.X + ca*other.Y,
		Theta: util.NormalizeAngle(p.Theta + other.Theta),
	}
}

func (p Pose) Inverse() Pose {
	c := math.Cos(p.Theta)
	s := math.Sin(p.Theta)
	return Pose{
		X:     -(c*p.X + s*p.Y),
		Y:     s*p.X - c*p.Y,
		Theta: util.NormalizeAngle(-p.Theta),
	}
}

func (p Pose) LocalToGlobal(localX, localY float64) (float64, float64) {
	ca := math.Cos(p.Theta)
	sa := math.Sin(p.Theta)
	return p.X + ca*localX - sa*localY, p.Y + sa*localX + ca*localY
}

func (p Pose) GlobalToLocal(globalX, globalY float64) (float64, float64) {
	dx := globalX - p.X
	dy := globalY - p.Y
	ca := math.Cos(-p.Theta)
	sa := math.Sin(-p.Theta)
	return dx*ca - dy*sa, dx*sa + dy*ca
}

func (p Pose) ToRobotState(initX, initY, initTheta int) types.RobotState {
	return types.RobotState{
		X:         util.MetresToCm(p.X),
		Y:         util.MetresToCm(p.Y),
		Theta:     util.RadiansToDegrees(p.Theta),
		XInit:     initX,
		YInit:     initY,
		ThetaInit: initTheta,
	}
}

func poseFromRobotState(r types.RobotState) Pose {
	return Pose{
		X:     util.CmToMetres(r.X),
		Y:     util.CmToMetres(r.Y),
		Theta: util.DegreesToRadians(r.Theta),
	}
}

func poseFromAdvMsg(msg types.AdvMsg) Pose {
	return Pose{
		X:     util.MmToMetres(msg.X),
		Y:     util.MmToMetres(msg.Y),
		Theta: util.DegreesToRadians(msg.Theta),
	}
}
