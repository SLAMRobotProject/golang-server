package robot

import (
	"golang-server/backend/pose"
	"golang-server/types"
	util "golang-server/utilities"
	"math"
)

type Robot struct {
	ID        int
	Current   pose.Pose
	Initial   pose.Pose
	Odom      pose.Pose
	OdomToMap pose.Pose
}

func NewRobot(id int, initial pose.Pose) *Robot {
	return &Robot{
		ID:        id,
		Current:   initial,
		Initial:   initial,
		Odom:      pose.Pose{},
		OdomToMap: initial,
	}
}

func (r *Robot) UpdateOdom(odom pose.Pose) {
	r.Odom = odom
	r.Current = r.OdomToMap.Compose(r.Odom)
}

func (r *Robot) ApplyMapCorrection(mapPose pose.Pose) {
	r.OdomToMap = mapPose.Compose(r.Odom.Inverse())
	r.Current = r.OdomToMap.Compose(r.Odom)
}

func (r Robot) MapPoseToBodyPose(target pose.Pose) pose.Pose {
	bodyX, bodyY := r.Current.GlobalToLocal(target.X, target.Y)
	return pose.Pose{X: bodyX, Y: bodyY, Theta: util.NormalizeAngle(target.Theta - r.Current.Theta)}
}

func (r Robot) VirtualCameraWorldPointToLocal(worldXMM, worldYMM int) (float64, float64) {
	rotXCM, rotYCM := util.Rotate(float64(worldXMM)/10.0, float64(worldYMM)/10.0, r.Initial.Theta)
	globalX := util.CmToMetres(int(math.Round(rotXCM))) + r.Initial.X
	globalY := util.CmToMetres(int(math.Round(rotYCM))) + r.Initial.Y
	return r.Current.GlobalToLocal(globalX, globalY)
}

func (r Robot) MapTargetToOdomTarget(mapTarget pose.Pose) pose.Pose {
	return r.OdomToMap.Inverse().Compose(mapTarget)
}

func (r Robot) NewRobotCommand(robotID int, target pose.Pose) types.RobotCommand {
	bodyTarget := r.MapPoseToBodyPose(target)
	return types.RobotCommand{
		RobotID: robotID,
		TargetPose: types.Pose2D{
			X:     bodyTarget.X,
			Y:     bodyTarget.Y,
			Theta: bodyTarget.Theta,
		},
	}
}

func (r Robot) ToState() types.RobotState {
	return types.RobotState{
		X:         util.MetresToCm(r.Current.X),
		Y:         util.MetresToCm(r.Current.Y),
		Theta:     util.RadiansToDegrees(r.Current.Theta),
		XInit:     util.MetresToCm(r.Initial.X),
		YInit:     util.MetresToCm(r.Initial.Y),
		ThetaInit: util.RadiansToDegrees(r.Initial.Theta),
	}
}
