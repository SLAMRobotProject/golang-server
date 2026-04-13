package main

import (
	"fmt"
	"golang-server/backend"
	"golang-server/communication"
	"golang-server/gui"
	"golang-server/types"
)

func main() {
	//Most channels are buffered for efficiency.

	//only backend can publish and receive
	chPublish := make(chan [3]int, 3)
	chReceive := make(chan types.AdvMsg, 3)
	chCamera := make(chan types.CameraMsg, 16)

	//g2b = gui to backend
	chG2bRobotInit := make(chan [4]int, 3)
	chG2bCommand := make(chan types.Command)

	//b2g = backend to gui
	chB2gUpdate := make(chan types.UpdateGui, 3) //Buffered so it won't block ThreadBackend(types.AdvMsg
	chB2gRobotPendingInit := make(chan int, 3)   //Buffered so it won't block ThreadBackend()
	chPoseUpdate := make(chan types.PoseUpdateMsg, 3)

	go backend.ThreadBackend(
		chPublish,
		chReceive,
		chCamera,
		chPoseUpdate,
		chB2gRobotPendingInit,
		chB2gUpdate,
		chG2bRobotInit,
		chG2bCommand,
	)

	client := communication.InitMqtt()
	communication.Subscribe(client, chReceive)
	communication.SubscribeCamera(client, chCamera)
	go communication.StartDigitalTwinTCPServer("localhost:9000", chCamera, chReceive, chG2bRobotInit, chPoseUpdate)
	go communication.ThreadMqttPublish(client, chPublish)

	//go slam.ThreadSlam(chReceive, chCamera, nil)

	fmt.Printf("Starter GUI...\n")

	//window.ShowAndRun() must be run in the main thread. So the GUI must be initialized here.
	window, mapImage, mapCanvas, slamLCanvas, slamGCanvas, allRobotsHandle, manualInput, initInput := gui.InitGui(chG2bCommand)
	go gui.ThreadGuiUpdate(
		mapImage,
		mapCanvas,
		slamLCanvas,
		slamGCanvas,
		allRobotsHandle,
		manualInput, initInput,
		chG2bCommand,
		chG2bRobotInit,
		chB2gRobotPendingInit,
		chB2gUpdate,
	)

	window.ShowAndRun()

}
