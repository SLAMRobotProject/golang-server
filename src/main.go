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
	chRobotCmd := make(chan types.RobotCommand, 3)
	chRobotTelemetry := make(chan types.RobotTelemetryMsg, 3)
	chCamera := make(chan types.CameraMsg, 16)

	//g2b = gui to backend
	chG2bRobotInit := make(chan [4]int, 3)
	chG2bCommand := make(chan types.Command)

	//b2g = backend to gui
	chB2gUpdate := make(chan types.UpdateGui, 3) // Buffered so it won't block RunBackend(...)
	chB2gRobotPendingInit := make(chan int, 3)   // Buffered so it won't block RunBackend()
	chVirtualTarget := make(chan types.VirtualTargetMsg, 8)
	chB2gVirtualPending := make(chan int, 3)
	chBackendState := make(chan *backend.BackendRuntimeState, 1)

	go backend.RunBackend(
		chRobotCmd,
		chRobotTelemetry,
		chCamera,
		chB2gRobotPendingInit,
		chB2gUpdate,
		chG2bRobotInit,
		chG2bCommand,
		chVirtualTarget,
		chBackendState,
	)

	backendState := <-chBackendState

	client := communication.InitMqtt()
	communication.Subscribe(client, chRobotTelemetry)
	communication.SubscribeCamera(client, chCamera)
	go communication.StartDigitalTwinTCPServer("localhost:9000", chCamera, chRobotTelemetry, chG2bRobotInit,
		chVirtualTarget, chB2gVirtualPending)
	go communication.ThreadMqttPublish(client, chRobotCmd)

	//go slam.ThreadSlam(chReceive, chCamera, nil)

	fmt.Printf("Starter GUI...\n")

	//window.ShowAndRun() must be run in the main thread. So the GUI must be initialized here.
	mainGUI := gui.NewMainGUI(chG2bCommand, backendState)

	go mainGUI.StartUpdateLoop(
		chG2bCommand,
		chG2bRobotInit,
		chB2gRobotPendingInit,
		chB2gUpdate,
		chB2gVirtualPending,
	)

	mainGUI.ShowAndRun()

}
