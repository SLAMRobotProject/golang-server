package main

import (
	"context"
	"fmt"
	"golang-server/backend"
	"golang-server/communication"
	"golang-server/gui"
	"golang-server/types"
	"os"
	"os/signal"
	"syscall"
)

func main() {
	//Most channels are buffered for efficiency.
	print("hellpo")
	//only backend can publish and receive
	chPublish := make(chan [3]int, 3)
	chPublishInit := make(chan [4]int, 3)
	chReceive := make(chan types.AdvMsg, 3)
	chReceiveMapFromRobot := make(chan types.RectangleMsg, 3)
	chPublishHome := make(chan types.HomePathMsg, 1)

	//chSubRecieve2 :=make(chan )
	//g2b = gui to backend
	chG2bRobotInit := make(chan [4]int, 3)
	chG2bCommand := make(chan types.Command)

	//b2g = backend to gui
	chB2gUpdate := make(chan types.UpdateGui, 3) //Buffered so it won't block ThreadBackend(types.AdvMsg
	chB2gRobotPendingInit := make(chan int, 3)   //Buffered so it won't block ThreadBackend()
	chB2gMapRectangle := make(chan types.RectangleMsg, 3)

	state := backend.InitFullSlamState()
	go backend.ThreadBackend( //add chPublishInit
		state,
		chPublish,
		chPublishInit,
		chReceive,
		chB2gRobotPendingInit,
		chB2gUpdate,
		chG2bRobotInit,
		chG2bCommand,
		chReceiveMapFromRobot,
		chB2gMapRectangle,
		chPublishHome,
	)

	client := communication.InitMqtt()
	communication.Subscribe(client, chReceive, chReceiveMapFromRobot)
	go communication.ThreadMqttPublish(client, chPublish, chPublishInit, chPublishHome)

	//window.ShowAndRun() must be run in the main thread. So the GUI must be initialized here.
	app, serverWindow, mapWindow, fasitWindow, serverMapImage, mapImage, serverMapCanvas, mapCanvas, allRobotsHandle, manualInput, initInput := gui.InitGui(chG2bCommand)
	go gui.ThreadGuiUpdate(
		serverMapImage,
		serverMapCanvas,
		allRobotsHandle,
		manualInput, initInput,
		chG2bCommand,
		chG2bRobotInit,
		chB2gRobotPendingInit,
		chB2gUpdate,
	)

	go gui.ThreadMapping(mapImage, mapCanvas, chB2gMapRectangle)

	fasitWindow.Show()
	serverWindow.Show()
	mapWindow.Show()

	// catch Ctrl-C/SIGTERM and trigger app.Quit()
	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()
	go func() {
		<-ctx.Done()
		fmt.Println("⚠️ shutdown signal received, quitting GUI…")
		app.Quit()
	}()

	// Run must be on main thread
	app.Run()

	// when Run() exits, all windows are closed – now persist the map
	fmt.Println("Shutting down, saving map…")
	if err := state.SaveMap("map.csv"); err != nil {
		fmt.Println("⚠️ failed to save map:", err)
	} else {
		fmt.Println("✔️ map saved to map.csv")
	}
}
