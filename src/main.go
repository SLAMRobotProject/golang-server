package main

import (
	"golang-server/backend"
	"golang-server/communication"
	"golang-server/gui"
	"golang-server/types"
)

func main() {
	//Most channels are buffered for efficiency.
<<<<<<< HEAD
	print("hellpo")
	//only backend can publish and receive
	chPublish := make(chan [3]int, 3)
	chPublishInit := make(chan [4]int, 3)
	chReceive := make(chan types.AdvMsg, 3)
	chReceiveMapFromRobot := make(chan types.RectangleMsg, 3)

	//chSubRecieve2 :=make(chan )
	//g2b = gui to backend
	chG2bRobotInit := make(chan [4]int, 3)
	chG2bCommand := make(chan types.Command)

	//b2g = backend to gui
	chB2gUpdate := make(chan types.UpdateGui, 3) //Buffered so it won't block ThreadBackend(types.AdvMsg
	chB2gRobotPendingInit := make(chan int, 3)   //Buffered so it won't block ThreadBackend()
	chB2gMapRectangle := make(chan types.RectangleMsg, 3)

	go backend.ThreadBackend( //add chPublishInit
		chPublish,
		chPublishInit,
		chReceive,
		chB2gRobotPendingInit,
		chB2gUpdate,
		chG2bRobotInit,
		chG2bCommand,
		chReceiveMapFromRobot,
		chB2gMapRectangle,
	)

	client := communication.InitMqtt()
	communication.Subscribe(client, chReceive, chReceiveMapFromRobot)
	go communication.ThreadMqttPublish(client, chPublish, chPublishInit)

	//window.ShowAndRun() must be run in the main thread. So the GUI must be initialized here.
	app, serverWindow, mapWindow,fasitWindow, serverMapImage, mapImage, serverMapCanvas, mapCanvas, allRobotsHandle, manualInput, initInput := gui.InitGui(chG2bCommand)
	go gui.ThreadGuiUpdate(
		serverMapImage,
		serverMapCanvas,
		allRobotsHandle,
		manualInput, initInput,
		chG2bCommand,
		chG2bRobotInit,
		chB2gRobotPendingInit,
		chB2gUpdate,
=======
	//only backend can publish and receive
	
		chPublish := make(chan [3]int, 3)
		chPublishInit := make(chan [4]int, 3)
		chPublishToggleMapping := make(chan [2]int,3)
		chReceive := make(chan types.AdvMsg, 3)
		chReceiveMapFromRobot := make(chan types.RectangleMsg, 3)

		//chSubRecieve2 :=make(chan )
		//g2b = gui to backend
		chG2bRobotInit := make(chan [4]int, 3)
		chG2bCommand := make(chan types.Command)
		chG2bToggleMapping:=make(chan [2]int,3)

		//b2g = backend to gui
		chB2gUpdate := make(chan types.UpdateGui, 3) //Buffered so it won't block ThreadBackend(types.AdvMsg
		chB2gRobotPendingInit := make(chan int, 3)   //Buffered so it won't block ThreadBackend()
		chB2gMapRectangle := make(chan types.RectangleMsg, 3)

		go backend.ThreadBackend( //add chPublishInit
			chPublish,
			chPublishInit,
			chPublishToggleMapping,
			chReceive,
			chB2gRobotPendingInit,
			chB2gUpdate,
			chG2bRobotInit,
			chG2bCommand,
			chG2bToggleMapping,
			chReceiveMapFromRobot,
			chB2gMapRectangle,
		)

		client := communication.InitMqtt()
		communication.Subscribe(client, chReceive, chReceiveMapFromRobot)
		go communication.ThreadMqttPublish(client, chPublish, chPublishInit,chPublishToggleMapping)


	app, serverWindow, mapWindow, serverMapImage, mapImage, serverMapCanvas, mapCanvas, allRobotsHandle, manualInput, initInput, mappingTab := gui.InitGui(chG2bCommand)
	
	go gui.ThreadGuiUpdate(
			serverMapImage,
			serverMapCanvas,
			allRobotsHandle,
			manualInput, initInput, mappingTab,
			chG2bCommand,
			chG2bRobotInit,
			chG2bToggleMapping,
			chB2gRobotPendingInit,
			chB2gUpdate,
>>>>>>> efdd8b5 (This server is adapted for the mapping scheme, should be checked before merging with main)
	)

	go gui.ThreadMapping(mapImage, mapCanvas, chB2gMapRectangle)

<<<<<<< HEAD
	fasitWindow.Show()
=======
>>>>>>> efdd8b5 (This server is adapted for the mapping scheme, should be checked before merging with main)
	serverWindow.Show()
	mapWindow.Show()
	app.Run()

}
