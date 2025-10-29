package main

import (
	//"golang-server/backend"
	//"golang-server/communication"
	//"golang-server/gui"
	"golang-server/slam"
	"golang-server/types"

	"log"

	"github.com/gorilla/websocket"
)

func main() {
	//Most channels are buffered for efficiency.

	//only backend can publish and receive
	//chPublish := make(chan [3]int, 3)
	//chReceive := make(chan types.AdvMsg, 3)
	chSensorData2Slam := make(chan types.SensorData, 3)

	//g2b = gui to backend
	//chG2bRobotInit := make(chan [4]int, 3)
	//chG2bCommand := make(chan types.Command)

	//b2g = backend to gui
	//chB2gUpdate := make(chan types.UpdateGui, 3) //Buffered so it won't block ThreadBackend(types.AdvMsg
	//chB2gRobotPendingInit := make(chan int, 3)   //Buffered so it won't block ThreadBackend()

	// go backend.ThreadBackend(
	// 	chPublish,
	// 	chReceive,
	// 	chB2gRobotPendingInit,
	// 	chB2gUpdate,
	// 	chG2bRobotInit,
	// 	chG2bCommand,
	// )

	//client := communication.InitMqtt()
	//communication.Subscribe(client, chReceive, chSensorData2Slam)
	//go communication.ThreadMqttPublish(client, chPublish)

	//window.ShowAndRun() must be run in the main thread. So the GUI must be initialized here.
	// window, mapImage, mapCanvas, allRobotsHandle, manualInput, initInput := gui.InitGui(chG2bCommand)
	// go gui.ThreadGuiUpdate(
	// 	mapImage,
	// 	mapCanvas,
	// 	allRobotsHandle,
	// 	manualInput, initInput,
	// 	chG2bCommand,
	// 	chG2bRobotInit,
	// 	chB2gRobotPendingInit,
	// 	chB2gUpdate,
	// )

	//SLAM WebSocket connection
	addr := "ws://localhost:8765"

	conn, _, err := websocket.DefaultDialer.Dial(addr, nil)
	if err != nil {
		log.Fatal("Dial error:", err)
	}
	defer conn.Close()

	go slam.SlamBackendLoop(chSensorData2Slam, conn)

	go slam.SimulateSensorData(chSensorData2Slam)

	//window.ShowAndRun()
	select {}

}
