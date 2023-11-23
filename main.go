package main

import (
	"golang-server/backend"
	"golang-server/communication"
	"golang-server/gui"
	"golang-server/types"
)

func main() {
	//Most channels are buffered for efficiency.

	//only backend can publish and receive
	ch_publish := make(chan [3]int, 3)
	ch_receive := make(chan types.AdvMsg, 3)

	//g2b = gui to backend
	ch_g2b_robotInit := make(chan [4]int, 3)
	ch_g2b_command := make(chan types.Command)

	//b2g = backend to gui
	ch_b2g_update := make(chan types.UpdateGui, 3) //Buffered so it won't block Thread_backend(types.AdvMsg
	ch_b2g_robotPendingInit := make(chan int, 3)   //Buffered so it won't block Thread_backend()

	go backend.Thread_backend(
		ch_publish,
		ch_receive,
		ch_b2g_robotPendingInit,
		ch_b2g_update,
		ch_g2b_robotInit,
		ch_g2b_command,
	)

	client := communication.Mqtt_init()
	communication.Subscribe(client, ch_receive)
	go communication.Thread_publish(client, ch_publish)

	//window.ShowAndRun() must be run in the main thread. So the GUI must be initialized here.
	window, map_image, map_canvas, multi_robot_handle, manual_input, init_input := gui.Init_gui(ch_g2b_command)
	go gui.Thread_guiUpdate(
		map_image,
		map_canvas,
		multi_robot_handle,
		manual_input, init_input,
		ch_g2b_command,
		ch_g2b_robotInit,
		ch_b2g_robotPendingInit,
		ch_b2g_update,
	)

	window.ShowAndRun()

}
