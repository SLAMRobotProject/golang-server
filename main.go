package main

func main() {
	//Most channels are buffered for efficiency.

	//only backend can publish and receive
	ch_publish := make(chan [3]int, 3)
	ch_receive := make(chan advMsg, 3)

	//g2b = gui to backend
	ch_g2b_robotInit := make(chan [4]int, 3)
	ch_g2b_command := make(chan guiCommand)

	//b2g = backend to gui
	ch_b2g_update := make(chan updateGui, 3)     //Buffered so it won't block thread_backend()
	ch_b2g_robotPendingInit := make(chan int, 3) //Buffered so it won't block thread_backend()

	go thread_backend(ch_publish, ch_b2g_robotPendingInit, ch_receive, ch_g2b_robotInit, ch_g2b_command, ch_b2g_update)

	client := mqtt_init()
	subscribe(client, ch_receive)
	go thread_publish(client, ch_publish)

	//window.ShowAndRun() must be run in the main thread. So the GUI must be initialized here.
	window, map_image, map_canvas, multi_robot_handle, manual_input, init_input := init_gui(ch_g2b_command, ch_receive, ch_g2b_robotInit)
	go thread_guiUpdate(map_image, map_canvas, multi_robot_handle, manual_input, init_input, ch_g2b_command, ch_g2b_robotInit, ch_b2g_robotPendingInit, ch_b2g_update)

	window.ShowAndRun()

}
