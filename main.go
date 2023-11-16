package main

func main() {
	ch_publish := make(chan [3]int)
	ch_receive := make(chan adv_msg)
	ch_robotBackendInit := make(chan [4]int)
	ch_robotGuiInit := make(chan [4]int)
	ch_robotPending := make(chan int, 3) //Buffered channel, so it won't block thread_backend()

	go thread_backend(ch_publish, ch_robotPending, ch_receive, ch_robotBackendInit)

	client := mqtt_init()
	sub(client, ch_receive)
	go publish(client, ch_publish)

	//window.ShowAndRun() must be run in the main thread. So the GUI must be initialized here.
	window, map_image, map_canvas, multi_robot_handle, manual_input, init_input := gui_init(ch_publish, ch_receive, ch_robotBackendInit, ch_robotGuiInit)
	go thread_guiUpdate(map_image, map_canvas, multi_robot_handle, manual_input, init_input, ch_publish, ch_robotBackendInit, ch_robotGuiInit, ch_robotPending)

	window.ShowAndRun()

}
