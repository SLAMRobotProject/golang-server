package main

import (
	"image"
	"image/color"
	"strconv"
	"time"

	"fyne.io/fyne/v2"
	"fyne.io/fyne/v2/app"
	"fyne.io/fyne/v2/canvas"
	"fyne.io/fyne/v2/container"
	"fyne.io/fyne/v2/widget"
)

var (
	//green = color.RGBA{0x00, 0xff, 0x00, 0xff}
	RED      = color.RGBA{0xff, 0x00, 0x00, 0xff}
	GREEN    = color.RGBA{0x00, 0xff, 0xff, 0xff}
	BLUE     = color.RGBA{0x00, 0x00, 0xff, 0xff}
	GRAY     = color.RGBA{0x80, 0x80, 0x80, 0xff}
	ORANGE   = color.RGBA{0xff, 0xd5, 0x00, 0xff}
	DARKRED  = color.RGBA{0x8b, 0x00, 0x00, 0xff}
	ORANGE_T = color.RGBA{0xff, 0xa5, 0x00, 0x80} //transparent orange
	WHITE    = color.White
	BLACK    = color.Black
)

func init_gui(
	ch_publish chan<- [3]int,
	ch_receive <-chan advMsg,
	ch_robotBackendInit chan<- [4]int,
) (fyne.Window, *image.RGBA, *canvas.Image, *multiRobotHandle, *container.AppTabs, *container.AppTabs) {

	a := app.New()
	w := a.NewWindow("Canvas")
	w.Resize(fyne.NewSize(WINDOW_BREADTH, WINDOW_HEIGHT))

	//map initialization
	map_shape := image.Rect(0, 0, MAP_SIZE, MAP_SIZE)
	map_image := image.NewRGBA(map_shape)
	for x := 0; x < MAP_SIZE; x++ {
		for y := 0; y < MAP_SIZE; y++ {
			map_image.Set(x, y, GRAY)
		}
	}
	map_canvas := canvas.NewImageFromImage(map_image)
	map_canvas.FillMode = canvas.ImageFillContain
	map_canvas.SetMinSize(fyne.NewSize(MAP_MINIMUM_DISPLAY_SIZE, MAP_MINIMUM_DISPLAY_SIZE))

	//robot initialization
	multi_robot_handle := init_multiRobotHandle()

	//input initialization
	manual_input := container.NewAppTabs()
	automatic_input := init_automaticInput(ch_publish)
	init_input := container.NewAppTabs()
	input_tabs := container.NewAppTabs(
		container.NewTabItem("Init", init_input),
		container.NewTabItem("Automatic", automatic_input),
		container.NewTabItem("Manual", manual_input),
	)

	//map axis initialization
	map_axis := init_mapAxis()
	axis_container := container.New(map_axis, map_axis.xAxis, map_axis.yAxis, map_axis.xText, map_axis.yText)

	//merging into one container
	map_with_robots := container.NewStack(map_canvas, axis_container, multi_robot_handle.multi_robot_container)
	input_and_map := container.NewHSplit(input_tabs, map_with_robots)
	w.SetContent(input_and_map)

	return w, map_image, map_canvas, multi_robot_handle, manual_input, init_input
}

func thread_guiUpdate(
	map_image *image.RGBA,
	map_canvas *canvas.Image,
	multi_robot_handle *multiRobotHandle,
	manual_input *container.AppTabs,
	init_input *container.AppTabs,
	ch_publish chan<- [3]int,
	ch_robotBackendInit chan<- [4]int,
	ch_robotPending <-chan int,
) {
	ch_robotGuiInit := make(chan [4]int, 3)
	ch_fps := time.Tick(time.Second / GUI_FRAME_RATE)
	for {
		select {
		case <-ch_fps:
			redraw_map(map_image, get_map())
			map_canvas.Refresh()
			redraw_robots(multi_robot_handle, get_multiRobotState())
		case id_pending := <-ch_robotPending:
			init_input.Append(container.NewTabItem("NRF-"+strconv.Itoa(id_pending), init_initInputTab(ch_robotBackendInit, ch_robotGuiInit, id_pending)))
		case init := <-ch_robotGuiInit:
			id := init[0]
			for i := 0; i < len(init_input.Items); i++ {
				if init_input.Items[i].Text == "NRF-"+strconv.Itoa(id) {
					init_input.Remove(init_input.Items[i])
				}
			}
			manual_input.Append(container.NewTabItem("NRF-"+strconv.Itoa(id), init_manualInputTab(ch_publish, id)))
		}
	}
}

func redraw_robots(multi_robot_handle *multiRobotHandle, multi_robot []robotState) {
	if len(multi_robot) > multi_robot_handle.NumRobots() {
		for i := multi_robot_handle.NumRobots(); i < len(multi_robot); i++ {
			//find ID
			for id, index := range get_id2index() {
				if index == i {
					multi_robot_handle.AddRobot(id)
					break
				}
			}
		}
	}
	for i := 0; i < len(multi_robot); i++ {
		multi_robot_handle.Move(i, fyne.NewPos(float32(multi_robot[i].x), -float32(multi_robot[i].y)))
		multi_robot_handle.Rotate(i, float64(multi_robot[i].theta))
	}
}

func redraw_map(map_image *image.RGBA, Map [MAP_SIZE][MAP_SIZE]uint8) {
	for x := 0; x < MAP_SIZE; x++ {
		for y := 0; y < MAP_SIZE; y++ {
			switch Map[x][y] {
			case MAP_UNKNOWN:
				map_image.Set(x, y, GRAY)
			case MAP_OBSTACLE:
				map_image.Set(x, y, BLACK)
			case MAP_OPEN:
				map_image.Set(x, y, WHITE)
			}
		}
	}
}

func init_initInputTab(ch_robotBackendInit, ch_robotGuiInit chan<- [4]int, id int) *fyne.Container {
	input_x := widget.NewEntry()
	input_x.SetPlaceHolder("x [cm]")

	input_y := widget.NewEntry()
	input_y.SetPlaceHolder("y [cm]")

	input_theta := widget.NewEntry()
	input_theta.SetPlaceHolder("theta [degrees]")

	input_button := widget.NewButton("Initialize", func() {
		//log.Println("Content was:", input_x.Text, ", ", input_y.Text)
		x, errX := strconv.Atoi(input_x.Text)
		y, errY := strconv.Atoi(input_y.Text)
		theta, errTheta := strconv.Atoi(input_theta.Text)
		if errX == nil && errY == nil && errTheta == nil {
			//send to backend [cm, cm, degrees]
			ch_robotBackendInit <- [4]int{id, x, y, theta}
			ch_robotGuiInit <- [4]int{id, x, y, theta}
			g_generalLogger.Println("Initializing robot with ID: ", id, " x: ", x, " y: ", y, " theta: ", theta, ".")
		} else {
			println("Invalid input")
			g_generalLogger.Println("Invalid input. Only integers are allowed.")

		}
	})

	default_button := widget.NewButton("Default [0, 0, 90]", func() {
		x, y, theta := 0, 0, 90
		ch_robotBackendInit <- [4]int{id, x, y, theta}
		ch_robotGuiInit <- [4]int{id, x, y, theta}

	})

	init_container := container.NewVBox(input_x, input_y, input_theta, input_button, default_button)
	return init_container
}

func init_automaticInput(ch_publish chan<- [3]int) *fyne.Container {
	input_x := widget.NewEntry()
	input_x.SetPlaceHolder("x [cm]")
	input_y := widget.NewEntry()
	input_y.SetPlaceHolder("y [cm]")
	automatic_container := container.NewVBox(input_x, input_y, widget.NewButton("Publish", func() {
		x, errX := strconv.Atoi(input_x.Text)
		y, errY := strconv.Atoi(input_y.Text)
		if errX == nil && errY == nil {
			id := find_closest_robot(x, y)
			if id == -1 {
				//already logged in find_closest_robot()
				return
			}

			//convert to mm because robot uses mm, and rotate back from init to get robot body coordinates
			robot := get_robot(id)
			x_robotBody, y_robotBody := rotate(float64(x-robot.x_init)*10, float64(y-robot.y_init)*10, -float64(robot.theta_init))
			ch_publish <- [3]int{id, int(x_robotBody), int(y_robotBody)}
			g_generalLogger.Println("Publishing automatic input to robot with ID: ", id, " x: ", x, " y: ", y, ".")
		} else {
			println("Invalid input")
			g_generalLogger.Println("Invalid input. Only integers are allowed.")
		}
	}))
	return automatic_container
}

func init_manualInputTab(ch_publish chan<- [3]int, id int) *fyne.Container {
	input_x := widget.NewEntry()
	input_x.SetPlaceHolder("x [cm]")
	input_y := widget.NewEntry()
	input_y.SetPlaceHolder("y [cm]")
	manual_container := container.NewVBox(input_x, input_y, widget.NewButton("Publish", func() {
		//log.Println("Content was:", input_x.Text, ", ", input_y.Text)
		x, errX := strconv.Atoi(input_x.Text)
		y, errY := strconv.Atoi(input_y.Text)
		if errX == nil && errY == nil {
			//convert to mm because robot uses mm, and rotate back from init to get robot body coordinates
			robot := get_robot(id)
			x_robotBody, y_robotBody := rotate(float64(x-robot.x_init)*10, float64(y-robot.y_init)*10, -float64(robot.theta_init))
			ch_publish <- [3]int{id, int(x_robotBody), int(y_robotBody)}
			g_generalLogger.Println("Publishing manual input to robot with ID: ", id, " x: ", x, " y: ", y, ".")
		} else {
			println("Invalid input")
			g_generalLogger.Println("Invalid input. Only integers are allowed.")
		}
	}))
	return manual_container
}
