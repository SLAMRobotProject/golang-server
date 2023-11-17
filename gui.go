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
	RED   = color.RGBA{0xff, 0x00, 0x00, 0xff}
	GREEN = color.RGBA{0x00, 0xff, 0xff, 0xff}
	BLUE  = color.RGBA{0x00, 0x00, 0xff, 0xff}
	GRAY  = color.RGBA{0x80, 0x80, 0x80, 0xff}
	WHITE = color.White
	BLACK = color.Black
)

func line_init(
	color color.Color,
	pos1 fyne.Position,
	pos2 fyne.Position,
	strokeWidth float32,
) *canvas.Line {
	l := canvas.NewLine(color)
	l.Position1 = pos1
	l.Position2 = pos2
	l.StrokeWidth = strokeWidth
	return l
}

func multi_robot_init() *multiRobotHandle {
	multiRobot_handle := NewMultiRobotHandle()
	//multiRobot_handle.AddRobot()
	return multiRobot_handle
}

func gui_init(
	ch_publish chan<- [3]int,
	ch_receive <-chan adv_msg,
	ch_robotBackendInit chan<- [4]int,
	ch_robotGuiInit chan<- [4]int,
) (fyne.Window, *image.RGBA, *canvas.Image, *multiRobotHandle, *container.AppTabs, *container.AppTabs) {

	a := app.New()
	w := a.NewWindow("Canvas")
	w.Resize(fyne.NewSize(650, 400))

	//map initialization
	map_shape := image.Rect(0, 0, map_size, map_size)
	map_image := image.NewRGBA(map_shape)
	for x := 0; x < map_size; x++ {
		for y := 0; y < map_size; y++ {
			map_image.Set(x, y, GRAY)
		}
	}
	map_canvas := canvas.NewImageFromImage(map_image)
	map_canvas.FillMode = canvas.ImageFillContain
	map_canvas.SetMinSize(fyne.NewSize(map_minimum_display_size, map_minimum_display_size))

	//input initialization
	manual_input := manual_input_init(ch_publish)
	automatic_input := automatic_input_init(ch_publish)
	init_input := init_input_init(ch_robotBackendInit, ch_robotGuiInit, 1)
	tabs := container.NewAppTabs(
		container.NewTabItem("Init", init_input),
		container.NewTabItem("Automatic", automatic_input),
		container.NewTabItem("Manual", manual_input),
	)

	multi_robot_handle := multi_robot_init()

	map_with_robots := container.NewStack(map_canvas, multi_robot_handle.multi_robot_container)

	grid := container.NewHSplit(tabs, map_with_robots)

	w.SetContent(grid)

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
	ch_robotGuiInit chan [4]int, //TODO: denne kan fjernes etterhvert, beholde kun inni her.
	ch_robotPending <-chan int,
) {
	ch_fps := time.Tick(time.Second / gui_frame_rate)
	for {
		select {
		case <-ch_fps:
			redraw_map(map_image, backend.Map)
			map_canvas.Refresh()
			redraw_robots(multi_robot_handle, backend.multi_robot)
		case id_pending := <-ch_robotPending:
			init_input.Append(container.NewTabItem("NRF-"+strconv.Itoa(id_pending), init_input_tabInit(ch_robotBackendInit, ch_robotGuiInit, id_pending)))
		case init := <-ch_robotGuiInit:
			id := init[0]
			for i := 0; i < len(init_input.Items); i++ {
				if init_input.Items[i].Text == "NRF-"+strconv.Itoa(id) {
					init_input.Remove(init_input.Items[i])
				}
			}
			manual_input.Append(container.NewTabItem("NRF-"+strconv.Itoa(id), manual_input_tabInit(ch_publish, id)))
		}
	}
}

func redraw_robots(multi_robot_handle *multiRobotHandle, multi_robot []Robot) {
	if len(multi_robot) > multi_robot_handle.NumRobots() {
		for i := multi_robot_handle.NumRobots(); i < len(multi_robot); i++ {
			//find ID
			for id, index := range backend.id2index {
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

func redraw_map(map_image *image.RGBA, Map [map_size][map_size]uint8) {
	for x := 0; x < map_size; x++ {
		for y := 0; y < map_size; y++ {
			switch Map[x][y] {
			case map_unknown:
				map_image.Set(x, y, GRAY)
			case map_obstacle:
				map_image.Set(x, y, BLACK)
			case map_open:
				map_image.Set(x, y, WHITE)
			}
		}
	}
}

func init_input_init(ch_robotBackendInit, ch_robotGuiInit chan<- [4]int, id int) *container.AppTabs {
	tabs := container.NewAppTabs(
	//container.NewTabItem("NRF-"+strconv.Itoa(id), init_input_tabInit(ch_robotBackendInit, ch_robotGuiInit, id)),
	)
	return tabs
}

func init_input_tabInit(ch_robotBackendInit, ch_robotGuiInit chan<- [4]int, id int) *fyne.Container {
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
			general_logger.Println("Initializing robot with ID: ", id, " x: ", x, " y: ", y, " theta: ", theta, ".")
		} else {
			println("Invalid input")
			general_logger.Println("Invalid input. Only integers are allowed.")

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

func automatic_input_init(ch_publish chan<- [3]int) *fyne.Container {
	input_x := widget.NewEntry()
	input_x.SetPlaceHolder("x [cm]")
	input_y := widget.NewEntry()
	input_y.SetPlaceHolder("y [cm]")
	automatic_container := container.NewVBox(input_x, input_y, widget.NewButton("Publish", func() {
		x, errX := strconv.Atoi(input_x.Text)
		y, errY := strconv.Atoi(input_y.Text)
		if errX == nil && errY == nil {
			id := backend.find_closest_robot(x, y)
			if id == -1 {
				//already logged in find_closest_robot()
				return
			}

			//convert to mm because robot uses mm, and rotate back from init to get robot body coordinates
			robot := backend.multi_robot[backend.id2index[id]]
			x_robotBody, y_robotBody := rotate(float64(x-robot.x_init)*10, float64(y-robot.y_init)*10, -float64(robot.theta_init))
			ch_publish <- [3]int{id, int(x_robotBody), int(y_robotBody)}
			general_logger.Println("Publishing automatic input to robot with ID: ", id, " x: ", x, " y: ", y, ".")
		} else {
			println("Invalid input")
			general_logger.Println("Invalid input. Only integers are allowed.")
		}
	}))
	return automatic_container
}

func manual_input_init(ch_robotInit chan<- [3]int) *container.AppTabs {
	//id := 2
	tabs := container.NewAppTabs(
	//container.NewTabItem("NRF-"+strconv.Itoa(id), manual_input_tabInit(ch_robotInit, id)),
	)
	return tabs
}

func manual_input_tabInit(ch_publish chan<- [3]int, id int) *fyne.Container {
	input_x := widget.NewEntry()
	input_x.SetPlaceHolder("x [cm]")
	input_y := widget.NewEntry()
	input_y.SetPlaceHolder("y [cm]")
	manual_container := container.NewVBox(input_x, input_y, widget.NewButton("Publish", func() {
		//log.Println("Content was:", input_x.Text, ", ", input_y.Text)
		x, errX := strconv.Atoi(input_x.Text)
		y, errY := strconv.Atoi(input_y.Text)
		//TODO: fix id
		if errX == nil && errY == nil {
			//convert to mm because robot uses mm, and rotate back from init to get robot body coordinates
			robot := backend.multi_robot[backend.id2index[id]]
			x_robotBody, y_robotBody := rotate(float64(x-robot.x_init)*10, float64(y-robot.y_init)*10, -float64(robot.theta_init))
			ch_publish <- [3]int{id, int(x_robotBody), int(y_robotBody)}
			general_logger.Println("Publishing manual input to robot with ID: ", id, " x: ", x, " y: ", y, ".")
		} else {
			println("Invalid input")
			general_logger.Println("Invalid input. Only integers are allowed.")
		}
	}))
	return manual_container
}
