package gui

import (
	"golang-server/config"
	"golang-server/log"
	"golang-server/types"
	"image"
	"image/color"
	"strconv"

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

func Init_gui(
	ch_g2b_command chan<- types.Command,
) (fyne.Window, *image.RGBA, *canvas.Image, *multiRobotHandle, *container.AppTabs, *container.AppTabs) {

	a := app.New()
	w := a.NewWindow("Canvas")
	w.Resize(fyne.NewSize(config.WINDOW_BREADTH, config.WINDOW_HEIGHT))

	//map initialization
	map_shape := image.Rect(0, 0, config.MAP_SIZE, config.MAP_SIZE)
	map_image := image.NewRGBA(map_shape)
	for x := 0; x < config.MAP_SIZE; x++ {
		for y := 0; y < config.MAP_SIZE; y++ {
			map_image.Set(x, y, GRAY)
		}
	}
	map_canvas := canvas.NewImageFromImage(map_image)
	map_canvas.FillMode = canvas.ImageFillContain
	map_canvas.SetMinSize(fyne.NewSize(config.MAP_MINIMUM_DISPLAY_SIZE, config.MAP_MINIMUM_DISPLAY_SIZE))

	//robot initialization
	multi_robot_handle := init_multiRobotHandle()

	//input initialization
	manual_input := container.NewAppTabs()
	automatic_input := init_automaticInput(ch_g2b_command)
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

func Thread_guiUpdate(
	map_image *image.RGBA,
	map_canvas *canvas.Image,
	multi_robot_handle *multiRobotHandle,
	manual_input *container.AppTabs,
	init_input *container.AppTabs,
	ch_g2b_command chan<- types.Command,
	ch_g2b_robotInit chan<- [4]int,
	ch_b2g_robotPendingInit <-chan int,
	ch_b2g_update <-chan types.UpdateGui,
) {
	ch_robotGuiInit := make(chan [4]int, 3)
	for {
		select {
		case partial_state := <-ch_b2g_update:
			redraw_map(map_image, partial_state.New_open, partial_state.New_obstacle)
			map_canvas.Refresh()
			redraw_robots(multi_robot_handle, partial_state.Multi_robot, partial_state.Id2index)
		case id_pending := <-ch_b2g_robotPendingInit:
			init_input.Append(container.NewTabItem("NRF-"+strconv.Itoa(id_pending), init_initInputTab(ch_g2b_robotInit, ch_robotGuiInit, id_pending)))
		case init := <-ch_robotGuiInit:
			id := init[0]
			for i := 0; i < len(init_input.Items); i++ {
				if init_input.Items[i].Text == "NRF-"+strconv.Itoa(id) {
					init_input.Remove(init_input.Items[i])
				}
			}
			manual_input.Append(container.NewTabItem("NRF-"+strconv.Itoa(id), init_manualInputTab(ch_g2b_command, id)))
		}
	}
}

func redraw_robots(multi_robot_handle *multiRobotHandle, backend_multiRobot []types.RobotState, backend_id2index map[int]int) {
	backend_num_robots := len(backend_multiRobot)
	if backend_num_robots > multi_robot_handle.NumRobots() {
		for i := multi_robot_handle.NumRobots(); i < backend_num_robots; i++ {
			//find ID
			for id, index := range backend_id2index {
				if index == i {
					multi_robot_handle.AddRobot(id)
					break
				}
			}
		}
	}
	for i := 0; i < backend_num_robots; i++ {
		multi_robot_handle.set_pose_label(i, backend_multiRobot[i].X, backend_multiRobot[i].Y, backend_multiRobot[i].Theta)
		multi_robot_handle.Move(i, fyne.NewPos(float32(backend_multiRobot[i].X), -float32(backend_multiRobot[i].Y)))
		multi_robot_handle.Rotate(i, float64(backend_multiRobot[i].Theta))
	}
}

func redraw_map(map_image *image.RGBA, new_open [][2]int, new_obstacle [][2]int) {
	for _, point := range new_open {
		map_image.Set(point[0], point[1], WHITE)
	}
	for _, point := range new_obstacle {
		map_image.Set(point[0], point[1], BLACK)
	}
}

func init_initInputTab(ch_g2b_robotInit, ch_robotGuiInit chan<- [4]int, id int) *fyne.Container {
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
			ch_g2b_robotInit <- [4]int{id, x, y, theta}
			ch_robotGuiInit <- [4]int{id, x, y, theta}
			log.G_generalLogger.Println("Initializing robot with ID: ", id, " x: ", x, " y: ", y, " theta: ", theta, ".")
		} else {
			println("Invalid input")
			log.G_generalLogger.Println("Invalid input. Only integers are allowed.")

		}
	})

	default_button := widget.NewButton("Default [0, 0, 90]", func() {
		x, y, theta := 0, 0, 90
		ch_g2b_robotInit <- [4]int{id, x, y, theta}
		ch_robotGuiInit <- [4]int{id, x, y, theta}

	})

	init_container := container.NewVBox(input_x, input_y, input_theta, input_button, default_button)
	return init_container
}

func init_automaticInput(ch_g2b_command chan<- types.Command) *fyne.Container {
	input_x := widget.NewEntry()
	input_x.SetPlaceHolder("x [cm]")
	input_y := widget.NewEntry()
	input_y.SetPlaceHolder("y [cm]")
	automatic_container := container.NewVBox(input_x, input_y, widget.NewButton("Publish", func() {
		x, errX := strconv.Atoi(input_x.Text)
		y, errY := strconv.Atoi(input_y.Text)
		if errX == nil && errY == nil {
			ch_g2b_command <- types.Command{Command_type: types.AUTOMATIC_COMMAND, Id: -1, X: x, Y: y}
		} else {
			println("Invalid input")
			log.G_generalLogger.Println("Invalid input. Only integers are allowed.")
		}
	}))
	return automatic_container
}

func init_manualInputTab(ch_g2b_command chan<- types.Command, id int) *fyne.Container {
	input_x := widget.NewEntry()
	input_x.SetPlaceHolder("x [cm]")
	input_y := widget.NewEntry()
	input_y.SetPlaceHolder("y [cm]")
	manual_container := container.NewVBox(input_x, input_y, widget.NewButton("Publish", func() {
		//log.Println("Content was:", input_x.Text, ", ", input_y.Text)
		x, errX := strconv.Atoi(input_x.Text)
		y, errY := strconv.Atoi(input_y.Text)
		if errX == nil && errY == nil {
			ch_g2b_command <- types.Command{Command_type: types.MANUAL_COMMAND, Id: id, X: x, Y: y}
		} else {
			println("Invalid input")
			log.G_generalLogger.Println("Invalid input. Only integers are allowed.")
		}
	}))
	return manual_container
}
