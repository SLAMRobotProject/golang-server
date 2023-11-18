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

func gui_init(
	ch_publish chan<- [3]int,
	ch_receive <-chan adv_msg,
	ch_robotBackendInit chan<- [4]int,
) (fyne.Window, *image.RGBA, *canvas.Image, *multiRobotHandle, *container.AppTabs, *container.AppTabs) {

	a := app.New()
	w := a.NewWindow("Canvas")
	w.Resize(fyne.NewSize(window_breadth, window_height))

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

	//robot initialization
	multi_robot_handle := multi_robot_init()

	//input initialization
	manual_input := container.NewAppTabs()
	automatic_input := automatic_input_init(ch_publish)
	init_input := container.NewAppTabs()
	input_tabs := container.NewAppTabs(
		container.NewTabItem("Init", init_input),
		container.NewTabItem("Automatic", automatic_input),
		container.NewTabItem("Manual", manual_input),
	)

	//map axis initialization
	map_axis := NewMapAxis()
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

func manual_input_tabInit(ch_publish chan<- [3]int, id int) *fyne.Container {
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

// Axis is below here

type mapAxis struct {
	xAxis, yAxis *canvas.Line
	xText, yText *canvas.Text
}

func NewMapAxis() *mapAxis {
	xAxis := canvas.NewLine(ORANGE_T)
	xAxis.Position1 = fyne.NewPos(0, map_center_y)
	xAxis.Position2 = fyne.NewPos(map_size, map_center_y)
	yAxis := canvas.NewLine(ORANGE_T)
	yAxis.Position1 = fyne.NewPos(map_center_x, 0)
	yAxis.Position2 = fyne.NewPos(map_center_x, map_size)
	xText := canvas.NewText("x="+strconv.Itoa(map_size-map_center_x), DARKRED)
	yText := canvas.NewText("y="+strconv.Itoa(map_size-map_center_y), DARKRED)

	return &mapAxis{xAxis, yAxis, xText, yText}
}

// Layout is called to pack all child objects into a specified size.
func (m *mapAxis) Layout(objects []fyne.CanvasObject, size fyne.Size) {
	//The map is square and centered, but we must offset the position of the lines relative to the top left corner of the container
	dx, dy := float32(0), float32(0)
	if size.Height > size.Width {
		dy += (size.Height - size.Width) / 2
	} else {
		dx += (size.Width - size.Height) / 2
	}
	current_map_size := min(size.Height, size.Width)
	current_ratio := current_map_size / float32(map_size)

	m.xAxis.Position1 = fyne.NewPos(dx, map_center_y*current_ratio+dy)
	m.xAxis.Position2 = fyne.NewPos(current_map_size+dx, map_center_y*current_ratio+dy)

	m.yAxis.Position1 = fyne.NewPos(map_center_x*current_ratio+dx, dy)
	m.yAxis.Position2 = fyne.NewPos(map_center_x*current_ratio+dx, current_map_size+dy)

	m.xText.Move(fyne.NewPos(current_map_size+dx-43, map_center_y*current_ratio+dy))
	m.yText.Move(fyne.NewPos(map_center_x*current_ratio+3+dx, dy+1))
}

// MinSize finds the smallest size that satisfies all the child objects.
func (m *mapAxis) MinSize(objects []fyne.CanvasObject) fyne.Size {
	minSize := fyne.NewSize(0, 0)
	for _, child := range objects {
		minSize = minSize.Max(child.Size())
	}
	return minSize
}
