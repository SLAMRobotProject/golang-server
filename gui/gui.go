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

func InitGui(
	chG2bCommand chan<- types.Command,
) (fyne.Window, *image.RGBA, *canvas.Image, *multiRobotHandle, *container.AppTabs, *container.AppTabs) {

	a := app.New()
	w := a.NewWindow("Canvas")
	w.Resize(fyne.NewSize(config.WINDOW_BREADTH, config.WINDOW_HEIGHT))

	//map initialization
	mapShape := image.Rect(0, 0, config.MAP_SIZE, config.MAP_SIZE)
	mapImage := image.NewRGBA(mapShape)
	for x := 0; x < config.MAP_SIZE; x++ {
		for y := 0; y < config.MAP_SIZE; y++ {
			mapImage.Set(x, y, GRAY)
		}
	}
	mapCanvas := canvas.NewImageFromImage(mapImage)
	mapCanvas.FillMode = canvas.ImageFillContain
	mapCanvas.SetMinSize(fyne.NewSize(config.MAP_MINIMUM_DISPLAY_SIZE, config.MAP_MINIMUM_DISPLAY_SIZE))

	//robot initialization
	allRobotsHandle := initMultiRobotHandle()

	//input initialization
	manualInput := container.NewAppTabs()
	automatic_input := initAutoInput(chG2bCommand)
	initInput := container.NewAppTabs()
	input_tabs := container.NewAppTabs(
		container.NewTabItem("Init", initInput),
		container.NewTabItem("Automatic", automatic_input),
		container.NewTabItem("Manual", manualInput),
	)

	//map axis initialization
	axis := init_mapAxis()
	axisContainer := container.New(axis, axis.xAxis, axis.yAxis, axis.xText, axis.yText)

	//merging into one container
	mapWithRobots := container.NewStack(mapCanvas, axisContainer, allRobotsHandle.container)
	InputAndMap := container.NewHSplit(input_tabs, mapWithRobots)
	w.SetContent(InputAndMap)

	return w, mapImage, mapCanvas, allRobotsHandle, manualInput, initInput
}

func ThreadGuiUpdate(
	mapImage *image.RGBA,
	mapCanvas *canvas.Image,
	allRobotsHandle *multiRobotHandle,
	manualInput *container.AppTabs,
	initInput *container.AppTabs,
	chG2bCommand chan<- types.Command,
	chG2bRobotInit chan<- [4]int,
	chB2gRobotPendingInit <-chan int,
	chB2gUpdate <-chan types.UpdateGui,
) {
	ch_robotGuiInit := make(chan [4]int, 3)
	for {
		select {
		case partialState := <-chB2gUpdate:
			redrawMap(mapImage, partialState.NewOpen, partialState.NewObstacle)
			mapCanvas.Refresh()
			redrawRobots(allRobotsHandle, partialState.MultiRobot, partialState.Id2index)
		case id_pending := <-chB2gRobotPendingInit:
			initInput.Append(container.NewTabItem("NRF-"+strconv.Itoa(id_pending), initInitializationInputTab(chG2bRobotInit, ch_robotGuiInit, id_pending)))
		case init := <-ch_robotGuiInit:
			id := init[0]
			for i := 0; i < len(initInput.Items); i++ {
				if initInput.Items[i].Text == "NRF-"+strconv.Itoa(id) {
					initInput.Remove(initInput.Items[i])
				}
			}
			manualInput.Append(container.NewTabItem("NRF-"+strconv.Itoa(id), initManualInputTab(chG2bCommand, id)))
		}
	}
}

func redrawRobots(allRobotsHandle *multiRobotHandle, backendMultiRobot []types.RobotState, backend_id2index map[int]int) {
	backendNumRobots := len(backendMultiRobot)
	if backendNumRobots > allRobotsHandle.NumRobots() {
		for i := allRobotsHandle.NumRobots(); i < backendNumRobots; i++ {
			//find ID
			for id, index := range backend_id2index {
				if index == i {
					allRobotsHandle.AddRobot(id)
					break
				}
			}
		}
	}
	for i := 0; i < backendNumRobots; i++ {
		allRobotsHandle.setPoseLabel(i, backendMultiRobot[i].X, backendMultiRobot[i].Y, backendMultiRobot[i].Theta)
		allRobotsHandle.Move(i, fyne.NewPos(float32(backendMultiRobot[i].X), -float32(backendMultiRobot[i].Y)))
		allRobotsHandle.Rotate(i, float64(backendMultiRobot[i].Theta))
	}
}

func redrawMap(mapImage *image.RGBA, newOpen [][2]int, newObstacle [][2]int) {
	for _, point := range newOpen {
		mapImage.Set(point[0], point[1], WHITE)
	}
	for _, point := range newObstacle {
		mapImage.Set(point[0], point[1], BLACK)
	}
}

func initInitializationInputTab(chG2bRobotInit, ch_robotGuiInit chan<- [4]int, id int) *fyne.Container {
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
			chG2bRobotInit <- [4]int{id, x, y, theta}
			ch_robotGuiInit <- [4]int{id, x, y, theta}
			log.GGeneralLogger.Println("Initializing robot with ID: ", id, " x: ", x, " y: ", y, " theta: ", theta, ".")
		} else {
			println("Invalid input")
			log.GGeneralLogger.Println("Invalid input. Only integers are allowed.")

		}
	})

	default_button := widget.NewButton("Default [0, 0, 90]", func() {
		x, y, theta := 0, 0, 90
		chG2bRobotInit <- [4]int{id, x, y, theta}
		ch_robotGuiInit <- [4]int{id, x, y, theta}

	})

	init_container := container.NewVBox(input_x, input_y, input_theta, input_button, default_button)
	return init_container
}

func initAutoInput(chG2bCommand chan<- types.Command) *fyne.Container {
	input_x := widget.NewEntry()
	input_x.SetPlaceHolder("x [cm]")
	input_y := widget.NewEntry()
	input_y.SetPlaceHolder("y [cm]")
	automatic_container := container.NewVBox(input_x, input_y, widget.NewButton("Publish", func() {
		x, errX := strconv.Atoi(input_x.Text)
		y, errY := strconv.Atoi(input_y.Text)
		if errX == nil && errY == nil {
			chG2bCommand <- types.Command{Command_type: types.AUTOMATIC_COMMAND, Id: -1, X: x, Y: y}
		} else {
			println("Invalid input")
			log.GGeneralLogger.Println("Invalid input. Only integers are allowed.")
		}
	}))
	return automatic_container
}

func initManualInputTab(chG2bCommand chan<- types.Command, id int) *fyne.Container {
	input_x := widget.NewEntry()
	input_x.SetPlaceHolder("x [cm]")
	input_y := widget.NewEntry()
	input_y.SetPlaceHolder("y [cm]")
	manual_container := container.NewVBox(input_x, input_y, widget.NewButton("Publish", func() {
		//log.Println("Content was:", input_x.Text, ", ", input_y.Text)
		x, errX := strconv.Atoi(input_x.Text)
		y, errY := strconv.Atoi(input_y.Text)
		if errX == nil && errY == nil {
			chG2bCommand <- types.Command{Command_type: types.MANUAL_COMMAND, Id: id, X: x, Y: y}
		} else {
			println("Invalid input")
			log.GGeneralLogger.Println("Invalid input. Only integers are allowed.")
		}
	}))
	return manual_container
}
