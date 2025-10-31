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
	red     = color.RGBA{0xff, 0x00, 0x00, 0xff}
	green   = color.RGBA{0x00, 0xff, 0xff, 0xff}
	blue    = color.RGBA{0x00, 0x00, 0xff, 0xff}
	gray    = color.RGBA{0x80, 0x80, 0x80, 0xff}
	darkRed = color.RGBA{0x8b, 0x00, 0x00, 0xff}
	orangeT = color.RGBA{0xff, 0xa5, 0x00, 0x80} //transparent orange
	white   = color.White
	black   = color.Black
)

func InitGui(
	chG2bCommand chan<- types.Command,
) (fyne.Window, *image.RGBA, *canvas.Image, *multiRobotHandle, *container.AppTabs, *container.AppTabs) {

	a := app.New()
	w := a.NewWindow("Canvas")
	w.Resize(fyne.NewSize(config.WindowBreadth, config.WindowHeight))

	//map initialization
	mapShape := image.Rect(0, 0, config.MapSize, config.MapSize)
	mapImage := image.NewRGBA(mapShape)
	for x := 0; x < config.MapSize; x++ {
		for y := 0; y < config.MapSize; y++ {
			mapImage.Set(x, y, gray)
		}
	}
	mapCanvas := canvas.NewImageFromImage(mapImage)
	mapCanvas.FillMode = canvas.ImageFillContain
	mapCanvas.SetMinSize(fyne.NewSize(config.MapMinimumDisplaySize, config.MapMinimumDisplaySize))

	//robot initialization
	allRobotsHandle := initMultiRobotHandle()

	//input initialization
	manualInput := container.NewAppTabs()
	automaticInput := initAutoInput(chG2bCommand)
	initInput := container.NewAppTabs()
	inputTabs := container.NewAppTabs(
		container.NewTabItem("Init", initInput),
		container.NewTabItem("Automatic", automaticInput),
		container.NewTabItem("Manual", manualInput),
	)

	//map axis initialization
	axis := initMapAxis()
	axisContainer := container.New(axis, axis.xAxis, axis.yAxis, axis.xText, axis.yText)

	//merging into one container
	mapWithRobots := container.NewStack(mapCanvas, axisContainer, allRobotsHandle.container)
	InputAndMap := container.NewHSplit(inputTabs, mapWithRobots)
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
	chRobotGuiInit := make(chan [4]int, 3)
	for {
		select {
		case partialState := <-chB2gUpdate:
			redrawMap(mapImage, partialState.NewOpen, partialState.NewObstacle)
			mapCanvas.Refresh()
			redrawRobots(allRobotsHandle, partialState.MultiRobot, partialState.Id2index)
		case idPending := <-chB2gRobotPendingInit:
			initInput.Append(container.NewTabItem("NRF-"+strconv.Itoa(idPending), initInitializationInputTab(chG2bRobotInit, chRobotGuiInit, idPending)))
		case init := <-chRobotGuiInit:
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

func redrawRobots(allRobotsHandle *multiRobotHandle, backendMultiRobot []types.RobotState, backendId2index map[int]int) {
	backendNumRobots := len(backendMultiRobot)
	if backendNumRobots > allRobotsHandle.NumRobots() {
		for i := allRobotsHandle.NumRobots(); i < backendNumRobots; i++ {
			//find ID
			for id, index := range backendId2index {
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
		mapImage.Set(point[0], point[1], white)
	}
	for _, point := range newObstacle {
		mapImage.Set(point[0], point[1], black)
	}
}

func initInitializationInputTab(chG2bRobotInit, chRobotGuiInit chan<- [4]int, id int) *fyne.Container {
	inputX := widget.NewEntry()
	inputX.SetPlaceHolder("x [cm]")

	inputY := widget.NewEntry()
	inputY.SetPlaceHolder("y [cm]")

	inputTheta := widget.NewEntry()
	inputTheta.SetPlaceHolder("theta [degrees]")

	inputButton := widget.NewButton("Initialize", func() {
		//log.Println("Content was:", inputX.Text, ", ", inputY.Text)
		x, errX := strconv.Atoi(inputX.Text)
		y, errY := strconv.Atoi(inputY.Text)
		theta, errTheta := strconv.Atoi(inputTheta.Text)
		if errX == nil && errY == nil && errTheta == nil {
			//send to backend [cm, cm, degrees]
			chG2bRobotInit <- [4]int{id, x, y, theta}
			chRobotGuiInit <- [4]int{id, x, y, theta}
			log.GGeneralLogger.Println("Initializing robot with ID: ", id, " x: ", x, " y: ", y, " theta: ", theta, ".")
		} else {
			println("Invalid input")
			log.GGeneralLogger.Println("Invalid input. Only integers are allowed.")

		}
	})

	defaultButton := widget.NewButton("Default [0, 0, 0]", func() {
		x, y, theta := 0, 0, 0
		chG2bRobotInit <- [4]int{id, x, y, theta}
		chRobotGuiInit <- [4]int{id, x, y, theta}

	})

	initContainer := container.NewVBox(inputX, inputY, inputTheta, inputButton, defaultButton)
	return initContainer
}

func initAutoInput(chG2bCommand chan<- types.Command) *fyne.Container {
	inputX := widget.NewEntry()
	inputX.SetPlaceHolder("x [cm]")
	inputY := widget.NewEntry()
	inputY.SetPlaceHolder("y [cm]")
	automaticContainer := container.NewVBox(inputX, inputY, widget.NewButton("Publish", func() {
		x, errX := strconv.Atoi(inputX.Text)
		y, errY := strconv.Atoi(inputY.Text)
		if errX == nil && errY == nil {
			chG2bCommand <- types.Command{CommandType: types.AutomaticCommand, Id: -1, X: x, Y: y}
		} else {
			println("Invalid input")
			log.GGeneralLogger.Println("Invalid input. Only integers are allowed.")
		}
	}))
	return automaticContainer
}

func initManualInputTab(chG2bCommand chan<- types.Command, id int) *fyne.Container {
	inputX := widget.NewEntry()
	inputX.SetPlaceHolder("x [cm]")
	inputY := widget.NewEntry()
	inputY.SetPlaceHolder("y [cm]")
	manualContainer := container.NewVBox(inputX, inputY, widget.NewButton("Publish", func() {
		//log.Println("Content was:", inputX.Text, ", ", inputY.Text)
		x, errX := strconv.Atoi(inputX.Text)
		y, errY := strconv.Atoi(inputY.Text)
		if errX == nil && errY == nil {
			chG2bCommand <- types.Command{CommandType: types.ManualCommand, Id: id, X: x, Y: y}
		} else {
			println("Invalid input")
			log.GGeneralLogger.Println("Invalid input. Only integers are allowed.")
		}
	}))
	return manualContainer
}
