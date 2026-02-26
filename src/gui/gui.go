package gui

import (
	"golang-server/config"
	"golang-server/log"
	"golang-server/types"
	"image"
	"image/color"
	"math"
	"strconv"
	"sync"
	"time"

	"fyne.io/fyne/v2"
	"fyne.io/fyne/v2/app"
	"fyne.io/fyne/v2/canvas"
	"fyne.io/fyne/v2/container"
	"fyne.io/fyne/v2/widget"
)

var (
	red     = color.RGBA{0xff, 0x00, 0x00, 0xff}
	green   = color.RGBA{0x00, 0xff, 0x00, 0xff}
	blue    = color.RGBA{0x00, 0x00, 0xff, 0xff}
	gray    = color.RGBA{0x80, 0x80, 0x80, 0xff}
	darkRed = color.RGBA{0x8b, 0x00, 0x00, 0xff}
	orangeT = color.RGBA{0xff, 0xa5, 0x00, 0x80} //transparent orange
	white   = color.White
	black   = color.Black
)

var (
	lastMultiRobot []types.RobotState
	lastId2Index   map[int]int
	stateMu        sync.RWMutex

	// Variabler for linjetegning
	linesContainer *fyne.Container
	activeLines    []*canvas.Line
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

	linesContainer = container.NewWithoutLayout()

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
	mapWithRobots := container.NewStack(mapCanvas, axisContainer, linesContainer, allRobotsHandle.container)
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

			if len(partialState.Lines) > 0 {
				redrawLines(partialState.Lines)
			} else if len(activeLines) > 0 {
				for _, l := range activeLines {
					l.Hidden = true
				}
				linesContainer.Refresh()
			}

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

func redrawLines(lines [][2]types.Point) {
	needed := len(lines)
	current := len(activeLines)

	if needed > current {
		for i := 0; i < (needed - current); i++ {
			newLine := canvas.NewLine(green)
			newLine.StrokeWidth = 2
			activeLines = append(activeLines, newLine)
			linesContainer.Add(newLine)
		}
	}

	if current > 100 && current > needed*2 {
		for i := needed; i < current; i++ {
			linesContainer.Remove(activeLines[i])
		}
		activeLines = activeLines[:needed]
		current = needed
	}

	offsetX := float32(config.MapCenterX)
	offsetY := float32(config.MapCenterY)

	for i := 0; i < len(activeLines); i++ {
		lineObj := activeLines[i]
		if i < needed {
			p1 := lines[i][0]
			p2 := lines[i][1]

			lineObj.Position1 = fyne.NewPos(
				float32(p1.X)+offsetX,
				-float32(p1.Y)+offsetY,
			)

			lineObj.Position2 = fyne.NewPos(
				float32(p2.X)+offsetX,
				-float32(p2.Y)+offsetY,
			)

			lineObj.Hidden = false
		} else {
			lineObj.Hidden = true
		}
	}
	linesContainer.Refresh()
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

	defaultButton := widget.NewButton("Default [0, 0, 90]", func() {
		x, y, theta := 0, 0, 90
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

func SquareTestButton(chG2bCommand chan<- types.Command, id int) *widget.Button {
	const (
		acceptableRadius = 8
		stallThreshold   = time.Second
		dwellDuration    = 1500 * time.Millisecond
		checkInterval    = 200 * time.Millisecond
		graceDuration    = 4 * time.Second // give the robot time to rotate/start moving
	)

	squarePath := []struct{ X, Y int }{
		{0, 100}, {100, 100}, {100, 0}, {0, 0},
		{100, 0}, {100, 100}, {0, 100}, {0, 0},
	}

	send := func(x, y int) {
		chG2bCommand <- types.Command{CommandType: types.ManualCommand, Id: id, X: x, Y: y}
	}

	waitForTarget := func(targetX, targetY int) {
		time.Sleep(graceDuration) // ignore movement checks for a bit after sending

		var (
			lastPose   types.RobotState
			lastUpdate = time.Now()
		)

		for {
			time.Sleep(checkInterval)

			pose, ok := getRobotPose(id)
			if !ok {
				continue
			}

			dist := math.Hypot(float64(pose.X-targetX), float64(pose.Y-targetY))
			if dist <= acceptableRadius {
				time.Sleep(dwellDuration)
				return
			}

			if pose.X != lastPose.X || pose.Y != lastPose.Y {
				lastPose = pose
				lastUpdate = time.Now()
				continue
			}

			if time.Since(lastUpdate) > stallThreshold {
				send(targetX, targetY)
				lastUpdate = time.Now()
			}
		}
	}

	return widget.NewButton("Run square test", func() {
		go func() {
			for _, waypoint := range squarePath {
				send(waypoint.X, waypoint.Y)
				waitForTarget(waypoint.X, waypoint.Y)
			}
		}()
	})
}

func PatternTestButton(chG2bCommand chan<- types.Command, id int) *widget.Button {
	const (
		acceptableRadius = 8
		stallThreshold   = time.Second
		dwellDuration    = 1500 * time.Millisecond
		checkInterval    = 200 * time.Millisecond
		graceDuration    = 4 * time.Second // give the robot time to rotate/start moving
	)

	patternPath := []struct{ X, Y int }{
		{50, 50},
		{0, 100},
		{-30, 80},
		{40, 20},
		{-40, 20},
		{-40, 60},
		{0, 60},
		{0, 0},
	}

	send := func(x, y int) {
		chG2bCommand <- types.Command{CommandType: types.ManualCommand, Id: id, X: x, Y: y}
	}

	waitForTarget := func(targetX, targetY int) {
		time.Sleep(graceDuration) // ignore movement checks for a bit after sending

		var (
			lastPose   types.RobotState
			lastUpdate = time.Now()
		)

		for {
			time.Sleep(checkInterval)

			pose, ok := getRobotPose(id)
			if !ok {
				continue
			}

			dist := math.Hypot(float64(pose.X-targetX), float64(pose.Y-targetY))
			if dist <= acceptableRadius {
				time.Sleep(dwellDuration)
				return
			}

			if pose.X != lastPose.X || pose.Y != lastPose.Y {
				lastPose = pose
				lastUpdate = time.Now()
				continue
			}

			if time.Since(lastUpdate) > stallThreshold {
				send(targetX, targetY)
				lastUpdate = time.Now()
			}
		}
	}

	return widget.NewButton("Run pattern test", func() {
		go func() {
			for _, waypoint := range patternPath {
				send(waypoint.X, waypoint.Y)
				waitForTarget(waypoint.X, waypoint.Y)
			}
		}()
	})
}

func getRobotPose(id int) (types.RobotState, bool) {
	stateMu.RLock()
	defer stateMu.RUnlock()

	index, ok := lastId2Index[id]
	if !ok || index < 0 || index >= len(lastMultiRobot) {
		return types.RobotState{}, false
	}
	return lastMultiRobot[index], true
}
