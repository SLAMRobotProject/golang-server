package gui

import (
	//"fmt"
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
	"fyne.io/fyne/v2/layout"
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
	yellow  = color.RGBA{0xff, 0xff, 0x64, 0xff}
)

func InitGui(
	chG2bCommand chan<- types.Command,
) (fyne.App, fyne.Window, fyne.Window, fyne.Window, *image.RGBA, *image.RGBA, *canvas.Image, *canvas.Image, *multiRobotHandle, *container.AppTabs, *container.AppTabs) {

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

	w2 := a.NewWindow("MapImage")
	w2.Resize(fyne.NewSize(config.WindowBreadth, config.WindowHeight))

	//Initializing map
	mapImage2 := image.NewRGBA(mapShape)

	for x := 0; x < config.MapSize; x++ {
		for y := 0; y < config.MapSize; y++ {
			if (x%config.InitialSquareLength == 0 || y%config.InitialSquareLength == 0) && x != 0 && y != 0 {
				mapImage2.Set(x, y, white)
			} else {
				mapImage2.Set(x, y, gray)
			}
			if x == config.MapCenterX || y == config.MapCenterY {
				mapImage2.Set(x, y, orangeT)
			}
		}

	}

	mapCanvas2 := canvas.NewImageFromImage(mapImage2)
	mapCanvas2.FillMode = canvas.ImageFillContain
	mapCanvas2.SetMinSize(fyne.NewSize(config.MapMinimumDisplaySize, config.MapMinimumDisplaySize))

	text1 := canvas.NewText("Obstacle color: ", color.Black)
	text2 := canvas.NewText("Reachable color: ", color.Black)
	text3 := canvas.NewText("Not reachable color: ", color.Black)
	obsRect := image.NewRGBA(image.Rect(0, 0, config.InitialSquareLength, config.InitialSquareLength))
	reachRect := image.NewRGBA(image.Rect(0, 0, config.InitialSquareLength, config.InitialSquareLength))
	notReachRect := image.NewRGBA(image.Rect(0, 0, config.InitialSquareLength, config.InitialSquareLength))

	for x := 0; x < config.InitialSquareLength; x++ {
		for y := 0; y < config.InitialSquareLength; y++ {
			obsRect.Set(x, y, red)
			reachRect.Set(x, y, green)
			notReachRect.Set(x, y, yellow)
		}
	}

	obsCanvas := canvas.NewImageFromImage(obsRect)
	obsCanvas.FillMode = canvas.ImageFillContain
	obsCanvas.SetMinSize(fyne.NewSize(config.InitialSquareLength, config.InitialSquareLength))
	reachCanvas := canvas.NewImageFromImage(reachRect)
	reachCanvas.FillMode = canvas.ImageFillContain
	reachCanvas.SetMinSize(fyne.NewSize(config.InitialSquareLength, config.InitialSquareLength))
	notReachCanvas := canvas.NewImageFromImage(notReachRect)
	notReachCanvas.FillMode = canvas.ImageFillContain
	notReachCanvas.SetMinSize(fyne.NewSize(config.InitialSquareLength, config.InitialSquareLength))

	obsDef := container.New(layout.NewHBoxLayout(), text1, obsCanvas)
	reachDef := container.New(layout.NewHBoxLayout(), text2, reachCanvas)
	notReachDef := container.New(layout.NewHBoxLayout(), text3, notReachCanvas)
	content := container.New(layout.NewVBoxLayout(), obsDef, reachDef, notReachDef)

	infoAndMap := container.NewHSplit(content, mapCanvas2)

	w2.SetContent(infoAndMap)
	fasitW := a.NewWindow("Correct Result Test 1")
	// Create correct result for mapping
	/*

		squares := [16][16]int{
			{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{-1, -1, -1, -1, -1, -1, 1, 5, 5, 1, -1, -1, -1, -1, -1, -1},
			{-1, -1, -1, -1, -1, 1, 0, 0, 0, 0, 1, -1, -1, -1, -1, -1},
			{-1, -1, -1, -1, -1, 5, 0, 1, 1, 0, 5, -1, -1, -1, -1, -1},
			{-1, -1, -1, -1, -1, 5, 0, 0, 0, 0, 5, -1, -1, -1, -1, -1},
			{-1, -1, -1, -1, -1, 1, 0, 0, 0, 0, 1, -1, -1, -1, -1, -1},
			{-1, -1, -1, -1, -1, -1, 1, 5, 5, 1, -1, -1, -1, -1, -1, -1},
			{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		}

		fasitW.Resize(fyne.NewSize(config.WindowBreadth, config.WindowHeight))
		image3 := image.NewRGBA(mapShape)

		for x := 0; x < config.MapSize; x++ {
			for y := 0; y < config.MapSize; y++ {
				if (x%config.InitialSquareLength == 0 || y%config.InitialSquareLength == 0) && x != 0 && y != 0 {
					image3.Set(x, y, white)
				} else {
					if squares[x/config.InitialSquareLength][(y)/config.InitialSquareLength] == 0 && x != 400 && y != 400 && x != 0 && y != 0 {
						image3.Set(x, y, green)
					} else if squares[x/config.InitialSquareLength][(y)/config.InitialSquareLength] == 1 && x != 400 && y != 400 && x != 0 && y != 0 {
						image3.Set(x, y, red)
					} else if squares[x/config.InitialSquareLength][(y)/config.InitialSquareLength] == 5 && x != 400 && y != 400 && x != 0 && y != 0 {
						image3.Set(x, y, yellow)
					} else {
						image3.Set(x, y, gray)
					}

				}
				if x == config.MapCenterX || y == config.MapCenterY {
					image3.Set(x, y, orangeT)
				}
			}

		}

		canvasi := canvas.NewImageFromImage(image3)
		canvasi.FillMode = canvas.ImageFillContain
		canvasi.SetMinSize(fyne.NewSize(config.MapMinimumDisplaySize, config.MapMinimumDisplaySize))

		fasitW.SetContent(canvasi)
	*/
	return a, w, w2, fasitW, mapImage, mapImage2, mapCanvas, mapCanvas2, allRobotsHandle, manualInput, initInput
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

func ThreadMapping(
	mapImage *image.RGBA,
	mapCanvas *canvas.Image,
	chReceiveMap <-chan types.RectangleMsg,
) {
	robotMaps := make(map[int]types.MapRectangle)
	idToColor := make(map[int]color.RGBA) //colors may be changed to create better visual
	idToColor[1] = color.RGBA{0x00, 0xff, 0x00, 0xff}
	idToColor[2] = color.RGBA{0xde, 0x02, 0x61, 0xff}
	idToColor[3] = color.RGBA{0xde, 0xb6, 0x02, 0xff}
	idToColor[4] = color.RGBA{0xff, 0x22, 0x58, 0xff}
	idToColor[5] = color.RGBA{0x5f, 0x00, 0x58, 0xff}
	idToColor[6] = color.RGBA{0xf5, 0x47, 0xe9, 0xff}

	for {
		rect := <-chReceiveMap
		//fmt.Print("\nRecieved rectangle")
		rect.Height = rect.Height / 10 //Convert from mm to cm
		rect.Width = rect.Width / 10
		rect.X = rect.X / 10
		rect.Y = rect.Y / 10

		if !rect.TotalMap {
			for x := config.MapCenterX + rect.X; x <= config.MapCenterX+rect.X+rect.Width; x++ {
				for y := config.MapCenterY - rect.Y - rect.Height; y <= config.MapCenterY-rect.Y; y++ { //Takes into account negative y-axis

					if rect.Obstacle == 1 && !(y == config.MapCenterY-rect.Y-rect.Height || x == config.MapCenterX+rect.X || y == config.MapCenterY-rect.Y || x == config.MapCenterX+rect.X+rect.Width) {
						mapImage.Set(x, y, red)
					} else if rect.Obstacle == 0 && !rect.Reachable && !(y == config.MapCenterY-rect.Y-rect.Height || x == config.MapCenterX+rect.X || y == config.MapCenterY-rect.Y || x == config.MapCenterX+rect.X+rect.Width) {
						mapImage.Set(x, y, yellow)
					} else if rect.Obstacle == 0 && rect.Reachable && !(y == config.MapCenterY-rect.Y-rect.Height || x == config.MapCenterX+rect.X || y == config.MapCenterY-rect.Y || x == config.MapCenterX+rect.X+rect.Width) {
						mapImage.Set(x, y, green)
					}

				}
			}
		} else {
			totalMap, ok := robotMaps[rect.Id]
			if ok { //Need to remove the previous map
				for x := config.MapCenterX + totalMap.X; x <= config.MapCenterX+totalMap.X+totalMap.Width; x++ {
					for y := config.MapCenterY - totalMap.Y - totalMap.Height; y <= config.MapCenterY-totalMap.Y; y++ {
						if y == config.MapCenterY-totalMap.Y-totalMap.Height ||
							x == config.MapCenterX+totalMap.X ||
							y == config.MapCenterY-totalMap.Y ||
							x == config.MapCenterX+totalMap.X+totalMap.Width {
							mapImage.Set(x, y, white)
						}
					}
				}
			}

			//Draw new map
			for x := config.MapCenterX + rect.X; x <= config.MapCenterX+rect.X+rect.Width; x++ {
				for y := config.MapCenterY - rect.Y - rect.Height; y <= config.MapCenterY-rect.Y; y++ {
					if y == config.MapCenterY-rect.Y-rect.Height ||
						x == config.MapCenterX+rect.X ||
						y == config.MapCenterY-rect.Y ||
						x == config.MapCenterX+rect.X+rect.Width {
						mapImage.Set(x, y, idToColor[rect.Id])
					}
				}
			}
			//Store new map
			totalMap.X = rect.X
			totalMap.Y = rect.Y
			totalMap.Width = rect.Width
			totalMap.Height = rect.Height
			robotMaps[rect.Id] = totalMap

		}

		mapCanvas.Refresh()
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
		x, y, theta := 0, 0, 0 //Change to 0,0,0 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
