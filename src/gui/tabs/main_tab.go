package tabs

import (
	"fmt"
	"golang-server/gui/widgets"
	"golang-server/log"
	pathP "golang-server/pathplanner"
	"golang-server/types"
	"image"
	"image/color"
	"strconv"

	"fyne.io/fyne/v2"
	"fyne.io/fyne/v2/canvas"
	"fyne.io/fyne/v2/container"
	"fyne.io/fyne/v2/layout"
	"fyne.io/fyne/v2/widget"
)

const (
	inputFieldWidth  = float32(110)
	inputFieldHeight = float32(36)
)

type mainInputTabs struct {
	Tabs              *container.AppTabs
	ManualLayout      *fyne.Container
	PhysicalInitList  *fyne.Container
	VirtualInitList   *fyne.Container
	PhysicalInitEmpty *widget.Label
	VirtualInitEmpty  *widget.Label
	ManualRobotSelect *widget.Select
	PathRobotSelect   *widget.Select
}

type MainTab struct {
	MapImage          *image.RGBA
	MapCanvas         *canvas.Image
	Content           fyne.CanvasObject
	ManualLayout      *fyne.Container
	PhysicalInitList  *fyne.Container
	VirtualInitList   *fyne.Container
	PhysicalInitEmpty *widget.Label
	VirtualInitEmpty  *widget.Label
	ManualRobotSelect *widget.Select
	PathRobotSelect   *widget.Select
}

// BuildMainTab creates the main tab content with controls on the left and map stack on the right.
func BuildMainTab(
	chG2bCommand chan<- types.Command,
	getSelectedManualRobotID func() int,
	getSelectedPathRobotID func() int,
	onManualRobotSelected func(string),
	onPathRobotSelected func(string),
	getRobotPose pathP.ManualPoseProvider,
	getAnyRobotPose pathP.AutoPoseProvider,
	mapSize int,
	fillColor color.Color,
	minMapSize fyne.Size,
	axisOverlay fyne.CanvasObject,
	linesOverlay fyne.CanvasObject,
	robotsOverlay fyne.CanvasObject,
	useSmoothScaling bool,
) *MainTab {
	inputTabSet := buildMainInputTabs(
		chG2bCommand,
		getSelectedManualRobotID,
		getSelectedPathRobotID,
		onManualRobotSelected,
		onPathRobotSelected,
		getRobotPose,
		getAnyRobotPose,
	)

	mapImage := widgets.BuildBaseMapImage(mapSize, fillColor)
	mapCanvas := widgets.BuildMapCanvas(mapImage, minMapSize, useSmoothScaling)

	mapWithRobots := container.NewStack(mapCanvas, axisOverlay, linesOverlay, robotsOverlay)

	return &MainTab{
		MapImage:          mapImage,
		MapCanvas:         mapCanvas,
		Content:           container.NewHSplit(inputTabSet.Tabs, mapWithRobots),
		ManualLayout:      inputTabSet.ManualLayout,
		PhysicalInitList:  inputTabSet.PhysicalInitList,
		VirtualInitList:   inputTabSet.VirtualInitList,
		PhysicalInitEmpty: inputTabSet.PhysicalInitEmpty,
		VirtualInitEmpty:  inputTabSet.VirtualInitEmpty,
		ManualRobotSelect: inputTabSet.ManualRobotSelect,
		PathRobotSelect:   inputTabSet.PathRobotSelect,
	}
}

func buildMainInputTabs(
	chG2bCommand chan<- types.Command,
	getSelectedManualRobotID func() int,
	getSelectedPathRobotID func() int,
	onManualRobotSelected func(string),
	onPathRobotSelected func(string),
	getRobotPose pathP.ManualPoseProvider,
	getAnyRobotPose pathP.AutoPoseProvider,
) *mainInputTabs {
	manualLayout, manualRobotSelect := buildManualControlLayout(chG2bCommand, getSelectedManualRobotID, onManualRobotSelected)
	autoLayout := buildAutoControlLayout(chG2bCommand)
	pathLayout, pathRobotSelect := buildPathControlLayout(chG2bCommand, getSelectedPathRobotID, onPathRobotSelected, getRobotPose, getAnyRobotPose)

	initLayout, physicalInitList, virtualInitList, physicalInitEmpty, virtualInitEmpty := buildInitRobotsLayout()
	controlLayout := buildControlModeLayout(manualLayout, autoLayout)

	inputTabs := container.NewAppTabs(
		container.NewTabItem("Init", initLayout),
		container.NewTabItem("Control", controlLayout),
		container.NewTabItem("Path", pathLayout),
	)

	return &mainInputTabs{
		Tabs:              inputTabs,
		ManualLayout:      manualLayout,
		PhysicalInitList:  physicalInitList,
		VirtualInitList:   virtualInitList,
		PhysicalInitEmpty: physicalInitEmpty,
		VirtualInitEmpty:  virtualInitEmpty,
		ManualRobotSelect: manualRobotSelect,
		PathRobotSelect:   pathRobotSelect,
	}
}

func buildControlModeLayout(manualLayout, autoLayout *fyne.Container) *fyne.Container {
	controlModeBody := container.NewStack(manualLayout)
	controlModeIsManual := true
	controlModeLabel := widget.NewLabel("Mode: Manual")
	controlModeToggle := widgets.NewToggleButton(false, func(on bool) {
		controlModeIsManual = !on
		if controlModeIsManual {
			controlModeLabel.SetText("Mode: Manual")
			controlModeBody.Objects = []fyne.CanvasObject{manualLayout}
		} else {
			controlModeLabel.SetText("Mode: Auto")
			controlModeBody.Objects = []fyne.CanvasObject{autoLayout}
		}
		controlModeBody.Refresh()
	})
	modeToggleRow := container.NewHBox(
		widget.NewLabel("Manual"),
		container.NewCenter(controlModeToggle),
		widget.NewLabel("Auto"),
	)
	modeHeaderRow := container.NewHBox(
		controlModeLabel,
		layout.NewSpacer(),
		modeToggleRow,
	)

	return container.NewVBox(
		modeHeaderRow,
		controlModeBody,
	)
}

func buildInitRobotsLayout() (
	*fyne.Container,
	*fyne.Container,
	*fyne.Container,
	*widget.Label,
	*widget.Label,
) {
	physicalInitList := container.NewVBox()
	physicalInitEmpty := widget.NewLabel("No physical robots detected")
	physicalHeader := container.NewVBox(widget.NewLabel("Physical robots"), widget.NewSeparator())
	physicalScroll := container.NewVScroll(container.NewVBox(physicalInitEmpty, physicalInitList))
	physicalScroll.SetMinSize(fyne.NewSize(320, 160))
	physicalInitPanel := container.NewBorder(
		physicalHeader,
		nil,
		nil,
		nil,
		physicalScroll,
	)

	virtualInitList := container.NewVBox()
	virtualInitEmpty := widget.NewLabel("No virtual robots detected")
	virtualHeader := container.NewVBox(widget.NewLabel("Virtual robots"), widget.NewSeparator())
	virtualScroll := container.NewVScroll(container.NewVBox(virtualInitEmpty, virtualInitList))
	virtualScroll.SetMinSize(fyne.NewSize(320, 160))
	virtualInitPanel := container.NewBorder(
		virtualHeader,
		nil,
		nil,
		nil,
		virtualScroll,
	)

	initLayout := container.NewVBox(physicalInitPanel, virtualInitPanel)
	return initLayout, physicalInitList, virtualInitList, physicalInitEmpty, virtualInitEmpty
}

func buildAutoControlLayout(chG2bCommand chan<- types.Command) *fyne.Container {
	xValue := 0
	yValue := 0

	inputX := widget.NewEntry()
	inputY := widget.NewEntry()
	inputX.SetPlaceHolder("[cm]")
	inputY.SetPlaceHolder("[cm]")
	inputXWrap := container.NewGridWrap(fyne.NewSize(inputFieldWidth, inputFieldHeight), inputX)
	inputYWrap := container.NewGridWrap(fyne.NewSize(inputFieldWidth, inputFieldHeight), inputY)

	automaticLayout := container.NewVBox(
		container.NewHBox(
			widget.NewLabel("x:"), inputXWrap,
			widget.NewLabel("y:"), inputYWrap,
		),
		widget.NewButton("Publish", func() {
			var errX, errY error
			xValue, errX = strconv.Atoi(inputX.Text)
			yValue, errY = strconv.Atoi(inputY.Text)
			if errX == nil && errY == nil {
				chG2bCommand <- types.Command{CommandType: types.AutomaticCommand, Id: -1, X: xValue, Y: yValue}
			} else {
				println("Invalid input")
				log.GGeneralLogger.Println("Invalid input. Only integers are allowed.")
			}
		}),
	)
	return automaticLayout
}

func buildManualControlLayout(
	chG2bCommand chan<- types.Command,
	getSelectedManualRobotID func() int,
	onManualRobotSelected func(string),
) (*fyne.Container, *widget.Select) {
	manualRobotSelect := widget.NewSelect([]string{}, onManualRobotSelected)
	manualRobotSelect.PlaceHolder = "Select robot"

	inputX := widget.NewEntry()
	inputY := widget.NewEntry()
	inputX.SetPlaceHolder("[cm]")
	inputY.SetPlaceHolder("[cm]")
	inputXWrap := container.NewGridWrap(fyne.NewSize(inputFieldWidth, inputFieldHeight), inputX)
	inputYWrap := container.NewGridWrap(fyne.NewSize(inputFieldWidth, inputFieldHeight), inputY)

	manualLayout := container.NewVBox(
		manualRobotSelect,
		container.NewHBox(
			widget.NewLabel("x:"), inputXWrap,
			widget.NewLabel("y:"), inputYWrap,
		),
		widget.NewButton("Publish", func() {
			x, errX := strconv.Atoi(inputX.Text)
			y, errY := strconv.Atoi(inputY.Text)
			if getSelectedManualRobotID() != -1 && errX == nil && errY == nil {
				chG2bCommand <- types.Command{CommandType: types.ManualCommand, Id: getSelectedManualRobotID(), X: x, Y: y}
			} else {
				println("Invalid input")
				log.GGeneralLogger.Println("Invalid input. Select a robot and enter integer x/y values.")
			}
		}),
	)

	return manualLayout, manualRobotSelect
}

func buildPathControlLayout(
	chG2bCommand chan<- types.Command,
	getSelectedPathRobotID func() int,
	onPathRobotSelected func(string),
	getRobotPose pathP.ManualPoseProvider,
	getAnyRobotPose pathP.AutoPoseProvider,
) (*fyne.Container, *widget.Select) {
	pathPlanner := pathP.NewPathPlanner()

	pathRobotSelect := widget.NewSelect([]string{}, onPathRobotSelected)
	pathRobotSelect.PlaceHolder = "Select robot"

	type waypointRow struct {
		xEntry *widget.Entry
		yEntry *widget.Entry
		row    *fyne.Container
	}

	waypointRows := make([]waypointRow, 0)
	waypointList := container.NewVBox()
	nextTargetLabel := widget.NewLabel("Next target: -")
	pathBusy := false

	var addWaypointBtn *widget.Button
	var runCustomPathBtn *widget.Button
	var runSquareBtn *widget.Button
	var runVirtualPathBtn *widget.Button

	setPathBusy := func(busy bool) {
		pathBusy = busy
		if addWaypointBtn != nil {
			if busy {
				addWaypointBtn.Disable()
			} else {
				addWaypointBtn.Enable()
			}
		}
		if runCustomPathBtn != nil {
			if busy {
				runCustomPathBtn.Disable()
			} else {
				runCustomPathBtn.Enable()
			}
		}
		if runSquareBtn != nil {
			if busy {
				runSquareBtn.Disable()
			} else {
				runSquareBtn.Enable()
			}
		}
		if runVirtualPathBtn != nil {
			if busy {
				runVirtualPathBtn.Disable()
			} else {
				runVirtualPathBtn.Enable()
			}
		}
		if pathRobotSelect != nil {
			if busy {
				pathRobotSelect.Disable()
			} else {
				pathRobotSelect.Enable()
			}
		}
	}

	addWaypointRow := func(xDefault, yDefault string) {
		xEntry := widget.NewEntry()
		xEntry.SetPlaceHolder("[cm]")
		xEntry.SetText(xDefault)
		yEntry := widget.NewEntry()
		yEntry.SetPlaceHolder("[cm]")
		yEntry.SetText(yDefault)

		xWrap := container.NewGridWrap(fyne.NewSize(inputFieldWidth, inputFieldHeight), xEntry)
		yWrap := container.NewGridWrap(fyne.NewSize(inputFieldWidth, inputFieldHeight), yEntry)

		row := container.NewHBox(
			widget.NewLabel("x:"), xWrap,
			widget.NewLabel("y:"), yWrap,
		)

		rowItem := waypointRow{xEntry: xEntry, yEntry: yEntry, row: row}
		waypointRows = append(waypointRows, rowItem)

		isFirstWaypoint := len(waypointRows) == 1
		if !isFirstWaypoint {
			removeBtn := widget.NewButton("✕", func() {
				if pathBusy {
					return
				}
				if len(waypointRows) <= 1 {
					return
				}

				idx := -1
				for i := range waypointRows {
					if waypointRows[i].xEntry == xEntry && waypointRows[i].yEntry == yEntry {
						idx = i
						break
					}
				}
				if idx == -1 {
					return
				}

				waypointRows = append(waypointRows[:idx], waypointRows[idx+1:]...)
				waypointList.Remove(row)
				waypointList.Refresh()
			})
			row.Add(removeBtn)
		}

		waypointList.Add(row)
		waypointList.Refresh()
	}

	syncPlannerPathFromRows := func() (pathP.Path, bool) {
		pathPlanner.Reset()
		for _, wr := range waypointRows {
			wp, err := pathP.ParseWaypoint(wr.xEntry.Text, wr.yEntry.Text)
			if err != nil {
				log.GGeneralLogger.Println("Invalid waypoint input. Integers required for x and y.")
				return pathP.Path{}, false
			}
			pathPlanner.AddWaypoint(wp)
		}
		return pathPlanner.PlannedPath(), true
	}

	addWaypointRow("", "")

	addWaypointBtn = widget.NewButton("+", func() {
		if pathBusy {
			return
		}
		addWaypointRow("", "")
	})

	runCustomPathBtn = widget.NewButton("Run custom waypoints", func() {
		if pathBusy {
			return
		}
		if getSelectedPathRobotID() == -1 {
			log.GGeneralLogger.Println("Select a robot before running custom waypoints.")
			return
		}

		customPath, ok := syncPlannerPathFromRows()
		if !ok {
			return
		}

		setPathBusy(true)
		go func(robotID int, path pathP.Path) {
			pathP.RunManualPath(chG2bCommand, robotID, path, getRobotPose, func(wp pathP.Waypoint) {
				fyne.Do(func() {
					nextTargetLabel.SetText(fmt.Sprintf("Next target: (%d, %d)", wp.X, wp.Y))
				})
			})
			fyne.Do(func() {
				nextTargetLabel.SetText("Next target: -")
				setPathBusy(false)
			})
		}(getSelectedPathRobotID(), customPath)
	})

	runSquareBtn = widget.NewButton("Run square test", func() {
		if pathBusy {
			return
		}
		if getSelectedPathRobotID() == -1 {
			log.GGeneralLogger.Println("Select a robot before running square test.")
			return
		}
		setPathBusy(true)
		go func(robotID int) {
			pathP.RunManualPath(chG2bCommand, robotID, pathP.SquareTestPath(), getRobotPose, func(wp pathP.Waypoint) {
				fyne.Do(func() {
					nextTargetLabel.SetText(fmt.Sprintf("Next target: (%d, %d)", wp.X, wp.Y))
				})
			})
			fyne.Do(func() {
				nextTargetLabel.SetText("Next target: -")
				setPathBusy(false)
			})
		}(getSelectedPathRobotID())
	})

	runVirtualPathBtn = widget.NewButton("Run virtual path", func() {
		if pathBusy {
			return
		}
		setPathBusy(true)
		go func() {
			pathP.RunAutomaticPath(chG2bCommand, pathP.VirtualDefaultPath(), getAnyRobotPose, func(wp pathP.Waypoint) {
				fyne.Do(func() {
					nextTargetLabel.SetText(fmt.Sprintf("Next target: (%d, %d)", wp.X, wp.Y))
				})
			})
			fyne.Do(func() {
				nextTargetLabel.SetText("Next target: -")
				setPathBusy(false)
			})
		}()
	})

	return container.NewVBox(
		pathRobotSelect,
		nextTargetLabel,
		container.NewHBox(widget.NewLabel("Custom waypoints"), addWaypointBtn),
		waypointList,
		runCustomPathBtn,
		widget.NewSeparator(),
		runSquareBtn,
		runVirtualPathBtn,
	), pathRobotSelect
}
