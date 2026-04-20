package gui

import (
	"fmt"
	"golang-server/backend"
	"golang-server/config"
	"golang-server/gui/tabs"
	"golang-server/gui/widgets"
	"golang-server/types"
	"image"
	"image/color"

	"fyne.io/fyne/v2"
	"fyne.io/fyne/v2/app"
	"fyne.io/fyne/v2/canvas"
	"fyne.io/fyne/v2/container"
	"fyne.io/fyne/v2/widget"
)

var (
	green = color.RGBA{0x00, 0xff, 0x00, 0xff}
	gray  = color.RGBA{0x80, 0x80, 0x80, 0xff}
	white = color.White
	black = color.Black
)

const (
	inputFieldWidth  = float32(110)
	inputFieldHeight = float32(36)
)

type MainGUI struct {
	window fyne.Window

	backendState *backend.BackendRuntimeState

	mapImage      *image.RGBA
	mapCanvas     *canvas.Image
	slamMapCanvas *image.RGBA
	slamLCanvas   *canvas.Image
	slamGCanvas   *canvas.Image

	robotManager *widgets.MultiRobotManager

	manualLayout      *fyne.Container
	physicalInitList  *fyne.Container
	virtualInitList   *fyne.Container
	physicalInitEmpty *widget.Label
	virtualInitEmpty  *widget.Label

	linesContainer *fyne.Container
	activeLines    []*canvas.Line

	selectedSlamRobotID int
	slamRobotSelect     *widget.Select
	slamPoseLabel       *widget.Label
	slamRobotOptions    []string

	selectedGlobalSlamRobotID int
	slamGRobotSelect          *widget.Select
	slamGRobotOptions         []string
	slamGShowMatches          bool

	selectedManualRobotID int
	manualRobotSelect     *widget.Select
	manualRobotOptions    []string

	selectedPathRobotID int
	pathRobotSelect     *widget.Select
}

func NewMainGUI(chG2bCommand chan<- types.Command, backendState *backend.BackendRuntimeState) *MainGUI {
	gui := &MainGUI{
		backendState:              backendState,
		selectedSlamRobotID:       -1,
		selectedGlobalSlamRobotID: -1,
		selectedManualRobotID:     -1,
		selectedPathRobotID:       -1,
	}

	// Create Fyne application and main window
	app := app.New()
	gui.window = app.NewWindow("Canvas")
	gui.window.Resize(fyne.NewSize(config.WindowBreadth, config.WindowHeight))

	minMapSize := fyne.NewSize(config.MapMinimumDisplaySize, config.MapMinimumDisplaySize)

	// ===== Initialize Robot Tracking =====
	gui.robotManager = widgets.NewMultiRobotManager()
	gui.linesContainer = container.NewWithoutLayout()

	// ===== Initialize Main Tab (Inputs + Map) =====
	axis := widgets.NewMapAxis()
	axisContainer := container.New(axis, axis.XAxis, axis.YAxis, axis.XText, axis.YText)
	mainTab := tabs.BuildMainTab(
		chG2bCommand,
		func() int { return gui.selectedManualRobotID },
		func() int { return gui.selectedPathRobotID },
		func(s string) { fmt.Sscanf(s, "Robot %d", &gui.selectedManualRobotID) },
		func(s string) { fmt.Sscanf(s, "Robot %d", &gui.selectedPathRobotID) },
		gui.backendState.GetRobotPose,
		gui.backendState.GetAnyRobotPose,
		config.MapSize,
		gray,
		minMapSize,
		axisContainer,
		gui.linesContainer,
		gui.robotManager.Container(),
		false,
	)
	gui.manualLayout = mainTab.ManualLayout
	gui.physicalInitList = mainTab.PhysicalInitList
	gui.virtualInitList = mainTab.VirtualInitList
	gui.physicalInitEmpty = mainTab.PhysicalInitEmpty
	gui.virtualInitEmpty = mainTab.VirtualInitEmpty
	gui.manualRobotSelect = mainTab.ManualRobotSelect
	gui.pathRobotSelect = mainTab.PathRobotSelect
	gui.mapImage = mainTab.MapImage
	gui.mapCanvas = mainTab.MapCanvas

	// ===== Initialize Local and Global Map Tabs =====
	gui.slamMapCanvas = widgets.BuildBaseMapImage(config.MapSize, gray)

	localMapTab := tabs.BuildLocalMapTab(gui.slamMapCanvas, minMapSize, func(s string) {
		fmt.Sscanf(s, "Robot %d", &gui.selectedSlamRobotID)
	})
	gui.slamLCanvas = localMapTab.Canvas
	gui.slamRobotSelect = localMapTab.RobotSelect
	gui.slamPoseLabel = localMapTab.PoseLabel

	globalMapTab := tabs.BuildGlobalMapTab(gui.slamMapCanvas, minMapSize, func(s string) {
		fmt.Sscanf(s, "Robot %d", &gui.selectedGlobalSlamRobotID)
	}, func(v bool) {
		gui.slamGShowMatches = v
	})
	gui.slamGCanvas = globalMapTab.Canvas
	gui.slamGRobotSelect = globalMapTab.RobotSelect

	// ===== Assemble Final Window with Tabs =====
	mainTabs := container.NewAppTabs(
		container.NewTabItem("Main", mainTab.Content),
		container.NewTabItem("Local Map", localMapTab.Content),
		container.NewTabItem("Global Map", globalMapTab.Content),
	)

	gui.window.SetContent(mainTabs)
	return gui
}

func (gui *MainGUI) ShowAndRun() {
	gui.window.ShowAndRun()
}
