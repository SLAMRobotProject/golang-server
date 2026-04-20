package tabs

import (
	"golang-server/gui/widgets"
	"image"

	"fyne.io/fyne/v2"
	"fyne.io/fyne/v2/canvas"
	"fyne.io/fyne/v2/container"
	"fyne.io/fyne/v2/widget"
)

type GlobalMapTab struct {
	Canvas      *canvas.Image
	RobotSelect *widget.Select
	Content     *fyne.Container
}

// BuildGlobalMapTab creates the global map tab UI and returns the created widgets.
func BuildGlobalMapTab(baseMap image.Image, minMapSize fyne.Size, onRobotSelected func(string), onShowMatchesChanged func(bool)) *GlobalMapTab {
	globalMapCanvas := widgets.BuildMapCanvas(baseMap, minMapSize, true)

	robotSelect := widget.NewSelect([]string{}, onRobotSelected)
	robotSelect.PlaceHolder = "Select robot"
	showMatches := widget.NewCheck("Show matches", onShowMatchesChanged)

	controls := container.NewHBox(robotSelect, showMatches)
	content := container.NewBorder(controls, nil, nil, nil, globalMapCanvas)

	return &GlobalMapTab{
		Canvas:      globalMapCanvas,
		RobotSelect: robotSelect,
		Content:     content,
	}
}
