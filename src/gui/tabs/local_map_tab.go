package tabs

import (
	"golang-server/gui/widgets"
	"image"

	"fyne.io/fyne/v2"
	"fyne.io/fyne/v2/canvas"
	"fyne.io/fyne/v2/container"
	"fyne.io/fyne/v2/widget"
)

type LocalMapTab struct {
	Canvas      *canvas.Image
	RobotSelect *widget.Select
	PoseLabel   *widget.Label
	Content     *fyne.Container
}

// BuildLocalMapTab creates the local map tab UI and returns the created widgets.
func BuildLocalMapTab(baseMap image.Image, minMapSize fyne.Size, onRobotSelected func(string)) *LocalMapTab {
	localMapCanvas := widgets.BuildMapCanvas(baseMap, minMapSize, true)

	robotSelect := widget.NewSelect([]string{}, onRobotSelected)
	robotSelect.PlaceHolder = "Select robot"
	poseLabel := widget.NewLabel("X: — Y: — θ: —  Submaps: —")

	controls := container.NewHBox(robotSelect, poseLabel)
	content := container.NewBorder(controls, nil, nil, nil, localMapCanvas)

	return &LocalMapTab{
		Canvas:      localMapCanvas,
		RobotSelect: robotSelect,
		PoseLabel:   poseLabel,
		Content:     content,
	}
}
