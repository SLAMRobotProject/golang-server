package widgets

import (
	"golang-server/config"
	"image/color"

	"strconv"

	"fyne.io/fyne/v2"
	"fyne.io/fyne/v2/canvas"
)

type MapAxis struct {
	XAxis, YAxis *canvas.Line
	XText, YText *canvas.Text
}

func NewMapAxis() *MapAxis {
	orangeT := color.RGBA{0xff, 0xa5, 0x00, 0x80}
	darkRed := color.RGBA{0x8b, 0x00, 0x00, 0xff}

	xAxis := canvas.NewLine(orangeT)
	xAxis.Position1 = fyne.NewPos(0, config.MapCenterY)
	xAxis.Position2 = fyne.NewPos(config.MapSize, config.MapCenterY)
	yAxis := canvas.NewLine(orangeT)
	yAxis.Position1 = fyne.NewPos(config.MapCenterX, 0)
	yAxis.Position2 = fyne.NewPos(config.MapCenterX, config.MapSize)
	xText := canvas.NewText("x="+strconv.Itoa(config.MapSize-config.MapCenterX), darkRed)
	yText := canvas.NewText("y="+strconv.Itoa(config.MapSize-config.MapCenterY), darkRed)

	return &MapAxis{xAxis, yAxis, xText, yText}
}

// Layout is called to pack all child objects into a specified size.
func (m *MapAxis) Layout(objects []fyne.CanvasObject, size fyne.Size) {
	//The map is square and centered, but we must offset the position of the lines relative to the top left corner of the container
	dx, dy := float32(0), float32(0)
	if size.Height > size.Width {
		dy += (size.Height - size.Width) / 2
	} else {
		dx += (size.Width - size.Height) / 2
	}
	currentMapSize := min(size.Height, size.Width)
	currentRatio := currentMapSize / float32(config.MapSize)

	m.XAxis.Position1 = fyne.NewPos(dx, config.MapCenterY*currentRatio+dy)
	m.XAxis.Position2 = fyne.NewPos(currentMapSize+dx, config.MapCenterY*currentRatio+dy)

	m.YAxis.Position1 = fyne.NewPos(config.MapCenterX*currentRatio+dx, dy)
	m.YAxis.Position2 = fyne.NewPos(config.MapCenterX*currentRatio+dx, currentMapSize+dy)

	m.XText.Move(fyne.NewPos(currentMapSize+dx-43, config.MapCenterY*currentRatio+dy))
	m.YText.Move(fyne.NewPos(config.MapCenterX*currentRatio+3+dx, dy+1))
}

// MinSize finds the smallest size that satisfies all the child objects.
func (m *MapAxis) MinSize(objects []fyne.CanvasObject) fyne.Size {
	minSize := fyne.NewSize(config.MapMinimumDisplaySize, config.MapMinimumDisplaySize)
	return minSize
}
