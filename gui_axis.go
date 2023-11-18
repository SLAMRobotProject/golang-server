package main

import (
	"strconv"

	"fyne.io/fyne/v2"
	"fyne.io/fyne/v2/canvas"
)

type mapAxis struct {
	xAxis, yAxis *canvas.Line
	xText, yText *canvas.Text
}

func init_mapAxis() *mapAxis {
	xAxis := canvas.NewLine(ORANGE_T)
	xAxis.Position1 = fyne.NewPos(0, MAP_CENTER_Y)
	xAxis.Position2 = fyne.NewPos(MAP_SIZE, MAP_CENTER_Y)
	yAxis := canvas.NewLine(ORANGE_T)
	yAxis.Position1 = fyne.NewPos(MAP_CENTER_X, 0)
	yAxis.Position2 = fyne.NewPos(MAP_CENTER_X, MAP_SIZE)
	xText := canvas.NewText("x="+strconv.Itoa(MAP_SIZE-MAP_CENTER_X), DARKRED)
	yText := canvas.NewText("y="+strconv.Itoa(MAP_SIZE-MAP_CENTER_Y), DARKRED)

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
	current_ratio := current_map_size / float32(MAP_SIZE)

	m.xAxis.Position1 = fyne.NewPos(dx, MAP_CENTER_Y*current_ratio+dy)
	m.xAxis.Position2 = fyne.NewPos(current_map_size+dx, MAP_CENTER_Y*current_ratio+dy)

	m.yAxis.Position1 = fyne.NewPos(MAP_CENTER_X*current_ratio+dx, dy)
	m.yAxis.Position2 = fyne.NewPos(MAP_CENTER_X*current_ratio+dx, current_map_size+dy)

	m.xText.Move(fyne.NewPos(current_map_size+dx-43, MAP_CENTER_Y*current_ratio+dy))
	m.yText.Move(fyne.NewPos(MAP_CENTER_X*current_ratio+3+dx, dy+1))
}

// MinSize finds the smallest size that satisfies all the child objects.
func (m *mapAxis) MinSize(objects []fyne.CanvasObject) fyne.Size {
	minSize := fyne.NewSize(MAP_MINIMUM_DISPLAY_SIZE, MAP_MINIMUM_DISPLAY_SIZE)
	return minSize
}
