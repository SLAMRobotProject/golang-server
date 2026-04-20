package gui

import (
	"image"

	"golang-server/config"
	"golang-server/gui/widgets"
	"golang-server/types"

	"fyne.io/fyne/v2"
	"fyne.io/fyne/v2/canvas"
)

func (gui *MainGUI) redrawLines(lines [][2]types.Point) {
	needed := len(lines)
	current := len(gui.activeLines)

	if needed > current {
		for i := 0; i < (needed - current); i++ {
			newLine := canvas.NewLine(green)
			newLine.StrokeWidth = 2
			gui.activeLines = append(gui.activeLines, newLine)
			gui.linesContainer.Add(newLine)
		}
	}

	if current > 100 && current > needed*2 {
		for i := needed; i < current; i++ {
			gui.linesContainer.Remove(gui.activeLines[i])
		}
		gui.activeLines = gui.activeLines[:needed]
		current = needed
	}

	offsetX := float32(config.MapCenterX)
	offsetY := float32(config.MapCenterY)

	for i := 0; i < len(gui.activeLines); i++ {
		lineObj := gui.activeLines[i]
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
	gui.linesContainer.Refresh()
}

func (gui *MainGUI) redrawRobots(robotManager *widgets.MultiRobotManager, backendMultiRobot []types.RobotState, backendId2index map[int]int) {
	backendNumRobots := len(backendMultiRobot)
	if backendNumRobots > robotManager.NumRobots() {
		for i := robotManager.NumRobots(); i < backendNumRobots; i++ {
			//find ID
			for id, index := range backendId2index {
				if index == i {
					robotManager.AddRobot(id)
					break
				}
			}
		}
	}
	for i := 0; i < backendNumRobots; i++ {
		robotManager.SetPoseLabel(i, backendMultiRobot[i].X, backendMultiRobot[i].Y, backendMultiRobot[i].Theta)
		robotManager.Move(i, fyne.NewPos(float32(backendMultiRobot[i].X), -float32(backendMultiRobot[i].Y)))
		robotManager.Rotate(i, float64(backendMultiRobot[i].Theta))
	}
}

func (gui *MainGUI) redrawMap(mapImage *image.RGBA, newOpen [][2]int, newObstacle [][2]int) {
	for _, point := range newOpen {
		mapImage.Set(point[0], point[1], white)
	}
	for _, point := range newObstacle {
		mapImage.Set(point[0], point[1], black)
	}
}
