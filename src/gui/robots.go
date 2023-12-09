package gui

import (
	"fmt"
	"golang-server/config"
	"golang-server/utilities"
	"image/color"
	"strconv"

	"fyne.io/fyne/v2"
	"fyne.io/fyne/v2/canvas"
	"fyne.io/fyne/v2/container"
)

////////////////////////////////
// Single robot
////////////////////////////////

type robotLayout struct {
	lines           [3]*canvas.Line
	poseLabel       *canvas.Text
	currentRatio    float32
	currentRotation float64
}

func initRobotLayout(lines [3]*canvas.Line) *robotLayout {
	poseLabel := &canvas.Text{Text: "(0, 0, 0)", Alignment: fyne.TextAlignLeading, TextSize: 8, Color: red}
	poseLabel.Move(fyne.NewPos(0, -20))
	return &robotLayout{lines, poseLabel, 1, 90}
}

// Layout is called to pack all child objects into a specified size.
func (m *robotLayout) Layout(objects []fyne.CanvasObject, size fyne.Size) {
	var ratio float32 = fyne.Min(size.Height, size.Width) / config.MapSize
	adjustment := ratio / m.currentRatio
	for _, line := range m.lines {
		line.Position1.X *= adjustment
		line.Position1.Y *= adjustment
		line.Position2.X *= adjustment
		line.Position2.Y *= adjustment
		line.StrokeWidth *= adjustment
	}
	m.currentRatio = ratio
}

// MinSize finds the smallest size that satisfies all the child objects.
func (m *robotLayout) MinSize(objects []fyne.CanvasObject) fyne.Size {
	minSize := fyne.NewSize(0, 0)
	for _, child := range objects {
		minSize = minSize.Max(child.Size())
	}
	return minSize
}

func (m *robotLayout) Rotate(thetaDeg float64) {
	if thetaDeg != m.currentRotation {
		diffTheta := -(thetaDeg - m.currentRotation) //negative because the rotation is clockwise (flipped y-axis)
		for _, line := range m.lines {
			x, y := utilities.Rotate(float64(line.Position1.X), float64(line.Position1.Y), float64(diffTheta))
			line.Position1.X, line.Position1.Y = float32(x), float32(y)

			x, y = utilities.Rotate(float64(line.Position2.X), float64(line.Position2.Y), float64(diffTheta))
			line.Position2.X, line.Position2.Y = float32(x), float32(y)
		}
		m.currentRotation = thetaDeg
	}
}

func initLine(
	color color.Color,
	pos1 fyne.Position,
	pos2 fyne.Position,
	strokeWidth float32,
) *canvas.Line {
	l := canvas.NewLine(color)
	l.Position1 = pos1
	l.Position2 = pos2
	l.StrokeWidth = strokeWidth
	return l
}

func initRobotGui() *robotLayout {
	mainBody := initLine(blue, fyne.NewPos(0, -10), fyne.NewPos(0, 10), 13)
	wheels := initLine(blue, fyne.NewPos(-10, 0), fyne.NewPos(10, 0), 6.5)
	directionIndicator := initLine(red, fyne.NewPos(0, 0), fyne.NewPos(0, -9), 3)
	robotLines := [3]*canvas.Line{mainBody, directionIndicator, wheels}
	robotHandle := initRobotLayout(robotLines)
	return robotHandle
}

///////////////////////////////
// Multiple Rotbots
///////////////////////////////

type multiRobotLayout struct {
	robots      []*robotLayout
	currentSize fyne.Size
}

func initMultiRobotLayout() *multiRobotLayout {
	return &multiRobotLayout{nil, fyne.NewSize(config.MapSize, config.MapSize)}
}

// Layout is called to pack all child objects into a specified size.
func (m *multiRobotLayout) Layout(objects []fyne.CanvasObject, newSize fyne.Size) {
	for _, child := range objects {
		child.Resize(newSize)
	}
	m.currentSize = newSize
}

// MinSize finds the smallest size that satisfies all the child objects.
func (m *multiRobotLayout) MinSize(objects []fyne.CanvasObject) fyne.Size {
	minSize := fyne.NewSize(0, 0)
	for _, child := range objects {
		minSize = minSize.Max(child.MinSize())
	}
	return minSize
}

///////////////////////////////
// Multiple Rotbots Handle
///////////////////////////////

type multiRobotHandle struct {
	layout    *multiRobotLayout
	container *fyne.Container
}

func (m *multiRobotHandle) Rotate(index int, theta float64) {
	m.layout.robots[index].Rotate(theta)
}

func (m *multiRobotHandle) Move(index int, position fyne.Position) {
	currentSize := m.layout.currentSize
	ratio := fyne.Min(currentSize.Height, currentSize.Width) / config.MapSize
	scalePosition := fyne.NewPos(float32(position.X)*ratio, float32(position.Y)*ratio)

	//The map is square and centered, but we must offset the position of the robots relative to the top left corner
	dx, dy := float32(config.MapCenterX)*ratio, float32(config.MapCenterY)*ratio
	if currentSize.Height > currentSize.Width {
		dy += (currentSize.Height - currentSize.Width) / 2
	} else {
		dx += (currentSize.Width - currentSize.Height) / 2
	}

	m.container.Objects[index].Move(scalePosition.AddXY(dx, dy))
}

func (m *multiRobotHandle) setPoseLabel(index int, x, y, theta int) {
	robot := m.layout.robots[index]
	robot.poseLabel.Text = fmt.Sprintf("(%d, %d, %d)", x, y, theta)
	robot.poseLabel.Refresh()
}

func (m *multiRobotHandle) AddRobot(id int) {
	robot := initRobotGui()

	m.layout.robots = append(m.layout.robots, robot)

	IdLabel := &canvas.Text{Text: strconv.Itoa(id), Alignment: fyne.TextAlignCenter, TextSize: 8, Color: green}

	robotContainer := container.New(robot, robot.lines[0], robot.lines[1], robot.lines[2], robot.poseLabel, IdLabel)
	m.container.Add(robotContainer)
}

func (m *multiRobotHandle) NumRobots() int {
	return len(m.layout.robots)
}

func initMultiRobotHandle() *multiRobotHandle {
	layout := initMultiRobotLayout()
	container := container.New(layout)
	return &multiRobotHandle{layout, container}
}
