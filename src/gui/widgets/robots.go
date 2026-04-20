package widgets

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

var (
	red   = color.RGBA{0xff, 0x00, 0x00, 0xff}
	green = color.RGBA{0x00, 0xff, 0x00, 0xff}
	blue  = color.RGBA{0x00, 0x00, 0xff, 0xff}
)

////////////////////////////////
// Single robot
////////////////////////////////

type robotLayout struct {
	lines           [3]*canvas.Line
	poseLabel       *canvas.Text
	currentRatio    float32
	currentRotation float64
	baseLines       [3][2]fyne.Position // Original unscaled, unrotated lines
	baseWidths      [3]float32          // Original unscaled widths
}

func initRobotLayout(lines [3]*canvas.Line) *robotLayout {
	poseLabel := &canvas.Text{Text: "(0, 0, 0)", Alignment: fyne.TextAlignLeading, TextSize: 8, Color: red}
	poseLabel.Move(fyne.NewPos(0, -20))

	var baseLines [3][2]fyne.Position
	var baseWidths [3]float32
	for i, l := range lines {
		baseLines[i][0] = l.Position1
		baseLines[i][1] = l.Position2
		baseWidths[i] = l.StrokeWidth
	}

	return &robotLayout{
		lines:           lines,
		poseLabel:       poseLabel,
		currentRatio:    1,
		currentRotation: 90,
		baseLines:       baseLines,
		baseWidths:      baseWidths,
	}
}

func (m *robotLayout) updateGeometry() {
	// The original geometry is drawn pointing UP (which means -Y in fyne).
	// When currentRotation is 90, it expects no rotation relative to the GUI.
	// As currentRotation deviates from 90, we rotate the lines.
	absRot := -(m.currentRotation - 90)

	for i, l := range m.lines {
		p1, p2 := m.baseLines[i][0], m.baseLines[i][1]

		x1, y1 := utilities.Rotate(float64(p1.X), float64(p1.Y), absRot)
		x2, y2 := utilities.Rotate(float64(p2.X), float64(p2.Y), absRot)

		l.Position1.X, l.Position1.Y = float32(x1)*m.currentRatio, float32(y1)*m.currentRatio
		l.Position2.X, l.Position2.Y = float32(x2)*m.currentRatio, float32(y2)*m.currentRatio
		l.StrokeWidth = m.baseWidths[i] * m.currentRatio
	}
}

// Layout is called to pack all child objects into a specified size.
func (m *robotLayout) Layout(objects []fyne.CanvasObject, size fyne.Size) {
	m.currentRatio = fyne.Min(size.Height, size.Width) / config.MapSize
	m.updateGeometry()
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
		m.currentRotation = thetaDeg
		m.updateGeometry()
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

type MultiRobotManager struct {
	layout    *multiRobotLayout
	container *fyne.Container
}

func (m *MultiRobotManager) Container() *fyne.Container {
	return m.container
}

func (m *MultiRobotManager) Rotate(index int, theta float64) {
	m.layout.robots[index].Rotate(theta)
}

func (m *MultiRobotManager) Move(index int, position fyne.Position) {
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

func (m *MultiRobotManager) SetPoseLabel(index int, x, y, theta int) {
	robot := m.layout.robots[index]
	robot.poseLabel.Text = fmt.Sprintf("(%d, %d, %d)", x, y, theta)
	robot.poseLabel.Refresh()
}

func (m *MultiRobotManager) AddRobot(id int) {
	robot := initRobotGui()

	m.layout.robots = append(m.layout.robots, robot)

	IdLabel := &canvas.Text{Text: strconv.Itoa(id), Alignment: fyne.TextAlignCenter, TextSize: 8, Color: green}

	robotContainer := container.New(robot, robot.lines[0], robot.lines[1], robot.lines[2], robot.poseLabel, IdLabel)
	m.container.Add(robotContainer)
}

func (m *MultiRobotManager) NumRobots() int {
	return len(m.layout.robots)
}

func NewMultiRobotManager() *MultiRobotManager {
	layout := initMultiRobotLayout()
	container := container.New(layout)
	return &MultiRobotManager{layout, container}
}
