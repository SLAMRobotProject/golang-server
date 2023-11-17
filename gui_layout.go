package main

import (
	"math"
	"strconv"

	"fyne.io/fyne/v2"
	"fyne.io/fyne/v2/canvas"
	"fyne.io/fyne/v2/container"
)

////////////////////////////////
// Single robot
///////////////////////////////

//onst wheel_distance = 16 // the length of the line symbolizing the wheels
// const wheel_breadth = 3
// const robot_lenth = 20
// const robot_breadth = 10

type robotLayout struct {
	lines            [3]*canvas.Line
	current_ratio    float32
	current_rotation float64
}

func NewRobotLayout(lines [3]*canvas.Line) *robotLayout {
	return &robotLayout{lines, 1, 0}
}

// Layout is called to pack all child objects into a specified size.
func (m *robotLayout) Layout(objects []fyne.CanvasObject, size fyne.Size) {
	var ratio float32 = fyne.Min(size.Height, size.Width) / map_size
	adjustment := ratio / m.current_ratio
	for _, line := range m.lines {
		line.Position1.X *= adjustment
		line.Position1.Y *= adjustment
		line.Position2.X *= adjustment
		line.Position2.Y *= adjustment
		line.StrokeWidth *= adjustment
	}
	m.current_ratio = ratio
}

// MinSize finds the smallest size that satisfies all the child objects.
func (m *robotLayout) MinSize(objects []fyne.CanvasObject) fyne.Size {
	minSize := fyne.NewSize(0, 0)
	for _, child := range objects {
		minSize = minSize.Max(child.Size())
	}
	return minSize
}

func (m *robotLayout) Rotate(theta_deg float64) {
	if theta_deg != m.current_rotation {
		diff_theta_rad := (theta_deg - m.current_rotation) * math.Pi / 180
		for _, line := range m.lines {
			x_temp := line.Position1.X //To avoid overwriting the value before it is used
			line.Position1.X = float32(float64(line.Position1.X)*math.Cos(diff_theta_rad) - float64(line.Position1.Y)*math.Sin(diff_theta_rad))
			line.Position1.Y = float32(float64(x_temp)*math.Sin(diff_theta_rad) + float64(line.Position1.Y)*math.Cos(diff_theta_rad))

			x_temp = line.Position2.X
			line.Position2.X = float32(float64(line.Position2.X)*math.Cos(diff_theta_rad) - float64(line.Position2.Y)*math.Sin(diff_theta_rad))
			line.Position2.Y = float32(float64(x_temp)*math.Sin(diff_theta_rad) + float64(line.Position2.Y)*math.Cos(diff_theta_rad))
		}
		m.current_rotation = theta_deg
	}
}

func robot_init() *robotLayout {
	main_body := line_init(BLUE, fyne.NewPos(0, -10), fyne.NewPos(0, 10), 10)
	wheels := line_init(BLUE, fyne.NewPos(-8, 0), fyne.NewPos(8, 0), 5)
	direction_indicator := line_init(RED, fyne.NewPos(0, 0), fyne.NewPos(0, -9), 3)
	robot_lines := [3]*canvas.Line{main_body, direction_indicator, wheels}
	robot_handle := NewRobotLayout(robot_lines)
	return robot_handle
}

///////////////////////////////
// Multiple Rotbots
///////////////////////////////

type multiRobotLayout struct {
	robots       []*robotLayout
	current_size fyne.Size
}

func NewMultiRobotLayout() *multiRobotLayout {
	return &multiRobotLayout{nil, fyne.NewSize(map_size, map_size)}
}

// Layout is called to pack all child objects into a specified size.
func (m *multiRobotLayout) Layout(objects []fyne.CanvasObject, new_size fyne.Size) {

	// The map is a square and centered, but we must adjust the position of the robots relative to the top left corner
	dx, dy := float32(0), float32(0)
	if new_size.Height > new_size.Width {
		dy = ((new_size.Height - m.current_size.Height) - (new_size.Width - m.current_size.Width)) / 2
	} else {
		dx = ((new_size.Width - m.current_size.Width) - (new_size.Height - m.current_size.Height)) / 2
	}

	ratio := fyne.Min(new_size.Height, new_size.Width) / fyne.Min(m.current_size.Height, m.current_size.Width)

	for _, child := range objects {
		//TODO: vudere å fjerne dette fordi det gjøres av seg selv når kartet tegnes på nytt.
		// TODO: blir faktisk ikke riktig engang hvis man ikke også tar current offset.....
		// LØSNING? Kan egentlig fjerne alt utenom resize large current_size også da.
		scale_position := fyne.NewPos(child.Position().X*ratio, child.Position().Y*ratio)
		offset_position := scale_position.AddXY(dx, dy)
		child.Move(offset_position)
		child.Resize(new_size)
	}
	m.current_size = new_size
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
	multi_robot_layout    *multiRobotLayout
	multi_robot_container *fyne.Container
}

func (m *multiRobotHandle) Rotate(index int, theta float64) {
	m.multi_robot_layout.robots[index].Rotate(theta)
}

func (m *multiRobotHandle) Move(index int, position fyne.Position) {
	current_size := m.multi_robot_layout.current_size
	ratio := fyne.Min(current_size.Height, current_size.Width) / map_size
	scale_position := fyne.NewPos(float32(position.X)*ratio, float32(position.Y)*ratio)

	//The map is square and centered, but we must offset the position of the robots relative to the top left corner
	dx, dy := float32(map_center_x)*ratio, float32(map_center_y)*ratio
	if current_size.Height > current_size.Width {
		dy += (current_size.Height - current_size.Width) / 2
	} else {
		dx += (current_size.Width - current_size.Height) / 2
	}

	m.multi_robot_container.Objects[index].Move(scale_position.AddXY(dx, dy))
}

func (m *multiRobotHandle) AddRobot(id int) {
	robot := robot_init()

	m.multi_robot_layout.robots = append(m.multi_robot_layout.robots, robot)

	id_label := &canvas.Text{Text: strconv.Itoa(id), Alignment: fyne.TextAlignCenter, TextSize: 8, Color: GREEN}

	robot_container := container.New(robot, robot.lines[0], robot.lines[1], robot.lines[2], id_label)
	m.multi_robot_container.Add(robot_container)
}

func (m *multiRobotHandle) NumRobots() int {
	return len(m.multi_robot_layout.robots)
}

func NewMultiRobotHandle() *multiRobotHandle {
	multiRobot_layout := NewMultiRobotLayout()
	multiRobot_container := container.New(multiRobot_layout)
	return &multiRobotHandle{multiRobot_layout, multiRobot_container}
}
