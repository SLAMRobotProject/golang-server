package main

import (
	"image"
	"image/color"
	"log"
	"math"
	"strconv"

	"fyne.io/fyne/v2"
	"fyne.io/fyne/v2/app"
	"fyne.io/fyne/v2/canvas"
	"fyne.io/fyne/v2/container"
	"fyne.io/fyne/v2/widget"
)

func GUI(
	ch_publish chan<- [2]int16,
	ch_receive <-chan adv_msg,
) (fyne.Window, *canvas.Image, *image.RGBA, *canvas.Circle, *fyne.Container) {
	a := app.New()
	w := a.NewWindow("Canvas")
	w.Resize(fyne.NewSize(400, 400))

	img_shape := image.Rect(0, 0, 400, 400)

	img_init := image.NewRGBA(img_shape)

	for x := 0; x < 400; x++ {
		for y := 0; y < 400; y++ {
			img_init.Set(x, y, color.RGBA{0x80, 0x80, 0x80, 0xff}) //Gray
		}
	}
	//img_init = draw_circle_lines(img_init) //used for testing bresenham line algorithm

	map_img := canvas.NewImageFromImage(img_init)
	map_img.FillMode = canvas.ImageFillContain
	map_img.SetMinSize(fyne.NewSize(400, 400))

	input_x := widget.NewEntry()
	input_x.SetPlaceHolder("x")
	input_y := widget.NewEntry()
	input_y.SetPlaceHolder("y")
	publish_container := container.NewVBox(input_x, input_y, widget.NewButton("Publish", func() {
		log.Println("Content was:", input_x.Text, ", ", input_y.Text)
		x, _ := strconv.Atoi(input_x.Text)
		y, _ := strconv.Atoi(input_y.Text)
		ch_publish <- [2]int16{int16(x), int16(y)}
	}))

	robot := canvas.NewCircle(color.Black)
	robot.FillColor = color.RGBA{0x00, 0xff, 0x00, 0xff}

	map_with_robot := container.NewStack(map_img, robot)

	grid := container.NewHSplit(publish_container, map_with_robot)

	w.SetContent(grid)

	return w, map_img, img_init, robot, map_with_robot
}

func bresenham_algorithm(x0, y0, x1, y1 int) [][]int {
	dx := math.Abs(float64(x1 - x0))
	//sx = x0 < x1 ? 1 : -1
	sx := 1
	if x0 > x1 {
		sx = -1
	}
	dy := -math.Abs(float64(y1 - y0))
	//sy = y0 < y1 ? 1 : -1
	sy := 1
	if y0 > y1 {
		sy = -1
	}

	err := dx + dy
	points := make([][]int, 2)

	for {
		points[0] = append(points[0], x0)
		points[1] = append(points[1], y0)
		if x0 == x1 && y0 == y1 {
			break
		}
		e2 := 2 * err
		if e2 >= dy {
			if x0 == x1 {
				break
			}
			err = err + dy
			x0 = x0 + sx
		}
		if e2 <= dx {
			if y0 == y1 {
				break
			}
			err = err + dx
			y0 = y0 + sy
		}
	}
	return points
}

func draw_line(img *image.RGBA, x0, y0, x1, y1 int) *image.RGBA {
	obstruction := false
	line_abs_length := math.Sqrt(math.Pow(float64(x0-x1), 2) + math.Pow(float64(y0-y1), 2))
	line_max_length := float64(30)
	if line_abs_length > line_max_length {
		how_much := line_max_length / line_abs_length
		x1 = x0 + int(how_much*float64(x1-x0))
		y1 = y0 + int(how_much*float64(y1-y0))
	} else {
		obstruction = true
	}

	points := bresenham_algorithm(x0, y0, x1, y1)
	for i := 0; i < len(points[0]); i++ {
		img.Set(int(points[0][i]), int(points[1][i]), color.White)
	}
	if obstruction {
		img.Set(x1, y1, color.Black)
	}
	return img
}

//the following functions will be moved or removed in a following cleanup, they were used for testing.

// func draw_points(img *image.RGBA, points [][]int) *image.RGBA {
// 	for i := 0; i < len(points[0]); i++ {
// 		img.Set(int(points[0][i]), int(points[1][i]), color.White)
// 	}
// 	return img
// }

// func circle_points(center_x, center_y, radius int) [][]int {
// 	points := make([][]int, 2)
// 	for i := 0; i < 200; i++ {
// 		x := center_x + int(float64(radius)*(math.Cos(2*math.Pi*float64(i)/float64(200))))
// 		y := center_y + int(float64(radius)*(math.Sin(2*math.Pi*float64(i)/float64(200))))
// 		points[0] = append(points[0], x)
// 		points[1] = append(points[1], y)
// 	}
// 	return points
// }

// func draw_circle_lines(img *image.RGBA) *image.RGBA {
// 	center_x := 100
// 	center_y := 100
// 	circlePoints := circle_points(center_x, center_y, 30)
// 	for i := 0; i < len(circlePoints[0]); i++ {
// 		linePoints := bresenham_algorithm(center_x, center_y, circlePoints[0][i], circlePoints[1][i])
// 		img = draw_points(img, linePoints)
// 	}
// 	return img
// }
