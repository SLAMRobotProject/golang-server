package widgets

import (
	"image"
	"image/color"

	"fyne.io/fyne/v2"
	"fyne.io/fyne/v2/canvas"
)

// BuildBaseMapImage creates a square map image filled with a single color.
func BuildBaseMapImage(mapSize int, fillColor color.Color) *image.RGBA {
	shape := image.Rect(0, 0, mapSize, mapSize)
	mapImage := image.NewRGBA(shape)
	for x := 0; x < mapSize; x++ {
		for y := 0; y < mapSize; y++ {
			mapImage.Set(x, y, fillColor)
		}
	}
	return mapImage
}

// BuildMapCanvas creates a configured Fyne image widget for map rendering.
func BuildMapCanvas(baseMap image.Image, minMapSize fyne.Size, smoothScale bool) *canvas.Image {
	mapCanvas := canvas.NewImageFromImage(baseMap)
	mapCanvas.FillMode = canvas.ImageFillContain
	if smoothScale {
		mapCanvas.ScaleMode = canvas.ImageScaleSmooth
	}
	mapCanvas.SetMinSize(minMapSize)
	return mapCanvas
}
