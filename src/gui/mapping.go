package gui

import(
	//"golang-server/types"

	"image"
	"image/color"
	//"image/png"
	//"image/draw"
	//"os"

)
var img=image.NewRGBA(image.Rect(0,0,100,100))
var col color.Color

// HLine draws a horizontal line
func HLine(x1, y, x2 int) {
    for ; x1 <= x2; x1++ {
        img.Set(x1, y, col)
    }
}

// VLine draws a veritcal line
func VLine(x, y1, y2 int) {
    for ; y1 <= y2; y1++ {
        img.Set(x, y1, col)
    }
}

// Rect draws a rectangle utilizing HLine() and VLine()
func Rect(x1, y1, x2, y2 int) {
    HLine(x1, y1, x2)
    HLine(x1, y2, x2)
    VLine(x1, y1, y2)
    VLine(x2, y1, y2)
}

/*func ThreadMapping() () {


	col=color.RGBA{0,255,185,255}
	//col=color.RGBA{0,255,0,255}
	rec:=image.Rect(20,20,50,100)
	draw.Draw(img,rec,&image.Uniform{col},image.ZP,draw.Src)
	rec=image.Rect(20,20,50,21)
	col=color.RGBA{0,255,0,255}
	draw.Draw(img,rec,&image.Uniform{col},image.ZP,draw.Src)
	HLine(20,20,80)


}*/