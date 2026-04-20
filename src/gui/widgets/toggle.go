package widgets

import (
	"image/color"

	"fyne.io/fyne/v2"
	"fyne.io/fyne/v2/canvas"
	"fyne.io/fyne/v2/widget"
)

const (
	toggleButtonWidth  = float32(36)
	toggleButtonHeight = float32(16)
)

// ToggleButtonState tracks the current mode and associated display values
type ToggleButtonState struct {
	ModeLabel    string
	CurrentInput fyne.CanvasObject
}

// ToggleButton is a custom toggle widget for switching between Manual and Auto modes.
type ToggleButton struct {
	widget.BaseWidget
	checked   bool
	onChanged func(bool)
	width     float32
	height    float32
}

type toggleButtonRenderer struct {
	toggle     *ToggleButton
	background *canvas.Rectangle
	knob       *canvas.Circle
	objects    []fyne.CanvasObject
}

func NewToggleButton(initial bool, onChanged func(bool)) *ToggleButton {
	toggle := &ToggleButton{checked: initial, onChanged: onChanged, width: toggleButtonWidth, height: toggleButtonHeight}
	toggle.ExtendBaseWidget(toggle)
	return toggle
}

func (m *ToggleButton) SetChecked(checked bool) {
	if m.checked == checked {
		return
	}
	m.checked = checked
	if m.onChanged != nil {
		m.onChanged(checked)
	}
	m.Refresh()
}

func (m *ToggleButton) Tapped(*fyne.PointEvent) {
	m.SetChecked(!m.checked)
}

func (m *ToggleButton) TappedSecondary(*fyne.PointEvent) {}

func (m *ToggleButton) CreateRenderer() fyne.WidgetRenderer {
	background := canvas.NewRectangle(color.NRGBA{R: 230, G: 230, B: 230, A: 255})
	background.CornerRadius = 999
	knob := canvas.NewCircle(color.White)
	knob.StrokeColor = color.NRGBA{R: 210, G: 210, B: 210, A: 255}
	knob.StrokeWidth = 1
	renderer := &toggleButtonRenderer{
		toggle:     m,
		background: background,
		knob:       knob,
		objects:    []fyne.CanvasObject{background, knob},
	}
	return renderer
}

func (r *toggleButtonRenderer) Layout(size fyne.Size) {
	padding := float32(1)
	height := r.toggle.height
	width := r.toggle.width
	knobSize := height - 2*padding

	r.background.Resize(fyne.NewSize(width, height))
	if r.toggle.checked {
		r.background.FillColor = color.NRGBA{R: 0, G: 201, B: 150, A: 255}
		r.knob.Move(fyne.NewPos(width-knobSize-padding, padding))
	} else {
		r.background.FillColor = color.NRGBA{R: 230, G: 230, B: 230, A: 255}
		r.knob.Move(fyne.NewPos(padding, padding))
	}
	r.knob.Resize(fyne.NewSize(knobSize, knobSize))
	r.background.Refresh()
	r.knob.Refresh()
}

func (r *toggleButtonRenderer) MinSize() fyne.Size {
	return fyne.NewSize(r.toggle.width, r.toggle.height)
}

func (r *toggleButtonRenderer) Refresh() {
	r.Layout(r.toggle.Size())
}

func (r *toggleButtonRenderer) BackgroundColor() color.Color {
	return color.Transparent
}

func (r *toggleButtonRenderer) Objects() []fyne.CanvasObject {
	return r.objects
}

func (r *toggleButtonRenderer) Destroy() {}
