package gui

import (
	"fmt"
	"strconv"

	"golang-server/types"

	"fyne.io/fyne/v2"
	"fyne.io/fyne/v2/container"
	"fyne.io/fyne/v2/widget"
)

func (gui *MainGUI) StartUpdateLoop(
	chG2bCommand chan<- types.Command,
	chG2bRobotInit chan<- [4]int,
	chB2gRobotPendingInit <-chan int,
	chB2gUpdate <-chan types.UpdateGui,
	chB2gVirtualPending <-chan int,
) {
	_ = chG2bCommand
	pendingPhysical := map[int]*fyne.Container{}
	pendingVirtual := map[int]*fyne.Container{}

	updateInitEmptyState := func() {
		if len(gui.physicalInitList.Objects) == 0 {
			gui.physicalInitEmpty.Show()
		} else {
			gui.physicalInitEmpty.Hide()
		}
		if len(gui.virtualInitList.Objects) == 0 {
			gui.virtualInitEmpty.Show()
		} else {
			gui.virtualInitEmpty.Hide()
		}
		gui.physicalInitList.Refresh()
		gui.virtualInitList.Refresh()
	}

	for {
		select {
		case partialState := <-chB2gUpdate:
			fyne.Do(func() {
				gui.applyBackendGuiUpdate(partialState)
			})

		case idPending := <-chB2gRobotPendingInit:
			fyne.Do(func() {
				if _, exists := pendingPhysical[idPending]; exists {
					return
				}
				inSession := false
				actionButton := widget.NewButton("Init", nil)
				actionButton.OnTapped = func() {
					if inSession {
						inSession = false
						actionButton.SetText("Init")
						actionButton.Refresh()
						return
					}
					x, y, theta := 0, 0, 0
					chG2bRobotInit <- [4]int{idPending, x, y, theta}
					gui.addManualRobotOption(idPending)
					inSession = true
					actionButton.SetText("Destruct")
					actionButton.Refresh()
				}
				row := container.NewHBox(
					widget.NewLabel("NRF-"+strconv.Itoa(idPending)),
					actionButton,
				)
				pendingPhysical[idPending] = row
				gui.physicalInitList.Add(row)
				updateInitEmptyState()
			})
		case id := <-chB2gVirtualPending:
			fyne.Do(func() {
				if _, exists := pendingVirtual[id]; exists {
					return
				}
				inSession := false
				actionButton := widget.NewButton("Init", nil)
				actionButton.OnTapped = func() {
					if inSession {
						inSession = false
						actionButton.SetText("Init")
						actionButton.Refresh()
						return
					}
					x, y, theta := 0, 0, 0
					chG2bRobotInit <- [4]int{id, x, y, theta}
					gui.addManualRobotOption(id)
					inSession = true
					actionButton.SetText("Destruct")
					actionButton.Refresh()
				}
				row := container.NewHBox(
					widget.NewLabel(fmt.Sprintf("Virtual-%d", id)),
					actionButton,
				)
				pendingVirtual[id] = row
				gui.virtualInitList.Add(row)
				updateInitEmptyState()
			})

		}
	}
}

func (gui *MainGUI) applyBackendGuiUpdate(partialState types.UpdateGui) {
	gui.updateSlamRobotSelector(partialState.Id2index)
	gui.updateManualRobotSelector(partialState.Id2index)
	gui.updateLocalSlamView(partialState)
	gui.updateGlobalSlamView(partialState)
	gui.updateMainMapAndOverlays(partialState)
}

func (gui *MainGUI) updateLocalSlamView(partialState types.UpdateGui) {
	selID := gui.selectedSlamRobotID
	if img, ok := partialState.SlamMapImgs[selID]; ok && img != nil {
		gui.slamLCanvas.Image = img
		gui.slamLCanvas.Refresh()
	}

	if selID < 0 {
		return
	}
	if idx, ok := partialState.Id2index[selID]; ok && idx < len(partialState.MultiRobot) {
		r := partialState.MultiRobot[idx]
		n := partialState.SlamSubmapCount[selID]
		gui.slamPoseLabel.SetText(fmt.Sprintf("X: %d cm  Y: %d cm  θ: %d°  Submaps: %d", r.X, r.Y, r.Theta, n))
	}
}

func (gui *MainGUI) updateGlobalSlamView(partialState types.UpdateGui) {
	gSelID := gui.selectedGlobalSlamRobotID
	if gui.slamGShowMatches {
		if img, ok := partialState.SlamGlobalDbgImgs[gSelID]; ok && img != nil {
			gui.slamGCanvas.Image = img
			gui.slamGCanvas.Refresh()
		}
		return
	}

	if img, ok := partialState.SlamGlobalImgs[gSelID]; ok && img != nil {
		gui.slamGCanvas.Image = img
		gui.slamGCanvas.Refresh()
	}
}

func (gui *MainGUI) updateMainMapAndOverlays(partialState types.UpdateGui) {
	gui.redrawMap(gui.mapImage, partialState.NewOpen, partialState.NewObstacle)
	gui.mapCanvas.Refresh()
	gui.redrawRobots(gui.robotManager, partialState.MultiRobot, partialState.Id2index)

	if len(partialState.Lines) > 0 {
		gui.redrawLines(partialState.Lines)
		return
	}

	if len(gui.activeLines) > 0 {
		for _, l := range gui.activeLines {
			l.Hidden = true
		}
		gui.linesContainer.Refresh()
	}
}

func (gui *MainGUI) addManualRobotOption(id int) {
	label := fmt.Sprintf("Robot %d", id)
	found := false
	for _, opt := range gui.manualRobotOptions {
		if opt == label {
			found = true
			break
		}
	}
	if found {
		return
	}
	gui.manualRobotOptions = append(gui.manualRobotOptions, label)
	if gui.manualRobotSelect != nil {
		gui.manualRobotSelect.Options = gui.manualRobotOptions
		gui.manualRobotSelect.Refresh()
		if gui.selectedManualRobotID == -1 {
			gui.manualRobotSelect.SetSelected(label)
		}
	}
	if gui.pathRobotSelect != nil {
		gui.pathRobotSelect.Options = gui.manualRobotOptions
		gui.pathRobotSelect.Refresh()
		if gui.selectedPathRobotID == -1 {
			gui.pathRobotSelect.SetSelected(label)
		}
	}
}

func (gui *MainGUI) updateManualRobotSelector(id2index map[int]int) {
	for id := range id2index {
		gui.addManualRobotOption(id)
	}
}

func (gui *MainGUI) updateSlamRobotSelector(id2index map[int]int) {
	changed := false
	for id := range id2index {
		label := fmt.Sprintf("Robot %d", id)
		found := false
		for _, opt := range gui.slamRobotOptions {
			if opt == label {
				found = true
				break
			}
		}
		if !found {
			gui.slamRobotOptions = append(gui.slamRobotOptions, label)
			gui.slamGRobotOptions = append(gui.slamGRobotOptions, label)
			changed = true
		}
	}
	if changed {
		gui.slamRobotSelect.Options = gui.slamRobotOptions
		gui.slamRobotSelect.Refresh()
		gui.slamGRobotSelect.Options = gui.slamGRobotOptions
		gui.slamGRobotSelect.Refresh()
		if gui.selectedSlamRobotID == -1 && len(gui.slamRobotOptions) > 0 {
			gui.slamRobotSelect.SetSelected(gui.slamRobotOptions[0])
		}
		if gui.selectedGlobalSlamRobotID == -1 && len(gui.slamGRobotOptions) > 0 {
			gui.slamGRobotSelect.SetSelected(gui.slamGRobotOptions[0])
		}
	}
}
