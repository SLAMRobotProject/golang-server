package communication

import (
	"bufio"
	"encoding/json"
	"fmt"
	"golang-server/log"
	"golang-server/types"
	"net"
	"sync"
)

type digitalTwinJSONMsg struct {
	Id         int  `json:"Id"`
	IsViritual bool `json:"IsViritual"`
	IsCamera   bool `json:"IsCamera"`
	X          int  `json:"X"`
	Y          int  `json:"Y"`
	Theta      int  `json:"Theta"`
	StartMM    int  `json:"StartMM"`
	WidthMM    int  `json:"WidthMM"`
	DistanceMM int  `json:"DistanceMM"`
}

func StartDigitalTwinTCPServer(port string, chCamera chan<- types.CameraMsg, chRobotTelemetry chan<- types.RobotTelemetryMsg, chG2bRobotInit chan<- [4]int, chVirtualTarget <-chan types.VirtualTargetMsg, chB2gVirtualPending chan<- int) {
	listener, err := net.Listen("tcp", port)
	if err != nil {
		log.GGeneralLogger.Printf("Kunne ikke starte TCP server på %s: %v", port, err)
		return
	}
	defer listener.Close()

	fmt.Printf("Digital Twin TCP-server lytter på %s\n", port)

	// connChans maps robot ID -> per-connection send channel.
	connChans := make(map[int]chan string)
	var connMu sync.Mutex

	// Fan out chVirtualTarget to the matching per-connection channel.
	go func() {
		for msg := range chVirtualTarget {
			connMu.Lock()
			ch, ok := connChans[msg.Id]
			connMu.Unlock()
			if ok {
				jsonStr := fmt.Sprintf(`{"type":"target","x":%f,"y":%f}`+"\n", msg.X, msg.Y)
				select {
				case ch <- jsonStr:
				default:
				}
			}
		}
	}()

	for {
		conn, err := listener.Accept()
		if err != nil {
			log.GGeneralLogger.Printf("Feil ved tilkobling: %v", err)
			continue
		}

		fmt.Println("Digital Twin koblet til!")

		go handleTwinConnection(conn, chCamera, chRobotTelemetry, chG2bRobotInit, connChans, &connMu, chB2gVirtualPending)
	}
}

func handleTwinConnection(conn net.Conn, chCamera chan<- types.CameraMsg, chRobotTelemetry chan<- types.RobotTelemetryMsg, chG2bRobotInit chan<- [4]int, connChans map[int]chan string, connMu *sync.Mutex, chB2gVirtualPending chan<- int) {
	defer conn.Close()

	scanner := bufio.NewScanner(conn)

	var robotID int
	initialized := false

	for scanner.Scan() {
		line := scanner.Text()

		var msg digitalTwinJSONMsg
		if err := json.Unmarshal([]byte(line), &msg); err != nil {
			continue // ignore bad json
		}

		if !initialized {
			robotID = msg.Id
			// Initialize at 0,0 since backend.go automatically offsets from map center.
			//chG2bRobotInit <- [4]int{msg.Id, 0, 0, 0}

			// Register a send channel for this robot so virtual targets can reach it.
			sendCh := make(chan string, 4)
			connMu.Lock()
			connChans[robotID] = sendCh
			connMu.Unlock()

			// Writer goroutine: drains sendCh and writes to the connection.
			go func(c net.Conn, ch <-chan string) {
				for s := range ch {
					c.Write([]byte(s))
				}
			}(conn, sendCh)

			// Notify the GUI so it can show an "Add controls" button for this twin.
			select {
			case chB2gVirtualPending <- robotID:
			default:
			}

			initialized = true
		}

		// Dispatch depending on message type
		if msg.IsCamera {
			chCamera <- types.CameraMsg{
				Id:         msg.Id,
				IsViritual: msg.IsViritual,
				Obj: types.CameraObject{
					StartMM: msg.StartMM,
					WidthMM: msg.WidthMM,
					DistMM:  msg.DistanceMM,
				},
			}
		} else if msg.IsViritual {
			chCamera <- types.CameraMsg{
				Id:         msg.Id,
				IsViritual: true,
				Obj: types.CameraObject{
					StartMM: msg.StartMM,
					WidthMM: msg.WidthMM,
					DistMM:  msg.DistanceMM,
				},
			}
		} else {
			chRobotTelemetry <- types.RobotTelemetryMsg{
				Id:    msg.Id,
				X:     msg.X,
				Y:     msg.Y,
				Theta: msg.Theta,
			}
		}
	}

	// Clean up on disconnect.
	if initialized {
		connMu.Lock()
		if ch, ok := connChans[robotID]; ok {
			close(ch)
			delete(connChans, robotID)
		}
		connMu.Unlock()
	}
}
