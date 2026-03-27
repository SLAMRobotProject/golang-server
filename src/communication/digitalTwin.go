package communication

import (
	"bufio"
	"encoding/json"
	"fmt"
	"golang-server/log"
	"golang-server/types"
	"net"
)

type digitalTwinJSONMsg struct {
	Id         int  `json:"Id"`
	IsDirect   bool `json:"IsDirect"`
	IsCamera   bool `json:"IsCamera"`
	X          int  `json:"X"`
	Y          int  `json:"Y"`
	Theta      int  `json:"Theta"`
	P1X        int  `json:"P1X"`
	P1Y        int  `json:"P1Y"`
	P2X        int  `json:"P2X"`
	P2Y        int  `json:"P2Y"`
	RhoMM      int  `json:"RhoMM"`
	AlphaMRad  int  `json:"AlphaMRad"`
	StartMM    int  `json:"StartMM"`
	WidthMM    int  `json:"WidthMM"`
	DistanceMM int  `json:"DistanceMM"`
}

func StartDigitalTwinTCPServer(port string, chCamera chan<- types.CameraMsg, chReceive chan<- types.AdvMsg, chG2bRobotInit chan<- [4]int, chCorrection <-chan types.CorrectionMsg) {
	listener, err := net.Listen("tcp", port)
	if err != nil {
		log.GGeneralLogger.Printf("Kunne ikke starte TCP server pÃ¥ %s: %v", port, err)
		return
	}
	defer listener.Close()

	fmt.Printf("Digital Twin TCP-server lytter pÃ¥ %s\n", port)

	for {
		conn, err := listener.Accept()
		if err != nil {
			log.GGeneralLogger.Printf("Feil ved tilkobling: %v", err)
			continue
		}

		fmt.Println("Digital Twin koblet til!")

		// Start a thread purely to send corrections to this TCP connection
		go func(c net.Conn) {
			for corr := range chCorrection {
				jsonStr := fmt.Sprintf(`{"type":"correction","x":%f,"y":%f,"theta":%f}`+"\n", corr.X, corr.Y, corr.Theta)
				c.Write([]byte(jsonStr))
			}
		}(conn)

		go handleTwinConnection(conn, chCamera, chReceive, chG2bRobotInit)
	}
}

func handleTwinConnection(conn net.Conn, chCamera chan<- types.CameraMsg, chReceive chan<- types.AdvMsg, chG2bRobotInit chan<- [4]int) {
	defer conn.Close()

	scanner := bufio.NewScanner(conn)

	initialized := false

	for scanner.Scan() {
		line := scanner.Text()

		var msg digitalTwinJSONMsg
		if err := json.Unmarshal([]byte(line), &msg); err != nil {
			continue // ignore bad json
		}

		if !initialized {
			// Initialize at 0,0 since backend.go automatically Offsets from map center
			chG2bRobotInit <- [4]int{msg.Id, 0, 0, 0}
			initialized = true
		}

		// Dispatch depending on message type
		if msg.IsCamera {
			chCamera <- types.CameraMsg{
				Id:       msg.Id,
				IsDirect: msg.IsDirect,
				P1X:      msg.P1X, P1Y: msg.P1Y,
				P2X: msg.P2X, P2Y: msg.P2Y,
				RhoMM: msg.RhoMM, AlphaMRad: msg.AlphaMRad,
				StartMM:    msg.StartMM,
				WidthMM:    msg.WidthMM,
				DistanceMM: msg.DistanceMM,
			}
		} else if msg.IsDirect {
			chCamera <- types.CameraMsg{
				Id:       msg.Id,
				IsDirect: true,
				P1X:      msg.P1X, P1Y: msg.P1Y,
				P2X: msg.P2X, P2Y: msg.P2Y,
				RhoMM: msg.RhoMM, AlphaMRad: msg.AlphaMRad,
			}
		} else {
			chReceive <- types.AdvMsg{
				Id:    msg.Id,
				X:     msg.X,
				Y:     msg.Y,
				Theta: msg.Theta,
			}
		}
	}
}
