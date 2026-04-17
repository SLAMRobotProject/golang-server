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
	IsViritual bool `json:"IsViritual"`
	IsCamera   bool `json:"IsCamera"`
	X          int  `json:"X"`
	Y          int  `json:"Y"`
	Theta      int  `json:"Theta"`
	StartMM    int  `json:"StartMM"`
	WidthMM    int  `json:"WidthMM"`
	DistanceMM int  `json:"DistanceMM"`
}

func StartDigitalTwinTCPServer(port string, chCamera chan<- types.CameraMsg, chReceive chan<- types.AdvMsg, chG2bRobotInit chan<- [4]int, chPoseUpdate <-chan types.PoseUpdateMsg) {
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

		// Start a thread purely to send pose updates to this TCP connection
		go func(c net.Conn) {
			for poseUpdate := range chPoseUpdate {
				jsonStr := fmt.Sprintf(`{"type":"pose_update","x":%f,"y":%f,"theta":%f}`+"\n", poseUpdate.X, poseUpdate.Y, poseUpdate.Theta)
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
			chReceive <- types.AdvMsg{
				Id:    msg.Id,
				X:     msg.X,
				Y:     msg.Y,
				Theta: msg.Theta,
			}
		}
	}
}
