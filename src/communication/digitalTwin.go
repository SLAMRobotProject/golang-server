package communication

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"golang-server/log"
	"golang-server/types"
	"io"
	"net"
)

func StartDigitalTwinTCPServer(port string, chCamera chan<- types.CameraMsg, chReceive chan<- types.AdvMsg, chG2bRobotInit chan<- [4]int) {
	listener, err := net.Listen("tcp", port)
	if err != nil {
		log.GGeneralLogger.Printf("Kunne ikke starte TCP server på %s: %v", port, err)
		return
	}
	defer listener.Close()

	fmt.Printf("Digital Twin TCP-server lytter på %s\n", port)

	for {
		conn, err := listener.Accept()
		if err != nil {
			log.GGeneralLogger.Printf("Feil ved tilkobling: %v", err)
			continue
		}

		fmt.Println("Digital Twin koblet til! Auto-initialiserer ID 99...")
		go func() { chG2bRobotInit <- [4]int{99, 0, 0, 0} }()

		go handleTwinConnection(conn, chCamera, chReceive)
	}
}

func handleTwinConnection(conn net.Conn, chCamera chan<- types.CameraMsg, chReceive chan<- types.AdvMsg) {
	defer conn.Close()

	// NY STØRRELSE: 19 BYTES
	// ID(1) + RobotPos(6) + P1(4) + P2(4) + Hesse(4) = 19
	// 1. En header-buffer på 1 byte for å finne starten (ID 99)
	scanBuf := make([]byte, 1)
	// 2. En payload-buffer for resten av pakken (18 bytes)
	dataBuf := make([]byte, 19)

	for {
		// Les byte 1 (start-byte 0xFF)
		_, err := io.ReadFull(conn, scanBuf)
		if err != nil {
			return
		}
		if scanBuf[0] != 0xFF {
			continue
		}

		// Les byte 2 (Vi fant 0xFF, nå må neste være 0xFE)
		_, err = io.ReadFull(conn, scanBuf)
		if err != nil {
			return
		}
		if scanBuf[0] != 0xFE {
			continue
		}

		// Les resten av pakken (19 bytes)
		_, err = io.ReadFull(conn, dataBuf)
		if err != nil {
			return
		}

		reader := bytes.NewReader(dataBuf)
		var id uint8
		var x, y, theta int16
		var p1x, p1y, p2x, p2y int16
		var rho, alpha int16 // Hesse variabler

		// Les Data
		binary.Read(reader, binary.LittleEndian, &id)
		binary.Read(reader, binary.LittleEndian, &x)
		binary.Read(reader, binary.LittleEndian, &y)
		binary.Read(reader, binary.LittleEndian, &theta)

		binary.Read(reader, binary.LittleEndian, &p1x)
		binary.Read(reader, binary.LittleEndian, &p1y)
		binary.Read(reader, binary.LittleEndian, &p2x)
		binary.Read(reader, binary.LittleEndian, &p2y)

		// Les Hesse
		binary.Read(reader, binary.LittleEndian, &rho)
		binary.Read(reader, binary.LittleEndian, &alpha)

		// Send Posisjon
		chReceive <- types.AdvMsg{
			Id:    int(id),
			X:     int(x) * 10,
			Y:     int(y) * 10,
			Theta: int(theta),
		}

		// Send Alt til Backend (Både P1/P2 og Rho/Alpha)
		if p1x != 0 || p1y != 0 || p2x != 0 || p2y != 0 {
			chCamera <- types.CameraMsg{
				Id:       int(id),
				IsDirect: true,

				// For GUI
				P1X: int(p1x), P1Y: int(p1y),
				P2X: int(p2x), P2Y: int(p2y),

				// For SLAM
				RhoMM:     int(rho),
				AlphaMRad: int(alpha),
			}
		}
	}
}
