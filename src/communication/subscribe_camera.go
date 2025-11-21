package communication

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"golang-server/config"
	"golang-server/log"
	"golang-server/types"

	mqtt "github.com/eclipse/paho.mqtt.golang"
)

// SubscribeCamera subscribes to camera topic and dispatches messages to chCamera
// only if config.UseNiclaVision is enabled. The camera payload is expected to
// contain three little-endian int16 values: x_start_mm, x_width_mm, distance_mm.
func SubscribeCamera(client mqtt.Client, chCamera chan<- types.CameraMsg) {
	if !config.UseNiclaVision {
		fmt.Println("\nNicla vision disabled via config.UseNiclaVision; camera subscription skipped")
		log.GGeneralLogger.Println("Nicla vision disabled via config.UseNiclaVision; camera subscription skipped")
		return
	}

	handler := func(client mqtt.Client, msg mqtt.Message) {
		payload := msg.Payload()
		topic := string(msg.Topic())
		log.GGeneralLogger.Printf("Camera message received on topic %s, %d bytes", topic, len(payload))
		reader := bytes.NewReader(payload)

		// Expect 1 byte identifier + 3x int16 = 7 bytes total
		if len(payload) != 7 {
			log.GGeneralLogger.Printf("Camera payload length not 7 bytes: %d bytes", len(payload))
			return
		}

		// Read payload
		var identifier uint8
		var start int16
		var width int16
		var distance int16

		if err := binary.Read(reader, binary.LittleEndian, &identifier); err != nil {
			log.GGeneralLogger.Printf("Failed to read camera payload identifier: %v", err)
			return
		}
		if err := binary.Read(reader, binary.LittleEndian, &start); err != nil {
			log.GGeneralLogger.Printf("Failed to read camera payload start: %v", err)
			return
		}
		if err := binary.Read(reader, binary.LittleEndian, &width); err != nil {
			log.GGeneralLogger.Printf("Failed to read camera payload width: %v", err)
			return
		}
		if err := binary.Read(reader, binary.LittleEndian, &distance); err != nil {
			log.GGeneralLogger.Printf("Failed to read camera payload distance: %v", err)
			return
		}

		// Log a single line with the parsed camera values including id
		log.GGeneralLogger.Printf("Camera message received: id=%d start=%d width=%d distance=%d", int(identifier), int(start), int(width), int(distance))

		// Ignore empty / invalid camera measurements (no detection)
		if start == 0 && width == 0 && distance == 0 {
			return
		}

		// Build and send a dedicated CameraMsg
		cam := types.CameraMsg{
			Id:         int(identifier),
			StartMM:    int(start),
			WidthMM:    int(width),
			DistanceMM: int(distance),
		}
		chCamera <- cam
	}

	topic := "v2/robot/+/cam"
	token := client.Subscribe(topic, 1, handler)
	token.Wait()
	fmt.Printf("\nSubscribed to camera topic: %s\n", topic)
	log.GGeneralLogger.Println("Subscribed to camera topic: ", topic)
}
