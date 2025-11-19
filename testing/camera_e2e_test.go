package main

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"os"
	"strconv"
	"testing"
	"time"

	mqtt "github.com/eclipse/paho.mqtt.golang"
)

// TestCameraPublish publishes a sample camera segment payload to the MQTT broker.
// This is an end-to-end helper test
// Run it while the server is running and a robot is also running, match the robot ID.
// Run with: go test -v
func TestCameraPublish(t *testing.T) {
	broker := os.Getenv("MQTT_BROKER")
	if broker == "" {
		broker = "slam"
	}
	portStr := os.Getenv("MQTT_PORT")
	port := 1883
	if portStr != "" {
		p, err := strconv.Atoi(portStr)
		if err == nil {
			port = p
		}
	}
	robotID := 6
	if rid := os.Getenv("MQTT_ROBOT_ID"); rid != "" {
		if r, err := strconv.Atoi(rid); err == nil {
			robotID = r
		}
	}

	opts := mqtt.NewClientOptions()
	opts.AddBroker(fmt.Sprintf("tcp://%s:%d", broker, port))
	client := mqtt.NewClient(opts)
	if token := client.Connect(); token.Wait() && token.Error() != nil {
		t.Fatalf("Failed to connect to broker %s:%d: %v", broker, port, token.Error())
	}
	defer client.Disconnect(250)

	// Topic can be fixed; the payload now contains the robot identifier as
	// the first byte (mqttsn_camera_msg.identifier).
	topic := "v2/robot/NRF_5/cam"

	// Example values (mm): start = 0 (center), width = 400, distance = 400
	start := int16(0)
	width := int16(400)
	distance := int16(400)

	buf := new(bytes.Buffer)
	// Write identifier (uint8) first, then three little-endian int16 values
	binary.Write(buf, binary.LittleEndian, uint8(robotID))
	binary.Write(buf, binary.LittleEndian, start)
	binary.Write(buf, binary.LittleEndian, width)
	binary.Write(buf, binary.LittleEndian, distance)

	t.Logf("Publishing camera test payload to %s (broker=%s:%d)", topic, broker, port)
	token := client.Publish(topic, 1, false, buf.Bytes())
	token.Wait()

	// give server time to receive and process
	time.Sleep(500 * time.Millisecond)

	t.Log("Publish complete")
}
