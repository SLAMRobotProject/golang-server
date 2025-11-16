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

	topic := fmt.Sprintf("v2/robot/NRF_%d/cam", robotID)

	// Example values (mm): start = -50 (left), width = 100, distance = 500
	start := int16(-400)
	width := int16(400)
	distance := int16(400)

	buf := new(bytes.Buffer)
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
