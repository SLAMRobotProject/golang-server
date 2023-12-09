package main

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"math"
	"math/rand"
	"time"

	mqtt "github.com/eclipse/paho.mqtt.golang"
)

// config...
const ID = 3
const MSG_PR_SECOND = 10
const DURATION = 200 //seconds

func publish(client mqtt.Client) {

	x, y, theta := 0, 0, 0
	var dx, dy int16 = 0, 0

	msg := adv_msg_packing{ID, 0, 0, 0, 1000, 0, -1000, 0, 0, -1000, 0, 1000, false}
	randomize_direction(x, y, theta, &msg)

	buf := new(bytes.Buffer)
	write_msg_to_buffer(buf, msg)

	start_time := time.Now()
	last_randomize := time.Now()
	for start_time.Add(time.Second * DURATION).After(time.Now()) {
		token := client.Publish("v2/robot/NRF_5/adv", 0, false, buf.Bytes())
		token.Wait()

		x, y, theta = int(msg.x), int(msg.y), int(msg.theta)

		time.Sleep(time.Second / MSG_PR_SECOND)

		if last_randomize.Add(time.Second * 4).Before(time.Now()) {
			dx, dy = randomize_direction(x, y, theta, &msg)
			buf.Reset()
			write_msg_to_buffer(buf, msg)
			last_randomize = time.Now()
		} else {
			msg.x += dx
			msg.y += dy
			buf.Reset()
			write_msg_to_buffer(buf, msg)
		}
	}
}

func randomize_direction(x, y, theta int, msg *adv_msg_packing) (int16, int16) {
	var angle float64
	speed := 30.0
	if x > 1500 || x < -1500 || y > 1500 || y < -1500 {
		//go back to senter when close to edge
		angle = math.Atan2(-float64(y), -float64(x))
	} else {
		angle = rand.Float64()*2*math.Pi - math.Pi

	}
	dx, dy := math.Cos(angle)*speed, math.Sin(angle)*speed
	msg.x = int16(dx) + int16(x)
	msg.y = int16(dy) + int16(y)
	msg.theta = int16(math.Round(angle * 180 / math.Pi))
	return int16(dx), int16(dy)
}

var connectHandler mqtt.OnConnectHandler = func(client mqtt.Client) {
	fmt.Println("Connected to broker")
}

var connectLostHandler mqtt.ConnectionLostHandler = func(client mqtt.Client, err error) {
	fmt.Printf("Connect lost: %v", err)
}

type adv_msg_packing struct {
	id                                                          int8
	x, y, theta, ir1x, ir1y, ir2x, ir2y, ir3x, ir3y, ir4x, ir4y int16
	valid                                                       bool
}

func write_msg_to_buffer(buf *bytes.Buffer, msg adv_msg_packing) {
	binary.Write(buf, binary.LittleEndian, msg.id)
	binary.Write(buf, binary.LittleEndian, msg.x)
	binary.Write(buf, binary.LittleEndian, msg.y)
	binary.Write(buf, binary.LittleEndian, msg.theta)
	binary.Write(buf, binary.LittleEndian, msg.ir1x)
	binary.Write(buf, binary.LittleEndian, msg.ir1y)
	binary.Write(buf, binary.LittleEndian, msg.ir2x)
	binary.Write(buf, binary.LittleEndian, msg.ir2y)
	binary.Write(buf, binary.LittleEndian, msg.ir3x)
	binary.Write(buf, binary.LittleEndian, msg.ir3y)
	binary.Write(buf, binary.LittleEndian, msg.ir4x)
	binary.Write(buf, binary.LittleEndian, msg.ir4y)
	binary.Write(buf, binary.LittleEndian, msg.valid)
}

func main() {
	var broker = "broker.emqx.io"
	var port = 1883
	opts := mqtt.NewClientOptions()
	opts.AddBroker(fmt.Sprintf("tcp://%s:%d", broker, port))
	opts.OnConnect = connectHandler
	opts.OnConnectionLost = connectLostHandler
	client := mqtt.NewClient(opts)
	if token := client.Connect(); token.Wait() && token.Error() != nil {
		panic(token.Error())
	}

	publish(client)

	client.Disconnect(250)

	fmt.Println("Programme end")
}
