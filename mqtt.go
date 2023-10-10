package main

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"time"

	mqtt "github.com/eclipse/paho.mqtt.golang"
)

type adv_msg struct {
	id                                                          int8
	x, y, theta, ir1x, ir1y, ir2x, ir2y, ir3x, ir3y, ir4x, ir4y int16
	valid                                                       bool
}

func messagePubHandler(
	ch_incoming_msg chan<- adv_msg,
) mqtt.MessageHandler {
	return func(client mqtt.Client, msg mqtt.Message) {
		payload := msg.Payload()
		reader := bytes.NewReader(payload)
		if len(payload) == 24 {
			new_msg := adv_msg{}
			binary.Read(reader, binary.LittleEndian, &new_msg.id)
			binary.Read(reader, binary.LittleEndian, &new_msg.x)
			binary.Read(reader, binary.LittleEndian, &new_msg.y)
			binary.Read(reader, binary.LittleEndian, &new_msg.theta)
			binary.Read(reader, binary.LittleEndian, &new_msg.ir1x)
			binary.Read(reader, binary.LittleEndian, &new_msg.ir1y)
			binary.Read(reader, binary.LittleEndian, &new_msg.ir2x)
			binary.Read(reader, binary.LittleEndian, &new_msg.ir2y)
			binary.Read(reader, binary.LittleEndian, &new_msg.ir3x)
			binary.Read(reader, binary.LittleEndian, &new_msg.ir3y)
			binary.Read(reader, binary.LittleEndian, &new_msg.ir4x)
			binary.Read(reader, binary.LittleEndian, &new_msg.ir4y)
			binary.Read(reader, binary.LittleEndian, &new_msg.valid)

			ch_incoming_msg <- new_msg

			fmt.Printf("Id: %d, x: %d, y: %d, theta: %d, ir1x: %d, ir1y: %d, ir2x: %d, ir2y: %d, ir3x: %d, ir3y: %d, ir4x: %d, ir4y: %d, valid: %t\n", new_msg.id, new_msg.x, new_msg.y, new_msg.theta, new_msg.ir1x, new_msg.ir1y, new_msg.ir2x, new_msg.ir2y, new_msg.ir3x, new_msg.ir3y, new_msg.ir4x, new_msg.ir4y, new_msg.valid)
			//fmt.Printf("Id: %d, x: %d, y: %d, theta: %d\n", new_msg.id, new_msg.x, new_msg.y, new_msg.theta)
		}
	}
}

var connectHandler mqtt.OnConnectHandler = func(client mqtt.Client) {
	fmt.Println("Connected")
}

var connectLostHandler mqtt.ConnectionLostHandler = func(client mqtt.Client, err error) {
	fmt.Printf("Connect lost: %v", err)
}

func publish(
	client mqtt.Client,
	ch_publish <-chan [2]int16,
) {
	num_byte := []byte{2}
	for pos := range ch_publish {

		buf := new(bytes.Buffer)
		binary.Write(buf, binary.LittleEndian, num_byte)
		binary.Write(buf, binary.LittleEndian, pos)

		//text := fmt.Sprintf("Message %d", i)
		token := client.Publish("v2/server/NRF_5/cmd", 0, false, buf.Bytes())
		token.Wait()
		time.Sleep(time.Second)
	}
}

func sub(client mqtt.Client) {
	topic := "v2/robot/NRF_5/adv"
	token := client.Subscribe(topic, 1, nil)
	token.Wait()
	fmt.Printf("Subscribed to topic: %s", topic)
}
