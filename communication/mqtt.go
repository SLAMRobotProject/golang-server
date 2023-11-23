package communication

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"golang-server/config"
	"golang-server/log"
	"golang-server/types"
	"strconv"
	"time"

	mqtt "github.com/eclipse/paho.mqtt.golang"
)

func Mqtt_init() mqtt.Client {
	opts := mqtt.NewClientOptions()
	opts.AddBroker(fmt.Sprintf("tcp://%s:%d", config.BROKER, config.PORT))
	opts.SetDefaultPublishHandler(messagePubHandler)
	opts.OnConnect = connectHandler
	opts.OnConnectionLost = connectLostHandler
	client := mqtt.NewClient(opts)
	if token := client.Connect(); token.Wait() && token.Error() != nil {
		log.G_generalLogger.Println("Failed to connect to mqtt broker. Error: ", token.Error())
		panic(token.Error())
	}
	return client
}

// Must use the correct amount of bytes for each data type.
type advMsgUnpacking struct {
	id                                                          int8
	x, y, theta, ir1x, ir1y, ir2x, ir2y, ir3x, ir3y, ir4x, ir4y int16
	valid                                                       bool
}

var messagePubHandler mqtt.MessageHandler = func(client mqtt.Client, msg mqtt.Message) {
	fmt.Printf("Received message: %s from topic: %s\n", msg.Payload(), msg.Topic())
	log.G_generalLogger.Println("Received message from unsubscribed topic: ", msg.Topic(), " Message: ", msg.Payload())
}

var connectHandler mqtt.OnConnectHandler = func(client mqtt.Client) {
	fmt.Println("Connected to mqtt broker")
	log.G_generalLogger.Println("Connected to mqtt broker")
}

var connectLostHandler mqtt.ConnectionLostHandler = func(client mqtt.Client, err error) {
	fmt.Printf("Lost connection to mqtt broker. Error: %v", err)
	log.G_generalLogger.Println("Lost connection to mqtt broker. Error: ", err)
}

func Thread_publish(
	client mqtt.Client,
	ch_publish <-chan [3]int,
) {
	prefix_byte := []byte{2}
	for msg := range ch_publish {

		buf := new(bytes.Buffer)
		binary.Write(buf, binary.LittleEndian, prefix_byte)
		binary.Write(buf, binary.LittleEndian, int16(msg[1]))
		binary.Write(buf, binary.LittleEndian, int16(msg[2]))

		token := client.Publish("v2/server/NRF_"+strconv.Itoa(msg[0])+"/cmd", 0, false, buf.Bytes())
		token.Wait()
		time.Sleep(time.Second)

		//logging is done in the different functions that writes to ch_publish
	}
}

func adv_messageHandler(
	ch_incoming_msg chan<- types.AdvMsg,
) mqtt.MessageHandler {
	return func(client mqtt.Client, msg mqtt.Message) {
		payload := msg.Payload()
		reader := bytes.NewReader(payload)
		if len(payload) == 24 {
			m := advMsgUnpacking{}
			binary.Read(reader, binary.LittleEndian, &m.id)
			binary.Read(reader, binary.LittleEndian, &m.x)
			binary.Read(reader, binary.LittleEndian, &m.y)
			binary.Read(reader, binary.LittleEndian, &m.theta)
			binary.Read(reader, binary.LittleEndian, &m.ir1x)
			binary.Read(reader, binary.LittleEndian, &m.ir1y)
			binary.Read(reader, binary.LittleEndian, &m.ir2x)
			binary.Read(reader, binary.LittleEndian, &m.ir2y)
			binary.Read(reader, binary.LittleEndian, &m.ir3x)
			binary.Read(reader, binary.LittleEndian, &m.ir3y)
			binary.Read(reader, binary.LittleEndian, &m.ir4x)
			binary.Read(reader, binary.LittleEndian, &m.ir4y)
			binary.Read(reader, binary.LittleEndian, &m.valid) //valid is not used in the robot code

			new_msg := types.AdvMsg{
				Id:    int(m.id),
				X:     int(m.x),
				Y:     int(m.y),
				Theta: int(m.theta),
				Ir1x:  int(m.ir1x),
				Ir1y:  int(m.ir1y),
				Ir2x:  int(m.ir2x),
				Ir2y:  int(m.ir2y),
				Ir3x:  int(m.ir3x),
				Ir3y:  int(m.ir3y),
				Ir4x:  int(m.ir4x),
				Ir4y:  int(m.ir4y),
			}

			ch_incoming_msg <- new_msg

			// One robots sends about 30 messages per second. Uncomment the following lines to see the messages.

			//fmt.Printf("Id: %d, x: %d, y: %d, theta: %d, ir1x: %d, ir1y: %d, ir2x: %d, ir2y: %d, ir3x: %d, ir3y: %d, ir4x: %d, ir4y: %d\n", new_msg.id, new_msg.x, new_msg.y, new_msg.theta, new_msg.ir1x, new_msg.ir1y, new_msg.ir2x, new_msg.ir2y, new_msg.ir3x, new_msg.ir3y, new_msg.ir4x, new_msg.ir4y)
			//log.G_generalLogger.Printf("Id: %d, x: %d, y: %d, theta: %d, ir1x: %d, ir1y: %d, ir2x: %d, ir2y: %d, ir3x: %d, ir3y: %d, ir4x: %d, ir4y: %d\n", new_msg.id, new_msg.x, new_msg.y, new_msg.theta, new_msg.ir1x, new_msg.ir1y, new_msg.ir2x, new_msg.ir2y, new_msg.ir3x, new_msg.ir3y, new_msg.ir4x, new_msg.ir4y)
			//fmt.Printf("Id: %d, x: %d, y: %d, theta: %d\n", new_msg.id, new_msg.x, new_msg.y, new_msg.theta)
			//log.G_generalLogger.Printf("Id: %d, x: %d, y: %d, theta: %d\n", new_msg.id, new_msg.x, new_msg.y, new_msg.theta)
		}
	}
}

func Subscribe(
	client mqtt.Client,
	ch_incoming_msg chan<- types.AdvMsg,
) {
	topic := "v2/robot/NRF_5/adv"
	token := client.Subscribe(topic, 1, adv_messageHandler(ch_incoming_msg))
	token.Wait()
	fmt.Printf("Subscribed to topic: %s", topic)
	log.G_generalLogger.Println("Subscribed to topic: ", topic)
}
