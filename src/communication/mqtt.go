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

func InitMqtt() mqtt.Client {
	opts := mqtt.NewClientOptions()
	opts.AddBroker(fmt.Sprintf("tcp://%s:%d", config.Broker, config.Port))
	opts.SetDefaultPublishHandler(messagePubHandler)
	opts.OnConnect = connectHandler
	opts.OnConnectionLost = connectLostHandler
	client := mqtt.NewClient(opts)
	if token := client.Connect(); token.Wait() && token.Error() != nil {
		log.GGeneralLogger.Println("Failed to connect to mqtt broker. Error: ", token.Error())
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

type recMsgUnpacking struct {
	id 					uint8
<<<<<<< HEAD
	totalMap			bool
	x, y, width, height int16
	obstacle            uint8
	reachable           bool
=======
	x, y				int16
	obstacle            uint8
>>>>>>> efdd8b5 (This server is adapted for the mapping scheme, should be checked before merging with main)
}

var messagePubHandler mqtt.MessageHandler = func(client mqtt.Client, msg mqtt.Message) {
	fmt.Printf("Received message: %s from topic: %s\n", msg.Payload(), msg.Topic())
	log.GGeneralLogger.Println("Received message from unsubscribed topic: ", msg.Topic(), " Message: ", msg.Payload())
}

var connectHandler mqtt.OnConnectHandler = func(client mqtt.Client) {
	fmt.Println("Connected to mqtt broker")
	log.GGeneralLogger.Println("Connected to mqtt broker")
}

var connectLostHandler mqtt.ConnectionLostHandler = func(client mqtt.Client, err error) {
	fmt.Printf("Lost connection to mqtt broker. Error: %v", err)
	log.GGeneralLogger.Println("Lost connection to mqtt broker. Error: ", err)
}

func ThreadMqttPublish(
	client mqtt.Client,
	chPublish <-chan [3]int,
	chPublishInit <-chan [4]int,
<<<<<<< HEAD
) {
	//prefixByte := []byte{2} //because the robot code expects a byte here
	//init:=false
=======
	chPublishToggleMapping <-chan [2]int,
) {
>>>>>>> efdd8b5 (This server is adapted for the mapping scheme, should be checked before merging with main)

	for {
		select {
		case msgCommand := <-chPublish:
<<<<<<< HEAD
			buf := new(bytes.Buffer)
			binary.Write(buf, binary.LittleEndian, uint8(1))
=======
			prefixByte := []byte{2}
			buf := new(bytes.Buffer)
			binary.Write(buf, binary.LittleEndian, uint8(1))
			binary.Write(buf, binary.LittleEndian, prefixByte)
>>>>>>> efdd8b5 (This server is adapted for the mapping scheme, should be checked before merging with main)
			binary.Write(buf, binary.LittleEndian, int16(msgCommand[1]))
			binary.Write(buf, binary.LittleEndian, int16(msgCommand[2]))
			token := client.Publish("v2/server/NRF_"+strconv.Itoa(msgCommand[0])+"/serverpub", 0, false, buf.Bytes())
			token.Wait()
			time.Sleep(time.Second)
		case msgInit := <-chPublishInit:
<<<<<<< HEAD
			buf := new(bytes.Buffer)
			binary.Write(buf, binary.LittleEndian, uint8(2))
=======
			prefixByte := []byte{2}
			buf := new(bytes.Buffer)
			binary.Write(buf, binary.LittleEndian, uint8(2))
			binary.Write(buf, binary.LittleEndian, prefixByte)
>>>>>>> efdd8b5 (This server is adapted for the mapping scheme, should be checked before merging with main)
			binary.Write(buf, binary.LittleEndian, int16(msgInit[1]))                                          //x
			binary.Write(buf, binary.LittleEndian, int16(msgInit[2]))                                         //y
			binary.Write(buf, binary.LittleEndian, int16(msgInit[3]))                                         //theta
			token := client.Publish("v2/server/NRF_"+strconv.Itoa(msgInit[0])+"/serverpub", 0, false, buf.Bytes()) //Publish to coresponding robot
			token.Wait()
			time.Sleep(time.Second)
			if token.Wait() && token.Error() != nil {
				fmt.Println("Error publishing")
				fmt.Printf("Buffer:%v\n", buf.Bytes())

			}
			//Resend in default case?
<<<<<<< HEAD
=======
		case msgToggleMapping := <-chPublishToggleMapping: //Publish start or stop command
			buf:=new(bytes.Buffer)
			binary.Write(buf,binary.LittleEndian,uint8(3))
			binary.Write(buf,binary.LittleEndian,int16(msgToggleMapping[1]))
			token := client.Publish("v2/server/NRF_"+strconv.Itoa(msgToggleMapping[0])+"/serverpub", 0, false, buf.Bytes()) //Publish to coresponding robot
			token.Wait()
			time.Sleep(time.Second)
>>>>>>> efdd8b5 (This server is adapted for the mapping scheme, should be checked before merging with main)
		}
		
	}
}

func advMessageHandler(
	chIncomingMsg chan<- types.AdvMsg,
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

			newMsg := types.AdvMsg{
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

			chIncomingMsg <- newMsg

			// One robots sends about 30 messages per second. Uncomment the following lines to see the messages.

			//fmt.Printf("Id: %d, x: %d, y: %d, theta: %d, ir1x: %d, ir1y: %d, ir2x: %d, ir2y: %d, ir3x: %d, ir3y: %d, ir4x: %d, ir4y: %d\n", newMsg.Id, newMsg.X, newMsg.Y, newMsg.Theta, newMsg.Ir1x, newMsg.Ir1y, newMsg.Ir2x, newMsg.Ir2y, newMsg.Ir3x, newMsg.Ir3y, newMsg.Ir4x, newMsg.Ir4y)
			log.GGeneralLogger.Printf("Id: %d, x: %d, y: %d, theta: %d, ir1x: %d, ir1y: %d, ir2x: %d, ir2y: %d, ir3x: %d, ir3y: %d, ir4x: %d, ir4y: %d\n", newMsg.Id, newMsg.X, newMsg.Y, newMsg.Theta, newMsg.Ir1x, newMsg.Ir1y, newMsg.Ir2x, newMsg.Ir2y, newMsg.Ir3x, newMsg.Ir3y, newMsg.Ir4x, newMsg.Ir4y)
			//fmt.Printf("Id: %d, x: %d, y: %d, theta: %d\n", newMsg.Id, newMsg.X, newMsg.Y, newMsg.Theta)
			//log.GGeneralLogger.Printf("Id: %d, x: %d, y: %d, theta: %d\n", newMsg.Id, newMsg.X, newMsg.Y, newMsg.Theta)
		}
	}
}

func recMessageHandler(
	chIncomingMsg chan<- types.RectangleMsg,
) mqtt.MessageHandler {
	return func(client mqtt.Client, msg mqtt.Message) {
		payload := msg.Payload()
		reader := bytes.NewReader(payload)
<<<<<<< HEAD
		if len(payload) == 12 { //Amount of bytes in message
			m := recMsgUnpacking{}
			binary.Read(reader, binary.LittleEndian, &m.id)
			binary.Read(reader, binary.LittleEndian, &m.totalMap)
			binary.Read(reader, binary.LittleEndian, &m.x)
			binary.Read(reader, binary.LittleEndian, &m.y)
			binary.Read(reader, binary.LittleEndian, &m.width)
			binary.Read(reader, binary.LittleEndian, &m.height)
			binary.Read(reader, binary.LittleEndian, &m.obstacle)
			binary.Read(reader, binary.LittleEndian, &m.reachable)

			newMsg := types.RectangleMsg{
				Id: 	   int(m.id),
				TotalMap:  bool(m.totalMap),
				X:         int(m.x),
				Y:         int(m.y),
				Width:     int(m.width),
				Height:    int(m.height),
				Obstacle:  int(m.obstacle),
				Reachable: bool(m.reachable),
=======
		if len(payload) == 6 { //Amount of bytes in message
			m := recMsgUnpacking{}
			binary.Read(reader, binary.LittleEndian, &m.id)
			binary.Read(reader, binary.LittleEndian, &m.x)
			binary.Read(reader, binary.LittleEndian, &m.y)
			binary.Read(reader, binary.LittleEndian, &m.obstacle)

			newMsg := types.RectangleMsg{
				Id: 	   int(m.id),
				X:         int(m.x),
				Y:         int(m.y),
				Obstacle:  int(m.obstacle),
>>>>>>> efdd8b5 (This server is adapted for the mapping scheme, should be checked before merging with main)
			}
			chIncomingMsg <- newMsg

		}
	}

}

func Subscribe(
	client mqtt.Client,
	chIncomingMsg chan<- types.AdvMsg,
	chIncomingMsgMap chan<- types.RectangleMsg,
) {
	topic := "v2/robot/NRF_5/adv"
	token := client.Subscribe(topic, 1, advMessageHandler(chIncomingMsg))
	token.Wait()
	fmt.Printf("Subscribed to topic: %s", topic)
	log.GGeneralLogger.Println("Subscribed to topic: ", topic)
	topic2 := "v2/robot/NRF_5/rectangle"
	token2 := client.Subscribe(topic2, 1, recMessageHandler(chIncomingMsgMap))
	token2.Wait()
	fmt.Printf("\nSubscribed to topic: %s", topic2)
	log.GGeneralLogger.Println("Subscribed to topic: ", topic2)
}
