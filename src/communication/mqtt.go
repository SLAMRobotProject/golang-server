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

type Coordinate struct {
	X int16
	Y int16
}

type advMsgUnpacking struct {
	Id               int8
	X                int16
	Y                int16
	Theta            int16
	IR               [4]Coordinate // Replaced ir1x, ir1y, etc. with an array
	CovarianceMatrix [25]float32   // Replaced individual fields with an array
	Valid            uint8         // Corrected from bool to uint8 to match C's bitmask
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
) {
	prefixByte := []byte{2} //because the robot code expects a byte here
	for msg := range chPublish {

		buf := new(bytes.Buffer)
		binary.Write(buf, binary.LittleEndian, prefixByte)
		binary.Write(buf, binary.LittleEndian, int16(msg[1]))
		binary.Write(buf, binary.LittleEndian, int16(msg[2]))

		token := client.Publish("v2/server/NRF_"+strconv.Itoa(msg[0])+"/cmd", 0, false, buf.Bytes())
		token.Wait()
		time.Sleep(time.Second)

		//logging is done in the different functions that writes to chPublish
	}
}

func advMessageHandler(
	chIncomingMsg chan<- types.AdvMsg,
) mqtt.MessageHandler {
	return func(client mqtt.Client, msg mqtt.Message) {
		payload := msg.Payload()
		reader := bytes.NewReader(payload)

		if len(payload) == 124 {
			m := advMsgUnpacking{}

			err := binary.Read(reader, binary.LittleEndian, &m)
			if err != nil {
				log.GGeneralLogger.Printf("Failed to read binary payload: %v", err)
				return
			}

			newMsg := types.AdvMsg{
				Id:                       int(m.Id),
				X:                        int(m.X),
				Y:                        int(m.Y),
				Theta:                    int(m.Theta),
				Ir1x:                     int(m.IR[0].X),
				Ir1y:                     int(m.IR[0].Y),
				Ir2x:                     int(m.IR[1].X),
				Ir2y:                     int(m.IR[1].Y),
				Ir3x:                     int(m.IR[2].X),
				Ir3y:                     int(m.IR[2].Y),
				Ir4x:                     int(m.IR[3].X),
				Ir4y:                     int(m.IR[3].Y),
				Valid:                    m.Valid,
				CovarianceMatrixNumber1:  m.CovarianceMatrix[0],
				CovarianceMatrixNumber2:  m.CovarianceMatrix[1],
				CovarianceMatrixNumber3:  m.CovarianceMatrix[2],
				CovarianceMatrixNumber4:  m.CovarianceMatrix[3],
				CovarianceMatrixNumber5:  m.CovarianceMatrix[4],
				CovarianceMatrixNumber6:  m.CovarianceMatrix[5],
				CovarianceMatrixNumber7:  m.CovarianceMatrix[6],
				CovarianceMatrixNumber8:  m.CovarianceMatrix[7],
				CovarianceMatrixNumber9:  m.CovarianceMatrix[8],
				CovarianceMatrixNumber10: m.CovarianceMatrix[9],
				CovarianceMatrixNumber11: m.CovarianceMatrix[10],
				CovarianceMatrixNumber12: m.CovarianceMatrix[11],
				CovarianceMatrixNumber13: m.CovarianceMatrix[12],
				CovarianceMatrixNumber14: m.CovarianceMatrix[13],
				CovarianceMatrixNumber15: m.CovarianceMatrix[14],
				CovarianceMatrixNumber16: m.CovarianceMatrix[15],
				CovarianceMatrixNumber17: m.CovarianceMatrix[16],
				CovarianceMatrixNumber18: m.CovarianceMatrix[17],
				CovarianceMatrixNumber19: m.CovarianceMatrix[18],
				CovarianceMatrixNumber20: m.CovarianceMatrix[19],
				CovarianceMatrixNumber21: m.CovarianceMatrix[20],
				CovarianceMatrixNumber22: m.CovarianceMatrix[21],
				CovarianceMatrixNumber23: m.CovarianceMatrix[22],
				CovarianceMatrixNumber24: m.CovarianceMatrix[23],
				CovarianceMatrixNumber25: m.CovarianceMatrix[24],
			}

			chIncomingMsg <- newMsg

			// One robots sends about 30 messages per second. Uncomment the following lines to see the messages.

			//fmt.Printf("Id: %d, x: %d, y: %d, theta: %d, ir1x: %d, ir1y: %d, ir2x: %d, ir2y: %d, ir3x: %d, ir3y: %d, ir4x: %d, ir4y: %d\n", newMsg.id, newMsg.x, newMsg.y, newMsg.theta, newMsg.ir1x, newMsg.ir1y, newMsg.ir2x, newMsg.ir2y, newMsg.ir3x, newMsg.ir3y, newMsg.ir4x, newMsg.ir4y)
			//log.GGeneralLogger.Printf("Id: %d, x: %d, y: %d, theta: %d, ir1x: %d, ir1y: %d, ir2x: %d, ir2y: %d, ir3x: %d, ir3y: %d, ir4x: %d, ir4y: %d\n", newMsg.id, newMsg.x, newMsg.y, newMsg.theta, newMsg.ir1x, newMsg.ir1y, newMsg.ir2x, newMsg.ir2y, newMsg.ir3x, newMsg.ir3y, newMsg.ir4x, newMsg.ir4y)
			//fmt.Printf("Id: %d, x: %d, y: %d, theta: %d\n", newMsg.id, newMsg.x, newMsg.y, newMsg.theta)
			//log.GGeneralLogger.Printf("Id: %d, x: %d, y: %d, theta: %d\n", newMsg.id, newMsg.x, newMsg.y, newMsg.theta)
		}
	}
}

func Subscribe(
	client mqtt.Client,
	chIncomingMsg chan<- types.AdvMsg,
) {
	topic := "v2/robot/NRF_5/adv"
	token := client.Subscribe(topic, 1, advMessageHandler(chIncomingMsg))
	token.Wait()
	fmt.Printf("Subscribed to topic: %s", topic)
	log.GGeneralLogger.Println("Subscribed to topic: ", topic)
}
