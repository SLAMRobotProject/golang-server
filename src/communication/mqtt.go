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
	id                                                                                                                               int8
	x, y, theta, ir1x, ir1y, ir2x, ir2y, ir3x, ir3y, ir4x, ir4y                                                                      int16
	covarianceMatrixNumber1, covarianceMatrixNumber2, covarianceMatrixNumber3, covarianceMatrixNumber4, covarianceMatrixNumber5      float32
	covarianceMatrixNumber6, covarianceMatrixNumber7, covarianceMatrixNumber8, covarianceMatrixNumber9, covarianceMatrixNumber10     float32
	covarianceMatrixNumber11, covarianceMatrixNumber12, covarianceMatrixNumber13, covarianceMatrixNumber14, covarianceMatrixNumber15 float32
	covarianceMatrixNumber16, covarianceMatrixNumber17, covarianceMatrixNumber18, covarianceMatrixNumber19, covarianceMatrixNumber20 float32
	covarianceMatrixNumber21, covarianceMatrixNumber22, covarianceMatrixNumber23, covarianceMatrixNumber24, covarianceMatrixNumber25 float32
	valid                                                                                                                            bool
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
		// NOTE, float in C = float32 in Go!
		if len(payload) == 124 { // 24 bytes for the initial fields + 25*4 bytes for the covariance matrix (each float32 is 4 bytes)
			m := advMsgUnpacking{}
			binary.Read(reader, binary.LittleEndian, &m.id)    //1
			binary.Read(reader, binary.LittleEndian, &m.x)     //2
			binary.Read(reader, binary.LittleEndian, &m.y)     //2
			binary.Read(reader, binary.LittleEndian, &m.theta) //2
			binary.Read(reader, binary.LittleEndian, &m.ir1x)  //2
			binary.Read(reader, binary.LittleEndian, &m.ir1y)  //2
			binary.Read(reader, binary.LittleEndian, &m.ir2x)  //2
			binary.Read(reader, binary.LittleEndian, &m.ir2y)  //2
			binary.Read(reader, binary.LittleEndian, &m.ir3x)  //2
			binary.Read(reader, binary.LittleEndian, &m.ir3y)  //2
			binary.Read(reader, binary.LittleEndian, &m.ir4x)  //2
			binary.Read(reader, binary.LittleEndian, &m.ir4y)  //2

			binary.Read(reader, binary.LittleEndian, &m.covarianceMatrixNumber1)  //4 //covarianceMatrixNumber is not used in the robot code
			binary.Read(reader, binary.LittleEndian, &m.covarianceMatrixNumber2)  //4
			binary.Read(reader, binary.LittleEndian, &m.covarianceMatrixNumber3)  //4
			binary.Read(reader, binary.LittleEndian, &m.covarianceMatrixNumber4)  //4
			binary.Read(reader, binary.LittleEndian, &m.covarianceMatrixNumber5)  //4
			binary.Read(reader, binary.LittleEndian, &m.covarianceMatrixNumber6)  //4
			binary.Read(reader, binary.LittleEndian, &m.covarianceMatrixNumber7)  //4
			binary.Read(reader, binary.LittleEndian, &m.covarianceMatrixNumber8)  //4
			binary.Read(reader, binary.LittleEndian, &m.covarianceMatrixNumber9)  //4
			binary.Read(reader, binary.LittleEndian, &m.covarianceMatrixNumber10) //4
			binary.Read(reader, binary.LittleEndian, &m.covarianceMatrixNumber11) //4
			binary.Read(reader, binary.LittleEndian, &m.covarianceMatrixNumber12) //4
			binary.Read(reader, binary.LittleEndian, &m.covarianceMatrixNumber13) //4
			binary.Read(reader, binary.LittleEndian, &m.covarianceMatrixNumber14) //4
			binary.Read(reader, binary.LittleEndian, &m.covarianceMatrixNumber15) //4
			binary.Read(reader, binary.LittleEndian, &m.covarianceMatrixNumber16) //4
			binary.Read(reader, binary.LittleEndian, &m.covarianceMatrixNumber17) //4
			binary.Read(reader, binary.LittleEndian, &m.covarianceMatrixNumber18) //4
			binary.Read(reader, binary.LittleEndian, &m.covarianceMatrixNumber19) //4
			binary.Read(reader, binary.LittleEndian, &m.covarianceMatrixNumber20) //4
			binary.Read(reader, binary.LittleEndian, &m.covarianceMatrixNumber21) //4
			binary.Read(reader, binary.LittleEndian, &m.covarianceMatrixNumber22) //4
			binary.Read(reader, binary.LittleEndian, &m.covarianceMatrixNumber23) //4
			binary.Read(reader, binary.LittleEndian, &m.covarianceMatrixNumber24) //4
			binary.Read(reader, binary.LittleEndian, &m.covarianceMatrixNumber25) //4

			binary.Read(reader, binary.LittleEndian, &m.valid) //1 //valid is not used in the robot code

			newMsg := types.AdvMsg{
				Id:                       int(m.id),
				X:                        int(m.x),
				Y:                        int(m.y),
				Theta:                    int(m.theta),
				Ir1x:                     int(m.ir1x),
				Ir1y:                     int(m.ir1y),
				Ir2x:                     int(m.ir2x),
				Ir2y:                     int(m.ir2y),
				Ir3x:                     int(m.ir3x),
				Ir3y:                     int(m.ir3y),
				Ir4x:                     int(m.ir4x),
				Ir4y:                     int(m.ir4y),
				CovarianceMatrixNumber1:  float32(m.covarianceMatrixNumber1),
				CovarianceMatrixNumber2:  float32(m.covarianceMatrixNumber2),
				CovarianceMatrixNumber3:  float32(m.covarianceMatrixNumber3),
				CovarianceMatrixNumber4:  float32(m.covarianceMatrixNumber4),
				CovarianceMatrixNumber5:  float32(m.covarianceMatrixNumber5),
				CovarianceMatrixNumber6:  float32(m.covarianceMatrixNumber6),
				CovarianceMatrixNumber7:  float32(m.covarianceMatrixNumber7),
				CovarianceMatrixNumber8:  float32(m.covarianceMatrixNumber8),
				CovarianceMatrixNumber9:  float32(m.covarianceMatrixNumber9),
				CovarianceMatrixNumber10: float32(m.covarianceMatrixNumber10),
				CovarianceMatrixNumber11: float32(m.covarianceMatrixNumber11),
				CovarianceMatrixNumber12: float32(m.covarianceMatrixNumber12),
				CovarianceMatrixNumber13: float32(m.covarianceMatrixNumber13),
				CovarianceMatrixNumber14: float32(m.covarianceMatrixNumber14),
				CovarianceMatrixNumber15: float32(m.covarianceMatrixNumber15),
				CovarianceMatrixNumber16: float32(m.covarianceMatrixNumber16),
				CovarianceMatrixNumber17: float32(m.covarianceMatrixNumber17),
				CovarianceMatrixNumber18: float32(m.covarianceMatrixNumber18),
				CovarianceMatrixNumber19: float32(m.covarianceMatrixNumber19),
				CovarianceMatrixNumber20: float32(m.covarianceMatrixNumber20),
				CovarianceMatrixNumber21: float32(m.covarianceMatrixNumber21),
				CovarianceMatrixNumber22: float32(m.covarianceMatrixNumber22),
				CovarianceMatrixNumber23: float32(m.covarianceMatrixNumber23),
				CovarianceMatrixNumber24: float32(m.covarianceMatrixNumber24),
				CovarianceMatrixNumber25: float32(m.covarianceMatrixNumber25),
				// CovarianceMatrix: m.covarianceMatrix,
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
