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

type coordinate struct {
	x int16
	y int16
}

// Must use the correct amount of bytes for each data type.
type advMsgUnpacking struct {
	id      uint8
	x       int16
	y       int16
	theta   int16
	accel_x float32
	accel_y float32
	// accel_z      float32
	// gyro_x       float32
	// gyro_y       float32
	gyro_z       float32
	ir           [4]coordinate
	covMatrix    [25]float32 //not used currently, but needed to keep the byte size correct
	valid        uint8
	iRTowerAngle uint8
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

var lastsize int = 0

func advMessageHandler(
	chIncomingMsg chan<- types.AdvMsg,
	chSensorData chan<- types.SensorData,
) mqtt.MessageHandler {
	return func(client mqtt.Client, msg mqtt.Message) {
		payload := msg.Payload()
		reader := bytes.NewReader(payload)

		if len(payload) != lastsize {
			println("Incoming payload is: ", len(payload))
			lastsize = len(payload)
		}

		if len(payload) == 137 { //expected size
			m := advMsgUnpacking{}
			binary.Read(reader, binary.LittleEndian, &m.id)
			binary.Read(reader, binary.LittleEndian, &m.x)
			binary.Read(reader, binary.LittleEndian, &m.y)
			binary.Read(reader, binary.LittleEndian, &m.theta)
			binary.Read(reader, binary.LittleEndian, &m.accel_x)
			binary.Read(reader, binary.LittleEndian, &m.accel_y)
			// binary.Read(reader, binary.LittleEndian, &m.accel_z)
			// binary.Read(reader, binary.LittleEndian, &m.gyro_x)
			// binary.Read(reader, binary.LittleEndian, &m.gyro_y)
			binary.Read(reader, binary.LittleEndian, &m.gyro_z)
			binary.Read(reader, binary.LittleEndian, &m.ir[0].x)
			binary.Read(reader, binary.LittleEndian, &m.ir[0].y)
			binary.Read(reader, binary.LittleEndian, &m.ir[1].x)
			binary.Read(reader, binary.LittleEndian, &m.ir[1].y)
			binary.Read(reader, binary.LittleEndian, &m.ir[2].x)
			binary.Read(reader, binary.LittleEndian, &m.ir[2].y)
			binary.Read(reader, binary.LittleEndian, &m.ir[3].x)
			binary.Read(reader, binary.LittleEndian, &m.ir[3].y)
			binary.Read(reader, binary.LittleEndian, &m.covMatrix)
			binary.Read(reader, binary.LittleEndian, &m.valid)
			binary.Read(reader, binary.LittleEndian, &m.iRTowerAngle)

			// Debug: Print the raw payload in hex format
			// fmt.Print("Payload hex: ")
			// for i, b := range payload {
			// 	fmt.Printf("%02X ", b)
			// 	if (i+1)%16 == 0 {
			// 		fmt.Println()
			// 	}
			// }
			// fmt.Println()

			newMsg := types.AdvMsg{
				Id:    int(m.id),
				X:     int(m.x),
				Y:     int(m.y),
				Theta: int(m.theta),
				Ir1x:  int(m.ir[0].x),
				Ir1y:  int(m.ir[0].y),
				Ir2x:  int(m.ir[1].x),
				Ir2y:  int(m.ir[1].y),
				Ir3x:  int(m.ir[2].x),
				Ir3y:  int(m.ir[2].y),
				Ir4x:  int(m.ir[3].x),
				Ir4y:  int(m.ir[3].y),
			}

			newSensorData := types.SensorData{
				Id: int32(m.id),
				Odometry: types.Odometry{
					X:     float64(m.x),
					Y:     float64(m.y),
					Theta: float64(m.theta),
				},
				Imu: types.Imu{
					AccelX: float64(m.accel_x),
					AccelY: float64(m.accel_y),
					GyroZ:  float64(m.gyro_z),
				},
				IRSensors: types.IRSensors{
					IrSensorData_0: types.Point2D{
						X: int32(m.ir[0].x), Y: int32(m.ir[0].y),
					},
					IrSensorData_1: types.Point2D{
						X: int32(m.ir[1].x), Y: int32(m.ir[1].y),
					},
					IrSensorData_2: types.Point2D{
						X: int32(m.ir[2].x), Y: int32(m.ir[2].y),
					},
					IrSensorData_3: types.Point2D{
						X: int32(m.ir[3].x), Y: int32(m.ir[3].y),
					},
				},
				IRTowerAngle: int32(m.iRTowerAngle),
			}

			chIncomingMsg <- newMsg
			chSensorData <- newSensorData

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
	chSensorData chan<- types.SensorData,
) {
	topic := "v2/robot/NRF_5/adv"
	token := client.Subscribe(topic, 1, advMessageHandler(chIncomingMsg, chSensorData))
	token.Wait()
	fmt.Printf("Subscribed to topic: %s", topic)
	log.GGeneralLogger.Println("Subscribed to topic: ", topic)
}
