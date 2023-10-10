package main

import (
	"fmt"
	"time"

	"fyne.io/fyne/v2"
	mqtt "github.com/eclipse/paho.mqtt.golang"
)

func scale_and_stuff(base, num int16) int16 {
	return int16(float64(num)/10 + float64(base)) //it gives distance data, not position data.
	//since it gives distance relative to the body so it must be rotated when the body is. A rotation matrix would get the job done.
}

func main() {
	ch_publish := make(chan [2]int16)
	ch_receive := make(chan adv_msg)
	window, map_img, img_init, robot, rob_img_stack := GUI(ch_publish, ch_receive)
	map_img.Refresh()
	robot.Resize(fyne.NewSize(20, 20))
	robot.Refresh()
	robot.Move(fyne.NewPos(200, 200))

	robot_pos := fyne.NewPos(190, 190)
	go func() {
		for msg := range ch_receive {
			robot_pos = robot_pos.AddXY(float32(msg.x)/10, float32(msg.y)/10)

			img_init = draw_line(img_init, int(robot_pos.X)+10, int(robot_pos.Y)+10, int(scale_and_stuff(int16(robot_pos.X+10), msg.ir1x)), int(scale_and_stuff(int16(robot_pos.Y+10), msg.ir1y)))
			img_init = draw_line(img_init, int(robot_pos.X)+10, int(robot_pos.Y)+10, int(scale_and_stuff(int16(robot_pos.X+10), msg.ir2x)), int(scale_and_stuff(int16(robot_pos.Y+10), msg.ir2y)))
			img_init = draw_line(img_init, int(robot_pos.X)+10, int(robot_pos.Y)+10, int(scale_and_stuff(int16(robot_pos.X+10), msg.ir3x)), int(scale_and_stuff(int16(robot_pos.Y+10), msg.ir3y)))
			img_init = draw_line(img_init, int(robot_pos.X)+10, int(robot_pos.Y)+10, int(scale_and_stuff(int16(robot_pos.X+10), msg.ir4x)), int(scale_and_stuff(int16(robot_pos.Y+10), msg.ir4y)))

		}
	}()
	go func() {
		for range time.Tick(time.Second) {
			rob_img_stack.Refresh()
			robot.Resize(fyne.NewSize(20, 20))
			robot.Move(robot_pos)
			//print("Robot X position:", int(robot.Position1.X), "Robot Y position:", int(robot.Position1.Y))
		}
	}()

	var broker = "slam" //"broker.emqx.io"
	var port = 1883
	opts := mqtt.NewClientOptions()
	opts.AddBroker(fmt.Sprintf("tcp://%s:%d", broker, port))
	opts.SetClientID("go_mqtt_client")
	opts.SetUsername("emqx")
	opts.SetPassword("public")
	opts.SetDefaultPublishHandler(messagePubHandler(ch_receive))
	opts.OnConnect = connectHandler
	opts.OnConnectionLost = connectLostHandler
	client := mqtt.NewClient(opts)
	if token := client.Connect(); token.Wait() && token.Error() != nil {
		panic(token.Error())
	}

	sub(client)
	go publish(client, ch_publish)

	window.ShowAndRun()

}
