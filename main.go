package main

import (
	"fmt"
	"math"
	"time"

	"fyne.io/fyne/v2"
	mqtt "github.com/eclipse/paho.mqtt.golang"
)

// func scale_and_stuff(base, num int16) int16 {
// 	return int16(float64(num)/10 + float64(base)) //it gives distance data, not position data.
// 	//since it gives distance relative to the body so it must be rotated when the body is. A rotation matrix would get the job done.
// }

func scale_rotate_transelate(base, x_body, y_body, theta int16) (int, int) {
	//Ir data is given in mm and it is relative to the body, so it must be rotated in order to draw correctly in the map

	theta_rad := float64(theta) * math.Pi / 180
	//rotate
	x_map := float64(x_body)*math.Cos(-theta_rad) - float64(y_body)*math.Sin(-theta_rad)
	y_map := float64(x_body)*math.Sin(-theta_rad) + float64(y_body)*math.Cos(-theta_rad)

	//scale and transelate
	return int(float64(x_map)/10 + float64(base)), int(float64(y_map)/10 + float64(base)) //it gives distance data, not position data.
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
	robot_theta := 180
	//prev_update := time.Now()
	go func() {
		for msg := range ch_receive {
			robot_pos = robot_pos.AddXY(float32(msg.x)/10, -float32(msg.y)/10)
			robot_theta += int(msg.theta)
			println("Theta msg:", msg.theta, "robot theta:", robot_theta, "Robot pos:", robot_pos.X, robot_pos.Y)

			x, y := scale_rotate_transelate(int16(robot_pos.X+10), msg.ir1x, msg.ir1y, int16(robot_theta))
			img_init = draw_line(img_init, int(robot_pos.X)+10, int(robot_pos.Y)+10, x, y)

			x, y = scale_rotate_transelate(int16(robot_pos.X+10), msg.ir2x, msg.ir2y, int16(robot_theta))
			img_init = draw_line(img_init, int(robot_pos.X)+10, int(robot_pos.Y)+10, x, y)

			x, y = scale_rotate_transelate(int16(robot_pos.X+10), msg.ir3x, msg.ir3y, int16(robot_theta))
			img_init = draw_line(img_init, int(robot_pos.X)+10, int(robot_pos.Y)+10, x, y)

			x, y = scale_rotate_transelate(int16(robot_pos.X+10), msg.ir4x, msg.ir4y, int16(robot_theta))
			img_init = draw_line(img_init, int(robot_pos.X)+10, int(robot_pos.Y)+10, x, y)

			// img_init = draw_line(img_init, int(robot_pos.X)+10, int(robot_pos.Y)+10, int(scale_and_stuff(int16(robot_pos.X+10), msg.ir1x)), int(scale_and_stuff(int16(robot_pos.Y+10), msg.ir1y)))
			// img_init = draw_line(img_init, int(robot_pos.X)+10, int(robot_pos.Y)+10, int(scale_and_stuff(int16(robot_pos.X+10), msg.ir2x)), int(scale_and_stuff(int16(robot_pos.Y+10), msg.ir2y)))
			// img_init = draw_line(img_init, int(robot_pos.X)+10, int(robot_pos.Y)+10, int(scale_and_stuff(int16(robot_pos.X+10), msg.ir3x)), int(scale_and_stuff(int16(robot_pos.Y+10), msg.ir3y)))
			// img_init = draw_line(img_init, int(robot_pos.X)+10, int(robot_pos.Y)+10, int(scale_and_stuff(int16(robot_pos.X+10), msg.ir4x)), int(scale_and_stuff(int16(robot_pos.Y+10), msg.ir4y)))

		}
	}()
	go func() {
		for range time.Tick(50 * time.Millisecond) {
			rob_img_stack.Refresh()
			robot.Resize(fyne.NewSize(20, 20))
			robot.Move(robot_pos)
			//print("Robot X position:", int(robot.Position1.X), "Robot Y position:", int(robot.Position1.Y))
			//print("Time since previous update", time.Since(prev_update).Milliseconds())
			//prev_update = time.Now()
		}

	}()

	var broker = "broker.emqx.io"
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
