package main

// MQTT
//"broker.emqx.io" can be used for testing. The program does not run unless it connects to a broker.
const broker = "slam" //"broker.emqx.io"
const port = 1883

// MAP
const map_size = 400              //cm, 400x400 squares
const map_center_x = map_size / 2 //cm (origin is at the top left corner)
const map_center_y = map_size / 2 //cm (origin is at the top left corner)

// ROBOT
const irSensor_maxDistance = 50 //cm

// GUI
const gui_frame_rate = 5             //fps
const map_minimum_display_size = 400 //px
const window_breadth = 650           //px
const window_height = 400            //px
