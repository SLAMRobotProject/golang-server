package main

// MQTT
const broker = "slam" //"broker.emqx.io"
const port = 1883

// MAP
const map_size = 400                 //cm, 400x400 squares
const map_center_x = map_size / 2    //cm
const map_center_y = map_size / 2    //cm
const map_minimum_display_size = 400 //px

// ROBOT
const irSensor_maxDistance = 30 //cm

// GUI
const gui_frame_rate = 1 //fps
