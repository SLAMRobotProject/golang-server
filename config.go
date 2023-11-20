package main

// MQTT
//"broker.emqx.io" can be used for testing. The program does not run unless it connects to a BROKER.
const BROKER = "broker.emqx.io"
const PORT = 1883

// MAP
const MAP_SIZE = 400              //cm, 400x400 squares
const MAP_CENTER_X = MAP_SIZE / 2 //cm (origin is at the top left corner)
const MAP_CENTER_Y = MAP_SIZE / 2 //cm (origin is at the top left corner)

// ROBOT
const IR_SENSOR_MAX_DISTANCE = 50 //cm

// GUI
const GUI_FRAME_RATE = 5             //fps
const MAP_MINIMUM_DISPLAY_SIZE = 400 //px
const WINDOW_BREADTH = 650           //px
const WINDOW_HEIGHT = 400            //px
