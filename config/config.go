package config

// MQTT
//"broker.emqx.io" can be used for testing. The program does not run unless it connects to a broker.
const Broker = "slam" //"broker.emqx.io"
const Port = 1883

// MAP
const MapSize = 400            //cm, 400x400 squares
const MapCenterX = MapSize / 2 //cm (origin is at the top left corner)
const MapCenterY = MapSize / 2 //cm (origin is at the top left corner)

// ROBOT
const IrSensorMaxDistance = 50 //cm

// GUI
const GuiFrameRate = 5            //fps
const MapMinimumDisplaySize = 400 //px
const WindowBreadth = 650         //px
const WindowHeight = 400          //px
