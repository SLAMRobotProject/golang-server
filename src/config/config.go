package config

// MQTT
//"broker.emqx.io" can be used for testing. The program does not run unless it connects to a broker.
const Broker = "slam"//"10.42.0.1"//"192.168.178.116"//"137.135.83.217"//"2003:d6:e710:9e00:ba27:ebff:fe79:41fa"// "137.135.83.217"//"10.42.0.1"//"64:ff9b:0:0:0:0:8efb:d18e"//"137.135.83.217"//"802.15.4"//"10.42.0.1"//broker.emqx.io"//"137.135.84.217"//"broker.emqx.io"//"10.42.0.1"//"slam" //"broker.emqx.io" // slam
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
