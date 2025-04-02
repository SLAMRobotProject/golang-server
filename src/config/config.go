package config

// MQTT
//"broker.emqx.io" can be used for testing. The program does not run unless it connects to a broker.
const Broker = "slam" //"broker.emqx.io"
const Port = 1883

// MAP
const MapSize = 400            //cm, 400x400 squares
const MapCenterX = MapSize / 2 //cm (origin is at the top left corner)
const MapCenterY = MapSize / 2 //cm (origin is at the top left corner)
const (
	MapOpen     uint8 = 1 << iota //1
	MapUnknown                    //2
	MapObstacle                   //4
)

// ROBOT
const IrSensorMaxDistance = 50 //cm

// GUI
const GuiFrameRate = 5            //fps
const MapMinimumDisplaySize = 400 //px
const WindowBreadth = 650         //px
const WindowHeight = 400          //px

// MAPPING
const InitialSquareLength = 25	//cm, INITIAL_RECTANGLE_SIDELENGTH in robot code

// PATHFINDING
const PathUpdateIntervalSec = 5 // Seconds between each A* path calculation
const InitialStartPositionX = 0 // cm, the robot will always start at this position (should be change in the future to be dynamic for different start positions for each robot)
const InitialStartPositionY = 0 // cm, the robot will always start at this position (should be change in the future to be dynamic for different start positions for each robot)