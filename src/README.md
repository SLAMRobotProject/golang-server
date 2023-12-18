# golang-server
This is a golang server based on communication via MQTT. It is made to be compatible with the robots from the SLAM robot project at NTNU. 

## How to run
Prerequisites: 
- Ensure that the robots are using the correct version of the robot code, which is currently located in the golang-server branch of the robot code.
- Ensure that the broker specified in ./config/config.go can be reached on the network. The code quits if it cannot connect to a broker, see the log file for confirmation.

Running from source:
1. Clone/download this repository.
1. Follow the Fyne (GUI package) [installation guide](https://developer.fyne.io/started/)
1. Open a terminal, navigate to this directory (the *src* directory) and run with `go run .` or build with `go build .`
   
The executable was created with `broker="slam"` meaning that it will only connect to the physical Raspberry Pi broker. Connecting to the Raspberry Pi broker can be done by connecting your computer to the Raspberry Pi WiFi: `BorderRouter-AP` with password `12345678`.

