package main

import (
	"log"
	"os"
)

func position_logger_init() *log.Logger {
	// Flags: Create if needed, write only, remove contents. 0666 is read/write permission for everyone.
	file, err := os.OpenFile("positions.csv", os.O_CREATE|os.O_WRONLY|os.O_TRUNC, 0666)
	if err != nil {
		log.Fatal(err)
	}
	var logger = log.New(file, "", 0)
	logger.Println("time,id,x,y,theta (the delimiter is a space)")
	logger.SetFlags(log.Ltime | log.Lmicroseconds)
	return logger
}
