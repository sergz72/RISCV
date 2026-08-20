package main

import (
	"eshell/src/core"
	"fmt"
	"log"
	"os"
	"strconv"
	"strings"
)

func main() {
	var deviceName string
	var baudRate int
	var err error
	var args []string

	switch len(os.Args) {
	case 1:
		fmt.Println("Usage: eshell device_name [baud_rate]")
		return
	case 2:
		deviceName = os.Args[1]
		baudRate = 115200
	default:
		deviceName = os.Args[1]
		baudRate, err = strconv.Atoi(os.Args[2])
		if err != nil {
			log.Fatal(err)
		}
		args = os.Args[3:]
	}

	shell, err := core.Init(deviceName, baudRate)
	if err != nil {
		log.Fatal(err)
	}
	defer shell.Close()
	err = shell.Run(strings.Join(args, " "))
	if err != nil {
		log.Fatal(err)
	}
}
