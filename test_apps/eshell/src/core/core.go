package core

import (
	"errors"
	"eshell/src/interfaces"
	"strings"
)

func Init(deviceName string, baudRate int) (*EShell, error) {
	var commPort interfaces.CommPort
	var err error
	if strings.HasPrefix(deviceName, "/dev/tty") {
		commPort, err = interfaces.NewSerialCommPort(deviceName, baudRate)
	} else {
		if deviceName == "emulator" {
			commPort = interfaces.NewSerialPortEmulator()
			err = nil
		} else {
			err = errors.New("unknown device name")
		}
	}
	if err != nil {
		return nil, err
	}
	return NewEShell(commPort), nil
}
