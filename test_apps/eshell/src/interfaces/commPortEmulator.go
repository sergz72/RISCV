package interfaces

import (
	"strings"
	"time"
)

var fileData = [...]string{
	"TDoxMDA6LTY0Ckw6MzAwOi02MQpMOjUwMDotNjQKTDo3MDA6LTY4Ckw6MTAwMDotNjQ=\n",
	"Ckw6MTMwMDotNDUKTDoxNjAwOi00NQpMOjIwMDA6LTM1Ckw6MjUwMDotMjYKTDozMDA=\n",
	"MDotMjAKTDozNTAwOi0xMgpMOjQwMDA6LTEyCkw6NDUwMDotOQpMOjUwMDA6LTkKUjo=\n",
	"MTAwOi01NApSOjMwMDotNTMKUjo1MDA6LTY0ClI6NzAwOi02MQpSOjEwMDA6LTU2ClI=\n",
	"OjEzMDA6LTI4ClI6MTYwMDotMjEKUjoyMDAwOjAKUjoyNTAwOjAKUjozMDAwOjAKUjo=\n",
	"MzUwMDowClI6NDAwMDowClI6NDUwMDowClI6NTAwMDowCg==\n",
}

type SerialPortEmulator struct {
	command       string
	fileDataIndex int
}

func NewSerialPortEmulator() *SerialPortEmulator {
	return &SerialPortEmulator{}
}

func (e *SerialPortEmulator) Read(data []byte) (int, error) {
	if len(e.command) == 0 {
		time.Sleep(100 * time.Millisecond)
		return 0, nil
	}
	if strings.HasPrefix(e.command, "fopen ") {
		e.fileDataIndex = 0
	} else if strings.HasPrefix(e.command, "fread ") {
		e.command = ""
		if e.fileDataIndex >= len(fileData) {
			data[0] = 'E'
			data[1] = 'r'
			data[2] = 'r'
			data[3] = '\n'
			return 4, nil
		}
		bytes := []byte(fileData[e.fileDataIndex])
		e.fileDataIndex++
		copy(data, bytes)
		return len(bytes), nil
	}
	e.command = ""
	data[0] = 'O'
	data[1] = 'K'
	data[2] = '\n'
	return 3, nil
}

func (e *SerialPortEmulator) Write(data []byte) error {
	e.command = string(data)
	return nil
}

func (e *SerialPortEmulator) Close() {

}
