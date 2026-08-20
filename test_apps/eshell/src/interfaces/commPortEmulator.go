package interfaces

import "time"

type SerialPortEmulator struct {
	command string
}

func NewSerialPortEmulator() *SerialPortEmulator {
	return &SerialPortEmulator{}
}

func (e *SerialPortEmulator) Read(data []byte) (int, error) {
	if len(e.command) == 0 {
		time.Sleep(100 * time.Millisecond)
		return 0, nil
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
