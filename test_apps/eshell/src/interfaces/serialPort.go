package interfaces

import (
	"errors"
	"os"
	"time"
)

type SerialCommPort struct {
	handle *os.File
}

func NewSerialCommPort(deviceName string, baudRate int) (*SerialCommPort, error) {
	baud, ok := bauds[baudRate]
	if !ok {
		return nil, errors.New("unsupported baud rate")
	}
	handle, err := openCommPort(deviceName, baud, 100*time.Millisecond)
	if err != nil {
		return nil, err
	}
	return &SerialCommPort{handle: handle}, nil
}

func (p *SerialCommPort) Read(buffer []byte) (int, error) {
	return p.handle.Read(buffer)
}

func (p *SerialCommPort) Write(buffer []byte) error {
	_, err := p.handle.Write(buffer)
	return err
}

func (p *SerialCommPort) Close() {
	_ = p.handle.Close()
}
