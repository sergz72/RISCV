package core

import (
	"bufio"
	"encoding/base64"
	"errors"
	"eshell/src/interfaces"
	"fmt"
	"io"
	"os"
	"strings"
	"sync"
	"time"
)

type EShell struct {
	commPort interfaces.CommPort
	response string
	lock     sync.Mutex
}

func NewEShell(port interfaces.CommPort) *EShell {
	return &EShell{commPort: port}
}

func (e *EShell) Reader() {
	buffer := make([]byte, 1024)
	var err error
	for {
		var n int
		n, err = e.commPort.Read(buffer)
		if err != nil && err != io.EOF {
			break
		}
		if n != 0 {
			for i := 0; i < n; i++ {
				if buffer[i] > 0x7F {
					buffer[i] = '\n'
				}
			}
			response := string(buffer[:n])
			fmt.Print(response)
			e.lock.Lock()
			e.response += response
			e.lock.Unlock()
		}
	}
	fmt.Printf("Reader error %v\n", err)
}

func (e *EShell) send(text string) error {
	e.lock.Lock()
	e.response = ""
	e.lock.Unlock()
	text += "\r"
	return e.commPort.Write([]byte(text))
}

func (e *EShell) sendWithEcho(text string) error {
	fmt.Println(text)
	return e.send(text)
}

func (e *EShell) waitForResponse() error {
	counter := 0
	for len(e.response) == 0 {
		time.Sleep(100 * time.Millisecond)
		counter++
		if counter >= 10 {
			return errors.New("read timeout")
		}
	}
	return nil
}

func (e *EShell) waitForOk() error {
	err := e.waitForResponse()
	if err != nil {
		return err
	}
	if strings.HasPrefix(e.response, "OK") {
		return nil
	}
	return fmt.Errorf("bad response with length %d \"%s\"", len(e.response), e.response)
}

func (e *EShell) sendFileData(data []byte) error {
	l := len(data)
	idx := 0
	for l > 0 {
		var ll int
		ll = min(l, 50)
		var writer strings.Builder
		encoder := base64.NewEncoder(base64.StdEncoding, &writer)
		_, err := encoder.Write(data[idx : idx+ll])
		if err != nil {
			_ = encoder.Close()
			return err
		}
		_ = encoder.Close()
		err = e.sendWithEcho("fwrite " + writer.String())
		if err != nil {
			return err
		}
		err = e.waitForOk()
		if err != nil {
			return err
		}
		l -= ll
		idx += ll
	}
	return nil
}

func (e *EShell) getFileData() ([]byte, error) {
	var result []byte

	for {
		err := e.sendWithEcho("fread 50")
		if err != nil {
			return nil, err
		}
		err = e.waitForResponse()
		if err != nil {
			return nil, err
		}
		decoder := base64.NewDecoder(base64.StdEncoding, strings.NewReader(e.response))
		buffer := make([]byte, 60)
		n, err := decoder.Read(buffer)
		if err != nil {
			return nil, err
		}
		if n == 0 {
			break
		}
		result = append(result, buffer[:n]...)
		if n < 50 {
			break
		}
	}

	return result, nil
}

func (e *EShell) fileGet(parts []string) error {
	if len(parts) != 2 {
		return errors.New("invalid get argument count")
	}
	err := e.sendWithEcho("fopen " + parts[0] + " r")
	if err != nil {
		return err
	}
	err = e.waitForOk()
	if err != nil {
		return err
	}
	data, err := e.getFileData()
	if err != nil {
		_ = e.sendWithEcho("fclose")
		return err
	}
	err = e.sendWithEcho("fclose")
	if err != nil {
		return err
	}
	return os.WriteFile(parts[1], data, 0666)
}

func (e *EShell) fileSend(parts []string) error {
	if len(parts) != 2 {
		return errors.New("invalid send argument count")
	}
	data, err := os.ReadFile(parts[0])
	if err != nil {
		return err
	}
	err = e.sendWithEcho("fopen " + parts[1] + " w")
	if err != nil {
		return err
	}
	err = e.waitForOk()
	if err != nil {
		return err
	}
	err = e.sendFileData(data)
	if err != nil {
		_ = e.sendWithEcho("fclose")
		return err
	}
	return e.sendWithEcho("fclose")
}

func (e *EShell) Run(command string) error {
	go e.Reader()
	commands := []string{"echo off", command}
	idx := 0
	reader := bufio.NewReader(os.Stdin)
	for {
		fmt.Print("# ")
		var input string
		var err error
		if idx < len(commands) {
			fmt.Println(commands[idx])
			input = commands[idx]
		} else {
			input, err = reader.ReadString('\n')
			if err != nil {
				return err
			}
			input = input[:len(input)-1]
		}
		parts := strings.Split(input, " ")
		if len(parts) != 0 {
			switch parts[0] {
			case "send":
				err = e.fileSend(parts[1:])
			case "get":
				err = e.fileGet(parts[1:])
			default:
				err = e.send(input)
			}
			if err != nil {
				return err
			}
		}
		if idx < len(commands) {
			_ = e.waitForResponse()
			idx++
		}
	}
}

func (p *EShell) Close() {
	p.commPort.Close()
}
