package interfaces

import (
	"errors"
	"os"
)

func openCommPort(ttyName string, rate uint32, timeout time.Duration) (*os.File, error) {
	return nil, errors.New("not implemented")
}
