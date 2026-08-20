package interfaces

type CommPort interface {
	Read([]byte) (int, error)
	Write([]byte) error
	Close()
}
