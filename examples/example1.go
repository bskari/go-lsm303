package lsm303

import (
	"fmt"
	"log"
	"time"
	lsm303 "github.com/bskari/go-lsm303"
)

func main() {
	accelerometer, err := NewLsm303Accelerometer()
	if err != nil {
		log.Fatal("Couldn't connect to accelerometer")
	}
	for {
		x, y, z := accelerometer.Read()
		fmt.Printf("x:%v y:%v z:%v\n", x, y, z)
		time.Sleep(time.Second * 1)
	}
}
