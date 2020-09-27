package lsm303

import (
	"fmt"
	"log"
	"time"
	lsm303 "github.com/bskari/go-lsm303"
)

func main() {
	accelerometer, err := NewAccelerometer()
	if err != nil {
		log.Fatal("Couldn't connect to accelerometer")
	}

	magnetometer, err != NewMagnetometer()
	if err != nil {
		log.Fatal("Couldn't connect to magnetometer")
	}

	for {
		xa, ya, za := accelerometer.Sense()
		fmt.Printf("accel x:%v y:%v z:%v\n", xa, ya, za)
		xa, ya, za = accelerometer.SenseRaw()
		fmt.Printf("raw accel x:%v y:%v z:%v\n", xa, ya, za)

		// The periph.io has units defined for many things, but not for
		// magnetometer flux, so we only have SenseRaw
		xm, ym, zm := magnetometer.SenseRaw()
		fmt.Printf("raw mag x:%v y:%v z:%v\n", xm, ym, zm)

		time.Sleep(time.Second * 1)
	}

	// Examples for setting options
	/*
	accelerometer.SetRange(ACCELEROMETER_RANGE_16G)
	accelerometer.SetMode(ACCELEROMETER_MODE_LOW_POWER)
	magnetometer.SetGain(MAGNETOMETER_GAIN_5_6)
	magnetometer.SetRate(MAGNETOMETER_RATE_75)
	*/
}
