package lsm303

import (
	"encoding/binary"
	"periph.io/x/periph/conn/i2c"
	"periph.io/x/periph/conn/i2c/i2ctest"
	"periph.io/x/periph/conn/mmr"
	"periph.io/x/periph/conn/physic"
	"testing"
)

func TestGetMultiplier(t *testing.T) {
	// Go won't truncate const floats to int unless they're exactly
	// representable, so I had to manually compute these values and enter them
	// as const ints. Just make sure that they match.
	modes := [...]AccelerometerMode{
		ACCELEROMETER_MODE_LOW_POWER,
		ACCELEROMETER_MODE_NORMAL,
		ACCELEROMETER_MODE_HIGH_RESOLUTION,
	}
	ranges := [...]AccelerometerRange{
		ACCELEROMETER_RANGE_2G,
		ACCELEROMETER_RANGE_4G,
		ACCELEROMETER_RANGE_8G,
		ACCELEROMETER_RANGE_16G,
	}

	for _, mode := range modes {
		for _, range_ := range ranges {
			expectedValue := int64(getLsb_(mode, range_, t)*float64(physic.EarthGravity)) >> getShift_(mode, t)
			computedValue := int64(getMultiplier(mode, range_))
			if computedValue != expectedValue {
				t.Errorf("getMultiplier(%s, %s) should be %v but was %v", mode, range_, expectedValue, computedValue)
			}
		}
	}
}

// Gets the Least Significant Bit value for the current mode and range
func getLsb_(mode AccelerometerMode, range_ AccelerometerRange, t *testing.T) float64 {
	switch mode {
	case ACCELEROMETER_MODE_LOW_POWER:
		switch range_ {
		case ACCELEROMETER_RANGE_2G:
			return 0.01563
		case ACCELEROMETER_RANGE_4G:
			return 0.03126
		case ACCELEROMETER_RANGE_8G:
			return 0.06252
		case ACCELEROMETER_RANGE_16G:
			return 0.18758
		}
	case ACCELEROMETER_MODE_NORMAL:
		switch range_ {
		case ACCELEROMETER_RANGE_2G:
			return 0.0039
		case ACCELEROMETER_RANGE_4G:
			return 0.00782
		case ACCELEROMETER_RANGE_8G:
			return 0.01563
		case ACCELEROMETER_RANGE_16G:
			return 0.0469
		}

	case ACCELEROMETER_MODE_HIGH_RESOLUTION:
		switch range_ {
		case ACCELEROMETER_RANGE_2G:
			return 0.00098
		case ACCELEROMETER_RANGE_4G:
			return 0.00195
		case ACCELEROMETER_RANGE_8G:
			return 0.0039
		case ACCELEROMETER_RANGE_16G:
			return 0.01172
		}
	}
	t.Error("Bad range or mode in test")
	return 0.0
}

// Gets the bit shift amount for the current mode
func getShift_(mode AccelerometerMode, t *testing.T) uint8 {
	switch mode {
	case ACCELEROMETER_MODE_HIGH_RESOLUTION:
		return 4
	case ACCELEROMETER_MODE_NORMAL:
		return 6
	case ACCELEROMETER_MODE_LOW_POWER:
		return 8
	default:
		t.Errorf("Bad mode in test")
		return 0
	}
}

func TestNewAccelerometer(t *testing.T) {
	scenario := &i2ctest.Playback{
		Ops: []i2ctest.IO{
			// Write the configuration, 100 Hz
			{Addr: ACCELEROMETER_ADDRESS, W: []byte{ACCELEROMETER_CTRL_REG1_A, 0x57}, R: []byte{}},
			// Read the chipId
			{Addr: ACCELEROMETER_ADDRESS, W: []byte{ACCELEROMETER_IDENTIFY}, R: []byte{0x33}},
			// Read range
			{Addr: ACCELEROMETER_ADDRESS, W: []byte{ACCELEROMETER_CTRL_REG4_A}, R: []byte{0}},
			// Write new range
			{Addr: ACCELEROMETER_ADDRESS, W: []byte{ACCELEROMETER_CTRL_REG4_A, 0x10}, R: []byte{}},
			// Read mode
			{Addr: ACCELEROMETER_ADDRESS, W: []byte{ACCELEROMETER_CTRL_REG1_A}, R: []byte{0}},
			// Write new mode power
			{Addr: ACCELEROMETER_ADDRESS, W: []byte{ACCELEROMETER_CTRL_REG1_A, 0}, R: []byte{}},
			// Read mode
			{Addr: ACCELEROMETER_ADDRESS, W: []byte{ACCELEROMETER_CTRL_REG4_A}, R: []byte{0}},
			// Write new mode resolution
			{Addr: ACCELEROMETER_ADDRESS, W: []byte{ACCELEROMETER_CTRL_REG4_A, 0}, R: []byte{}},
		},
	}
	_, err := NewAccelerometer(scenario, &DefaultAccelerometerOpts)
	if err != nil {
		t.Fatal(err)
	}
}

func TestAccelerometerSense(t *testing.T) {
	scenario := &i2ctest.Playback{
		Ops: []i2ctest.IO{
			// Read registers
			{Addr: ACCELEROMETER_ADDRESS, W: []byte{ACCELEROMETER_OUT_X_L_A}, R: []byte{0}},
			{Addr: ACCELEROMETER_ADDRESS, W: []byte{ACCELEROMETER_OUT_X_H_A}, R: []byte{1}},
			{Addr: ACCELEROMETER_ADDRESS, W: []byte{ACCELEROMETER_OUT_Y_L_A}, R: []byte{100}},
			{Addr: ACCELEROMETER_ADDRESS, W: []byte{ACCELEROMETER_OUT_Y_H_A}, R: []byte{0}},
			{Addr: ACCELEROMETER_ADDRESS, W: []byte{ACCELEROMETER_OUT_Z_L_A}, R: []byte{0xff}},
			{Addr: ACCELEROMETER_ADDRESS, W: []byte{ACCELEROMETER_OUT_Z_H_A}, R: []byte{0xff}},
		},
	}

	accelerometer := &Accelerometer{
		mmr: mmr.Dev8{
			Conn:  &i2c.Dev{Bus: scenario, Addr: uint16(ACCELEROMETER_ADDRESS)},
			Order: binary.BigEndian,
		},
		range_: ACCELEROMETER_RANGE_4G,
		mode:   ACCELEROMETER_MODE_NORMAL,
	}

	x, y, z, err := accelerometer.SenseRaw()
	if err != nil {
		t.Fatal(err)
	}

	if x != 256 {
		t.Fatal("Bad x")
	}
	if y != 100 {
		t.Fatal("Bad y")
	}
	if z != -1 {
		t.Fatal("Bad z")
	}
}

func TestNewMagnetometer(t *testing.T) {
	scenario := &i2ctest.Playback{
		Ops: []i2ctest.IO{
			// Read the chip ID (not a real ID, just a constant)
			{Addr: MAGNETOMETER_ADDRESS, W: []byte{MAGNETOMETER_IRA_REG_M}, R: []byte{0b01001000}},
			// Read gain
			{Addr: MAGNETOMETER_ADDRESS, W: []byte{MAGNETOMETER_CRB_REG_M}, R: []byte{0}},
			// Write new gain
			{Addr: MAGNETOMETER_ADDRESS, W: []byte{MAGNETOMETER_CRB_REG_M, uint8(DefaultMagnetometerOpts.Gain) << 5}, R: []byte{}},
			// Write new rate
			{Addr: MAGNETOMETER_ADDRESS, W: []byte{MAGNETOMETER_CRA_REG_M, (uint8(DefaultMagnetometerOpts.Rate) << 2) | 0b10000000}, R: []byte{}},
		},
	}
	_, err := NewMagnetometer(scenario, &DefaultMagnetometerOpts)
	if err != nil {
		t.Fatal(err)
	}
}

func TestMagnetometerSense(t *testing.T) {
	scenario := &i2ctest.Playback{
		Ops: []i2ctest.IO{
			// Read registers
			{Addr: MAGNETOMETER_ADDRESS, W: []byte{MAGNETOMETER_OUT_X_L_M}, R: []byte{0}},
			{Addr: MAGNETOMETER_ADDRESS, W: []byte{MAGNETOMETER_OUT_X_H_M}, R: []byte{1}},
			{Addr: MAGNETOMETER_ADDRESS, W: []byte{MAGNETOMETER_OUT_Y_L_M}, R: []byte{100}},
			{Addr: MAGNETOMETER_ADDRESS, W: []byte{MAGNETOMETER_OUT_Y_H_M}, R: []byte{0}},
			{Addr: MAGNETOMETER_ADDRESS, W: []byte{MAGNETOMETER_OUT_Z_L_M}, R: []byte{0xff}},
			{Addr: MAGNETOMETER_ADDRESS, W: []byte{MAGNETOMETER_OUT_Z_H_M}, R: []byte{0xff}},
		},
	}

	magnetometer := &Magnetometer{
		mmr: mmr.Dev8{
			Conn:  &i2c.Dev{Bus: scenario, Addr: uint16(MAGNETOMETER_ADDRESS)},
			Order: binary.BigEndian,
		},
		gain: MAGNETOMETER_GAIN_4_0,
		rate: MAGNETOMETER_RATE_30,
	}

	x, y, z, err := magnetometer.SenseRaw()
	if err != nil {
		t.Fatal(err)
	}

	if x != 256 {
		t.Fatal("Bad x")
	}
	if y != 100 {
		t.Fatal("Bad y")
	}
	if z != -1 {
		t.Fatal("Bad z")
	}
}

func TestGetTemperature(t *testing.T) {
	scenario := &i2ctest.Playback{
		Ops: []i2ctest.IO{
			{Addr: MAGNETOMETER_ADDRESS, W: []byte{MAGNETOMETER_TEMP_OUT_H_M}, R: []byte{0}},
			{Addr: MAGNETOMETER_ADDRESS, W: []byte{MAGNETOMETER_TEMP_OUT_L_M}, R: []byte{0}},
			{Addr: MAGNETOMETER_ADDRESS, W: []byte{MAGNETOMETER_TEMP_OUT_H_M}, R: []byte{0}},
			{Addr: MAGNETOMETER_ADDRESS, W: []byte{MAGNETOMETER_TEMP_OUT_L_M}, R: []byte{0b10000000}},
			{Addr: MAGNETOMETER_ADDRESS, W: []byte{MAGNETOMETER_TEMP_OUT_H_M}, R: []byte{0b11111111}},
			{Addr: MAGNETOMETER_ADDRESS, W: []byte{MAGNETOMETER_TEMP_OUT_L_M}, R: []byte{0b10000000}},
		},
	}

	magnetometer := &Magnetometer{
		mmr: mmr.Dev8{
			Conn:  &i2c.Dev{Bus: scenario, Addr: uint16(MAGNETOMETER_ADDRESS)},
			Order: binary.BigEndian,
		},
		gain: MAGNETOMETER_GAIN_4_0,
		rate: MAGNETOMETER_RATE_30,
	}

	temperature, err := magnetometer.GetTemperature()
	if err != nil {
		t.Fatal(err)
	}
	const offset = 20
	if temperature != physic.ZeroCelsius+offset*physic.Celsius {
		t.Fatal("Not 0 C")
	}

	temperature, err = magnetometer.GetTemperature()
	if err != nil {
		t.Fatal(err)
	}
	if temperature != physic.ZeroCelsius+physic.Celsius+offset*physic.Celsius {
		t.Fatal("Not 1 C")
	}

	temperature, err = magnetometer.GetTemperature()
	if err != nil {
		t.Fatal(err)
	}
	if temperature != physic.ZeroCelsius-physic.Celsius+offset*physic.Celsius {
		t.Fatal("Not -1 C")
	}
}
