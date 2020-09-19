package lsm303

import (
	"encoding/binary"
	"errors"
	"log"
	"periph.io/x/periph/conn/i2c"
	"periph.io/x/periph/conn/mmr"
	"periph.io/x/periph/conn/physic"
	"time"
)

// Opts holds the configuration options.
type Opts struct {
	Range AccelerometerRange
	Mode  AccelerometerMode
}

// DefaultOpts is the recommended default options.
var DefaultOpts = Opts{
	Range: ACCELEROMETER_RANGE_4G,
	Mode:  ACCELEROMETER_MODE_NORMAL,
}

// New accelerometer opens a handle to an nSM303 accelerometer sensor.
func NewAccelerometer(bus i2c.Bus, opts *Opts) (*Accelerometer, error) {
	const ACCELEROMETER_ADDRESS = 0x19
	device := &Accelerometer{
		mmr: mmr.Dev8{
			Conn: &i2c.Dev{Bus: bus, Addr: uint16(ACCELEROMETER_ADDRESS)},
			// I don't think we ever access more than 1 byte at once, so
			// this is irrelevant
			Order: binary.BigEndian,
		},
		range_: opts.Range,
		mode:   opts.Mode,
	}

	// Enable the accelerometer 100 Hz, 0x57 = 0b01010111
	// Bits 0-2 = X, Y, Z enable
	// Bit 3 = low power mode
	// Bits 4-7 = speed, 0 = power down, 1-7 = 1 10 25 50 100 200 400 Hz, 8 = low
	//   power mode 1.62 khZ, 9 = normal 1.34 kHz / low power 5.376 kHz
	// TODO: Allow the user to set the Hz and toggle axes
	err := device.mmr.WriteUint8(ACCELEROMETER_CTRL_REG1_A, 0x57)
	if err != nil {
		return nil, err
	}

	chipId, err := device.mmr.ReadUint8(ACCELEROMETER_IDENTIFY)
	if err != nil {
		return nil, err
	}
	if chipId != 0x33 {
		return nil, errors.New("No LSM303 detected")
	}

	device.SetRange(opts.Range)
	device.SetMode(opts.Mode)

	return device, nil
}

// Dev is a handle to the LSM303 accelerometer sensor.
type Accelerometer struct {
	mmr    mmr.Dev8
	range_ AccelerometerRange
	mode   AccelerometerMode
}

func (accelerometer *Accelerometer) SenseRaw() (int16, int16, int16) {
	xLow, err := accelerometer.mmr.ReadUint8(ACCELEROMETER_OUT_X_L_A)
	if err != nil {
		log.Fatal(err)
	}
	xHigh, err := accelerometer.mmr.ReadUint8(ACCELEROMETER_OUT_X_H_A)
	if err != nil {
		log.Fatal(err)
	}
	yLow, err := accelerometer.mmr.ReadUint8(ACCELEROMETER_OUT_Y_L_A)
	if err != nil {
		log.Fatal(err)
	}
	yHigh, err := accelerometer.mmr.ReadUint8(ACCELEROMETER_OUT_Y_H_A)
	if err != nil {
		log.Fatal(err)
	}
	zLow, err := accelerometer.mmr.ReadUint8(ACCELEROMETER_OUT_Z_L_A)
	if err != nil {
		log.Fatal(err)
	}
	zHigh, err := accelerometer.mmr.ReadUint8(ACCELEROMETER_OUT_Z_H_A)
	if err != nil {
		log.Fatal(err)
	}

	xValue := (int16)((((uint16)(xHigh)) << 8) + (uint16)(xLow))
	yValue := (int16)((((uint16)(yHigh)) << 8) + (uint16)(yLow))
	zValue := (int16)((((uint16)(zHigh)) << 8) + (uint16)(zLow))

	return xValue, yValue, zValue
}

func (accelerometer *Accelerometer) Sense() (physic.Force, physic.Force, physic.Force) {
	xValue, yValue, zValue := accelerometer.SenseRaw()
	multiplier := getMultiplier(accelerometer.mode, accelerometer.range_)
	xAcceleration := (physic.Force)(int64(xValue) * multiplier)
	yAcceleration := (physic.Force)(int64(yValue) * multiplier)
	zAcceleration := (physic.Force)(int64(zValue) * multiplier)

	return xAcceleration, yAcceleration, zAcceleration
}

func (accelerometer *Accelerometer) GetMode() (AccelerometerMode, error) {
	lowPowerU8, err := accelerometer.mmr.ReadUint8(ACCELEROMETER_CTRL_REG1_A)
	if err != nil {
		return ACCELEROMETER_MODE_NORMAL, err
	}
	lowPowerBit := readBits((uint32)(lowPowerU8), 1, 3)

	highResolutionU8, err := accelerometer.mmr.ReadUint8(ACCELEROMETER_CTRL_REG4_A)
	if err != nil {
		return ACCELEROMETER_MODE_NORMAL, err
	}
	highResolutionBit := readBits((uint32)(highResolutionU8), 1, 3)

	return (AccelerometerMode)((lowPowerBit << 1) | highResolutionBit), nil
}

func (accelerometer *Accelerometer) SetMode(mode AccelerometerMode) error {
	const bits = 1
	const shift = 3

	data := (uint8)((mode & 0x02) >> 1)
	power, err := accelerometer.mmr.ReadUint8(ACCELEROMETER_CTRL_REG1_A)
	if err != nil {
		return err
	}

	mask := (uint8)((1 << bits) - 1)
	data &= mask
	mask <<= shift
	power &= (^mask)
	power |= data << shift
	err = accelerometer.mmr.WriteUint8(ACCELEROMETER_CTRL_REG1_A, power)
	if err != nil {
		return err
	}
	time.Sleep(time.Millisecond * 20)

	data = (uint8)(mode & 0x01)
	resolution, err := accelerometer.mmr.ReadUint8(ACCELEROMETER_CTRL_REG4_A)
	if err != nil {
		return err
	}
	mask = (uint8)((1 << bits) - 1)
	data &= mask
	mask <<= shift
	resolution &= (^mask)
	resolution |= data << shift
	err = accelerometer.mmr.WriteUint8(ACCELEROMETER_CTRL_REG4_A, resolution)
	if err != nil {
		return err
	}
	time.Sleep(time.Millisecond * 20)

	accelerometer.mode = mode

	return nil
}

func (accelerometer *Accelerometer) GetRange() (AccelerometerRange, error) {
	value, err := accelerometer.mmr.ReadUint8(ACCELEROMETER_CTRL_REG4_A)
	if err != nil {
		return ACCELEROMETER_RANGE_4G, err
	}
	range_ := (((uint32)(value)) >> 4) & ((1 << 2) - 1)
	return (AccelerometerRange)(range_), nil
}

func (accelerometer *Accelerometer) SetRange(range_ AccelerometerRange) error {
	const bits = 2
	const shift = 4

	data := (uint8)(range_)
	currentRange, err := accelerometer.mmr.ReadUint8(ACCELEROMETER_CTRL_REG4_A)
	if err != nil {
		return err
	}

	mask := (uint8)((1 << bits) - 1)
	data &= mask
	mask <<= shift
	currentRange &= (^mask)
	currentRange |= data << shift
	err = accelerometer.mmr.WriteUint8(ACCELEROMETER_CTRL_REG4_A, currentRange)
	if err != nil {
		return err
	}
	time.Sleep(time.Millisecond * 20)

	accelerometer.range_ = range_

	return nil
}

type AccelerometerMode int

const (
	ACCELEROMETER_MODE_NORMAL AccelerometerMode = iota
	ACCELEROMETER_MODE_HIGH_RESOLUTION
	ACCELEROMETER_MODE_LOW_POWER
)

func (mode AccelerometerMode) String() string {
	return [...]string{"normal", "high resolution", "low power"}[mode]
}

type AccelerometerRange int

const (
	ACCELEROMETER_RANGE_2G AccelerometerRange = iota
	ACCELEROMETER_RANGE_4G
	ACCELEROMETER_RANGE_8G
	ACCELEROMETER_RANGE_16G
)

func (range_ AccelerometerRange) String() string {
	return [...]string{"2G", "4G", "8G", "16G"}[range_]
}

func readBits(value uint32, bits uint32, shift uint8) uint32 {
	value >>= shift
	return value & ((1 << bits) - 1)
}

// Gets the multiplier for the accelerometer mode and range
func getMultiplier(mode AccelerometerMode, range_ AccelerometerRange) int64 {
	// The constants in here needed to be rounded because some of then aren't
	// exactly representable. I added tests for what the true value should be.
	switch mode {
	case ACCELEROMETER_MODE_LOW_POWER:
		switch range_ {
		case ACCELEROMETER_RANGE_2G:
			return 153277939 >> 8
		case ACCELEROMETER_RANGE_4G:
			return 306555879 >> 8
		case ACCELEROMETER_RANGE_8G:
			return 613111758 >> 8
		case ACCELEROMETER_RANGE_16G:
			return 1839531407 >> 8
		}
	case ACCELEROMETER_MODE_NORMAL:
		switch range_ {
		case ACCELEROMETER_RANGE_2G:
			return 38245935 >> 6
		case ACCELEROMETER_RANGE_4G:
			return 76688003 >> 6
		case ACCELEROMETER_RANGE_8G:
			return 153277939 >> 6
		case ACCELEROMETER_RANGE_16G:
			return 459931885 >> 6
		}

	case ACCELEROMETER_MODE_HIGH_RESOLUTION:
		switch range_ {
		case ACCELEROMETER_RANGE_2G:
			return 9610517 >> 4
		case ACCELEROMETER_RANGE_4G:
			return 19122967 >> 4
		case ACCELEROMETER_RANGE_8G:
			return 38245935 >> 4
		case ACCELEROMETER_RANGE_16G:
			return 114933938 >> 4
		}
	default:
		log.Fatalf("Unknown mode %v in getMultiplier", mode)
	}
	log.Fatalf("Unknown range %v in getMultiplier", range_)
	return 0.0
}

const (
	// Copied from the data sheet. Unused values are commented out.
	ACCELEROMETER_IDENTIFY    = 0x0F
	ACCELEROMETER_CTRL_REG1_A = 0x20
	//ACCELEROMETER_CTRL_REG2_A     = 0x21
	//ACCELEROMETER_CTRL_REG3_A     = 0x22
	ACCELEROMETER_CTRL_REG4_A = 0x23
	//ACCELEROMETER_CTRL_REG5_A     = 0x24
	//ACCELEROMETER_CTRL_REG6_A     = 0x25
	//ACCELEROMETER_REFERENCE_A     = 0x26
	//ACCELEROMETER_STATUS_REG_A    = 0x27
	ACCELEROMETER_OUT_X_L_A = 0x28
	ACCELEROMETER_OUT_X_H_A = 0x29
	ACCELEROMETER_OUT_Y_L_A = 0x2A
	ACCELEROMETER_OUT_Y_H_A = 0x2B
	ACCELEROMETER_OUT_Z_L_A = 0x2C
	ACCELEROMETER_OUT_Z_H_A = 0x2D
	//ACCELEROMETER_FIFO_CTRL_REG_A = 0x2E
	//ACCELEROMETER_FIFO_SRC_REG_A  = 0x2F
	//ACCELEROMETER_INT1_CFG_A      = 0x30
	//ACCELEROMETER_INT1_SOURCE_A   = 0x31
	//ACCELEROMETER_INT1_THS_A      = 0x32
	//ACCELEROMETER_INT1_DURATION_A = 0x33
	//ACCELEROMETER_INT2_CFG_A      = 0x34
	//ACCELEROMETER_INT2_SOURCE_A   = 0x35
	//ACCELEROMETER_INT2_THS_A      = 0x36
	//ACCELEROMETER_INT2_DURATION_A = 0x37
	//ACCELEROMETER_CLICK_CFG_A     = 0x38
	//ACCELEROMETER_CLICK_SRC_A     = 0x39
	//ACCELEROMETER_CLICK_THS_A     = 0x3A
	//ACCELEROMETER_TIME_LATENCY_A  = 0x3B
	//ACCELEROMETER_TIME_LIMIT_A    = 0x3C
	//ACCELEROMETER_TIME_WINDOW_A   = 0x3D
)
