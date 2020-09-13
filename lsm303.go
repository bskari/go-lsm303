package lsm303

import (
	"errors"
	"github.com/d2r2/go-i2c"
	"log"
	"time"
)

type Lsm303Accelerometer struct {
	i2c    *i2c.I2C
	range_ Lsm303AccelerometerRange
	mode   Lsm303AccelerometerMode
}

func NewLsm303Accelerometer() (*Lsm303Accelerometer, error) {
	const LSM303_ACCELEROMETER_ADDRESS = 0x19
	i2c, err := i2c.NewI2C(LSM303_ACCELEROMETER_ADDRESS, 1)
	if err != nil {
		return nil, err
	}
	defer i2c.Close()
	// Enable the accelerometer 100 Hz, 0x57 = 0b01010111
	// Bits 0-2 = X, Y, Z enable
	// Bit 3 = low power mode
	// Bits 4-7 = speed, 0 = power down, 1-7 = 1 10 25 50 100 200 400 Hz, 8 = low
	//   power mode 1.62 khZ, 9 = normal 1.34 kHz / low power 5.376 kHz
	// TODO: Allow the user to set the Hz
	err = i2c.WriteRegU8(LSM303_ACCELEROMETER_CTRL_REG1_A, 0x57)
	if err != nil {
		return nil, err
	}

	// Check the chip ID
	chipId, err := i2c.ReadRegU8(LSM303_ACCELEROMETER_IDENTIFY)
	if err != nil {
		return nil, err
	}
	if chipId != 0x33 {
		return nil, errors.New("No LSM303 detected")
	}

	accelerometer := Lsm303Accelerometer{
		i2c:    i2c,
		range_: LSM303_ACCELEROMETER_RANGE_4G,
		mode:   LSM303_ACCELEROMETER_MODE_NORMAL,
	}
	accelerometer.SetRange(accelerometer.range_)
	accelerometer.SetMode(accelerometer.mode)
	return &accelerometer, nil
}

func (accelerometer *Lsm303Accelerometer) Read() (float32, float32, float32) {
	xLow, err := accelerometer.i2c.ReadRegU8(LSM303_ACCELEROMETER_OUT_X_L_A)
	if err != nil {
		log.Fatal(err)
	}
	xHigh, err := accelerometer.i2c.ReadRegU8(LSM303_ACCELEROMETER_OUT_X_H_A)
	if err != nil {
		log.Fatal(err)
	}
	yLow, err := accelerometer.i2c.ReadRegU8(LSM303_ACCELEROMETER_OUT_Y_L_A)
	if err != nil {
		log.Fatal(err)
	}
	yHigh, err := accelerometer.i2c.ReadRegU8(LSM303_ACCELEROMETER_OUT_Y_H_A)
	if err != nil {
		log.Fatal(err)
	}
	zLow, err := accelerometer.i2c.ReadRegU8(LSM303_ACCELEROMETER_OUT_Z_L_A)
	if err != nil {
		log.Fatal(err)
	}
	zHigh, err := accelerometer.i2c.ReadRegU8(LSM303_ACCELEROMETER_OUT_Z_H_A)
	if err != nil {
		log.Fatal(err)
	}

	xValue := (int16)((((uint16)(xHigh)) << 8) + (uint16)(xLow))
	yValue := (int16)((((uint16)(yHigh)) << 8) + (uint16)(yLow))
	zValue := (int16)((((uint16)(zHigh)) << 8) + (uint16)(zLow))

	shift := getShift(accelerometer.mode)
	lsb := getLsb(accelerometer.mode, accelerometer.range_)
	xAcceleration := (float32)(xValue>>shift) * lsb * EARTH_GRAVITY_MPS
	yAcceleration := (float32)(yValue>>shift) * lsb * EARTH_GRAVITY_MPS
	zAcceleration := (float32)(zValue>>shift) * lsb * EARTH_GRAVITY_MPS

	return xAcceleration, yAcceleration, zAcceleration
}

func (accelerometer *Lsm303Accelerometer) GetMode() (Lsm303AccelerometerMode, error) {
	lowPowerU8, err := accelerometer.i2c.ReadRegU8(LSM303_ACCELEROMETER_CTRL_REG1_A)
	if err != nil {
		return LSM303_ACCELEROMETER_MODE_NORMAL, err
	}
	lowPowerBit := readBits((uint32)(lowPowerU8), 1, 3)

	highResolutionU8, err := accelerometer.i2c.ReadRegU8(LSM303_ACCELEROMETER_CTRL_REG4_A)
	if err != nil {
		return LSM303_ACCELEROMETER_MODE_NORMAL, err
	}
	highResolutionBit := readBits((uint32)(highResolutionU8), 1, 3)

	return (Lsm303AccelerometerMode)((lowPowerBit << 1) | highResolutionBit), nil
}

func (accelerometer *Lsm303Accelerometer) SetMode(mode Lsm303AccelerometerMode) error {
	const bits = 1
	const shift = 3

	data := (uint8)((mode & 0x02) >> 1)
	power, err := accelerometer.i2c.ReadRegU8(LSM303_ACCELEROMETER_CTRL_REG1_A)
	if err != nil {
		return err
	}

	mask := (uint8)((1 << bits) - 1)
	data &= mask
	mask <<= shift
	power &= (^mask)
	power |= data << shift
	err = accelerometer.i2c.WriteRegU8(LSM303_ACCELEROMETER_CTRL_REG1_A, power)
	if err != nil {
		return err
	}
	time.Sleep(time.Millisecond * 20)

	data = (uint8)(mode & 0x01)
	resolution, err := accelerometer.i2c.ReadRegU8(LSM303_ACCELEROMETER_CTRL_REG4_A)
	if err != nil {
		return err
	}
	mask = (uint8)((1 << bits) - 1)
	data &= mask
	mask <<= shift
	resolution &= (^mask)
	resolution |= data << shift
	err = accelerometer.i2c.WriteRegU8(LSM303_ACCELEROMETER_CTRL_REG4_A, resolution)
	if err != nil {
		return err
	}
	time.Sleep(time.Millisecond * 20)

	accelerometer.mode = mode

	return nil
}

func (accelerometer *Lsm303Accelerometer) GetRange() (Lsm303AccelerometerRange, error) {
	value, err := accelerometer.i2c.ReadRegU8(LSM303_ACCELEROMETER_CTRL_REG4_A)
	if err != nil {
		return LSM303_ACCELEROMETER_RANGE_4G, err
	}
	range_ := (((uint32)(value)) >> 4) & ((1 << 2) - 1)
	return (Lsm303AccelerometerRange)(range_), nil
}

func (accelerometer *Lsm303Accelerometer) SetRange(range_ Lsm303AccelerometerRange) error {
	const bits = 2
	const shift = 4

	data := (uint8)(range_)
	currentRange, err := accelerometer.i2c.ReadRegU8(LSM303_ACCELEROMETER_CTRL_REG4_A)
	if err != nil {
		return err
	}

	mask := (uint8)((1 << bits) - 1)
	data &= mask
	mask <<= shift
	currentRange &= (^mask)
	currentRange |= data << shift
	err = accelerometer.i2c.WriteRegU8(LSM303_ACCELEROMETER_CTRL_REG4_A, currentRange)
	if err != nil {
		return err
	}
	time.Sleep(time.Millisecond * 20)

	accelerometer.range_ = range_

	return nil
}

type Lsm303AccelerometerMode int

const (
	LSM303_ACCELEROMETER_MODE_NORMAL Lsm303AccelerometerMode = iota
	LSM303_ACCELEROMETER_MODE_HIGH_RESOLUTION
	LSM303_ACCELEROMETER_MODE_LOW_POWER
)

func (mode Lsm303AccelerometerMode) String() string {
	return [...]string{"normal", "high resolution", "low power"}[mode]
}

type Lsm303AccelerometerRange int

const (
	LSM303_ACCELEROMETER_RANGE_2G Lsm303AccelerometerRange = iota
	LSM303_ACCELEROMETER_RANGE_4G
	LSM303_ACCELEROMETER_RANGE_8G
	LSM303_ACCELEROMETER_RANGE_16G
)

func (range_ Lsm303AccelerometerRange) String() string {
	return [...]string{"2G", "4G", "8G", "16G"}[range_]
}

func readBits(value uint32, bits uint32, shift uint8) uint32 {
	value >>= shift
	return value & ((1 << bits) - 1)
}

// Gets the bit shift amount for the current mode
func getShift(mode Lsm303AccelerometerMode) uint8 {
	switch mode {
	case LSM303_ACCELEROMETER_MODE_HIGH_RESOLUTION:
		return 4
	case LSM303_ACCELEROMETER_MODE_NORMAL:
		return 6
	case LSM303_ACCELEROMETER_MODE_LOW_POWER:
		return 8
	default:
		log.Fatal("Unknown mode %v in getShift", mode)
		return 0
	}
}

// Gets the Least Significant Bit value for the current mode
func getLsb(mode Lsm303AccelerometerMode, range_ Lsm303AccelerometerRange) float32 {
	switch mode {
	case LSM303_ACCELEROMETER_MODE_LOW_POWER:
		switch range_ {
		case LSM303_ACCELEROMETER_RANGE_2G:
			return 0.01563
		case LSM303_ACCELEROMETER_RANGE_4G:
			return 0.03126
		case LSM303_ACCELEROMETER_RANGE_8G:
			return 0.06252
		case LSM303_ACCELEROMETER_RANGE_16G:
			return 0.18758
		}
	case LSM303_ACCELEROMETER_MODE_NORMAL:
		switch range_ {
		case LSM303_ACCELEROMETER_RANGE_2G:
			return 0.0039
		case LSM303_ACCELEROMETER_RANGE_4G:
			return 0.00782
		case LSM303_ACCELEROMETER_RANGE_8G:
			return 0.01563
		case LSM303_ACCELEROMETER_RANGE_16G:
			return 0.0469
		}

	case LSM303_ACCELEROMETER_MODE_HIGH_RESOLUTION:
		switch range_ {
		case LSM303_ACCELEROMETER_RANGE_2G:
			return 0.00098
		case LSM303_ACCELEROMETER_RANGE_4G:
			return 0.00195
		case LSM303_ACCELEROMETER_RANGE_8G:
			return 0.0039
		case LSM303_ACCELEROMETER_RANGE_16G:
			return 0.01172
		}
	default:
		log.Fatal("Unknown mode %v in getLsb", mode)
	}
	log.Fatal("Unknown range %v in getLsb", range_)
	return 0.0
}

const (
	// Copied from the data sheet. Unused values are commented out.
	LSM303_ACCELEROMETER_IDENTIFY    = 0x0F
	LSM303_ACCELEROMETER_CTRL_REG1_A = 0x20
	//LSM303_ACCELEROMETER_CTRL_REG2_A     = 0x21
	//LSM303_ACCELEROMETER_CTRL_REG3_A     = 0x22
	LSM303_ACCELEROMETER_CTRL_REG4_A = 0x23
	//LSM303_ACCELEROMETER_CTRL_REG5_A     = 0x24
	//LSM303_ACCELEROMETER_CTRL_REG6_A     = 0x25
	//LSM303_ACCELEROMETER_REFERENCE_A     = 0x26
	//LSM303_ACCELEROMETER_STATUS_REG_A    = 0x27
	LSM303_ACCELEROMETER_OUT_X_L_A = 0x28
	LSM303_ACCELEROMETER_OUT_X_H_A = 0x29
	LSM303_ACCELEROMETER_OUT_Y_L_A = 0x2A
	LSM303_ACCELEROMETER_OUT_Y_H_A = 0x2B
	LSM303_ACCELEROMETER_OUT_Z_L_A = 0x2C
	LSM303_ACCELEROMETER_OUT_Z_H_A = 0x2D
	//LSM303_ACCELEROMETER_FIFO_CTRL_REG_A = 0x2E
	//LSM303_ACCELEROMETER_FIFO_SRC_REG_A  = 0x2F
	//LSM303_ACCELEROMETER_INT1_CFG_A      = 0x30
	//LSM303_ACCELEROMETER_INT1_SOURCE_A   = 0x31
	//LSM303_ACCELEROMETER_INT1_THS_A      = 0x32
	//LSM303_ACCELEROMETER_INT1_DURATION_A = 0x33
	//LSM303_ACCELEROMETER_INT2_CFG_A      = 0x34
	//LSM303_ACCELEROMETER_INT2_SOURCE_A   = 0x35
	//LSM303_ACCELEROMETER_INT2_THS_A      = 0x36
	//LSM303_ACCELEROMETER_INT2_DURATION_A = 0x37
	//LSM303_ACCELEROMETER_CLICK_CFG_A     = 0x38
	//LSM303_ACCELEROMETER_CLICK_SRC_A     = 0x39
	//LSM303_ACCELEROMETER_CLICK_THS_A     = 0x3A
	//LSM303_ACCELEROMETER_TIME_LATENCY_A  = 0x3B
	//LSM303_ACCELEROMETER_TIME_LIMIT_A    = 0x3C
	//LSM303_ACCELEROMETER_TIME_WINDOW_A   = 0x3D
)

const EARTH_GRAVITY_MPS = 9.80665
