package lsm303

import (
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
