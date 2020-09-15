package lsm303

import (
	"testing"
	"periph.io/x/periph/conn/physic"
)

func TestGetLsb(t *testing.T) {
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
			expectedValue := int64(getLsb_(mode, range_, t) * float64(physic.EarthGravity))
			computedValue := int64(getLsb(mode, range_))
			if computedValue != expectedValue {
				t.Errorf("getLsb(%s, %s) should be %v but was %v", mode, range_, expectedValue, computedValue)
			}
		}
	}
}

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
	return -1.0
}
