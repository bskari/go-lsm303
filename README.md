# go-lsm303
Adafruit LSM303 accelerometer and magnetometer via I2C-bus from Raspberry PI

## Usage

### Acceleroometer

    accelerometer, err := NewAccelerometer()
    xa, ya, za, err := accelerometer.Sense()
    // Configuration
    accelerometer.SetRange(ACCELEROMETER_RANGE_16G)
    accelerometer.SetMode(ACCELEROMETER_MODE_LOW_POWER)

### Magnetometer 

    // The periph.io has units defined for many things, but not for
    // magnetometer flux, so we only have SenseRaw
    xm, ym, zm, err := magnetometer.SenseRaw()

    // Configuration
    magnetometer.SetGain(MAGNETOMETER_GAIN_5_6)
    magnetometer.SetRate(MAGNETOMETER_RATE_75)

    // The magnetometer also has a relative temperature sensor. It's not
    // calibrated, but adding 20 should give the approximate temperature.
    temp, err := magnetometer.SenseRelativeTemperature()

### Computing heading

With these sensors, you can compute the tilt-compensated heading. Note that the
magnetometer will have some hard interference from locally mounted metal
objects. To compensate, you will need to log the maximum and minimum
magnetometer readings while moving the object through all orientations.

    type Axes struct {
        Pitch Radians
        Roll  Radians
        Yaw   Radians
    }

    xa, ya, za, err := accelerometer.Sense()
    xm, ym, zm, err := magnetometer.SenseRaw()
    axes := computeAxes(xa, ya, za, xm, ym, zm)

    func computeAxes(xRawA, yRawA, zRawA, xRawM, yRawM, zRawM int16) Axes {
        // Avoid divide by zero problems
        if zRawA == 0 {
            zRawA = 1
        }
        // The roll calculation assumes that +y is forward, x is right, and
        // +z is up
        x2 := int32(xRawA) * int32(xRawA)
        z2 := int32(zRawA) * int32(zRawA)

        // Tilt compensated compass readings
        pitch_r := math.Atan2(float64(yRawA), math.Sqrt(float64(x2+z2)))
        roll_r := -math.Atan2(float64(xRawA), float64(zRawA))

        pitch_r -= configuration.PitchOffset
        for pitch_r < ToRadians(-180.0) {
            pitch_r += ToRadians(360.0)
        }
        for pitch_r > ToRadians(180.0) {
            pitch_r -= ToRadians(360.0)
        }

        roll_r -= configuration.RollOffset
        for roll_r < ToRadians(-180.0) {
            roll_r += ToRadians(360.0)
        }
        for roll_r > ToRadians(180.0) {
            roll_r -= ToRadians(360.0)
        }

        temp_m.x -= ((int32_t)m_min.x + m_max.x) / 2;
        temp_m.y -= ((int32_t)m_min.y + m_max.y) / 2;

        // Here you will need to set the maximum and minimum magnetometer
        // readings from calibration as described above
        const int16 maxXM = 0
        const int16 maxYM = 0
        const int16 maxZM = 0
        const int16 minXM = 0
        const int16 minYM = 0
        const int16 minZM = 0

        xM := float64(xRawM - (maxXM - minXM) / 2)
        yM := float64(yRawM - (maxYM - minYM) / 2)
        zM := float64(zRawM - (maxZM - minZM) / 2)
        xHorizontal := xM*math.Cos(-pitch_r) + yM*math.Sin(roll_r)*math.Sin(-pitch_r) - zM*math.Cos(roll_r)*math.Sin(-pitch_r)
        yHorizontal := yM*math.Cos(roll_r) + zM*math.Sin(roll_r)
        yaw_r := math.Atan2(yHorizontal, xHorizontal)
        return Axes{
            Pitch: pitch_r,
            Roll:  roll_r,
            Yaw:   yaw_r,
        }
    }

## Caveats

This uses periph.io to access peripherals. periph.io made some major updates in
2020, (see [Periph's blog post](https://periph.io/news/2020/a_new_start/)) but
I haven't had a chance to test the new version yet. Pull requests welcome!

For best results, you will need to calibrate the magnetometer as described above.
