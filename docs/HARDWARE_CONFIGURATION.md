# Hardware configuration checklist

Hardware-dependent settings intentionally remain unconfirmed. Do not enable the
mechanism or connect its load until this checklist has been reviewed on the actual unit.

- ESP32 module and board revision
- I2C SDA/SCL pins, pull-ups, bus speed, and exact barometer/IMU variants
- GPS model, supply voltage, UART pins, baud rate, and enabled NMEA sentences
- Radio model, supply, UART pins, baud rate, RF frequency, power, and framing mode
- Battery chemistry, divider resistor values, ADC channel, and safe input voltage
- Current-sensor model and transfer function, if fitted
- Mechanism type, driver circuit, GPIO, active level, pulse duration, and independent inhibit
- Confirmed team ID and authoritative competition protocol
- Pin-conflict and ESP32 boot-strapping review

Record measurements, wiring diagrams, calibration values, and reviewer/date here before
changing any safety-related Kconfig default.
