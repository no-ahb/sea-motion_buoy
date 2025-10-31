from machine import Pin, I2C
import time

i2c = I2C(0, scl=Pin(1), sda=Pin(0))
print("I2C scan:", i2c.scan())

# Expected output: [0x4A] or [0x4B]
# If nothing appears, check wiring and power.