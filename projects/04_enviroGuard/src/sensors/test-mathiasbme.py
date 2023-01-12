import board
import digitalio
from adafruit_bme280 import basic as adafruit_bme280
from time import sleep
i2c = board.I2C()  # uses board.SCL and board.SDA
while True:
    bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)

    print("\nTemperature: %0.1f C" % bme280.temperature)
    print("Humidity: %0.1f %%" % bme280.humidity)
    print("Pressure: %0.1f hPa" % bme280.pressure)
    sleep(1)