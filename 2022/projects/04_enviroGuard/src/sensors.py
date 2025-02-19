import board
import digitalio
import smbus

from adafruit_bme280 import basic as adafruit_bme280
from time import sleep
i2c = board.I2C()  # uses board.SCL and board.SDA
bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)
veml_bus = smbus.SMBus(1)

veml_addr = 0x10

#Write registers
als_conf_0 = 0x00
als_WH = 0x01
als_WL = 0x02
pow_sav = 0x03

#Read registers
als = 0x04
white = 0x05
interrupt = 0x06


# These settings will provide the max range for the sensor (0-120Klx)
# but at the lowest precision:
#              LSB   MSB
# see www.vishay.com/docs/84323/designingveml7700.pdf
confValues = [0x00, 0x13] # 1/8 gain, 25ms IT (Integration Time)
#Reference data sheet Table 1 for configuration settings

interrupt_high = [0x00, 0x00] # Clear values
#Reference data sheet Table 2 for High Threshold

interrupt_low = [0x00, 0x00] # Clear values
#Reference data sheet Table 3 for Low Threshold

power_save_mode = [0x00, 0x00] # Clear values
#Reference data sheet Table 4 for Power Saving Modes

veml_bus.write_i2c_block_data(veml_addr, als_conf_0, confValues)
veml_bus.write_i2c_block_data(veml_addr, als_WH, interrupt_high)
veml_bus.write_i2c_block_data(veml_addr, als_WL, interrupt_low)
veml_bus.write_i2c_block_data(veml_addr, pow_sav, power_save_mode)

def getLux():
	word = veml_bus.read_word_data(veml_addr, als)
	gain = 1.8432 # gain for chosen confValues, see www.vishay.com/docs/84323/designingveml7700.pdf
	luxVal = round(word * gain, 1)
	return luxVal

def getTemperature():
    return bme280.temperature
def getHumidity():
    return bme280.humidity
def getPressure():
    return bme280.pressure
def getData():
    # while True:
        
    #     print("\n")
    #     print("Temperature: "+str(getTemperature()))
    #     print("Humidity: "+str(getHumidity()))
    #     print("Pressure: "+str(getPressure()))
    #     print("Lux: "+str(getLux()))
    #     sleep(10)

    temperature = getTemperature()
    humidity = getHumidity()
    pressure = getPressure()
    lux=getLux()
    #tvoc = getTvoc()
    tvoc = 0
    #co2 = getCO2
    co2 = 0

    return temperature, humidity, pressure, lux, tvoc, co2

