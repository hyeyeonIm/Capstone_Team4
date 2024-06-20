# This code receives and outputs data from three sensors(DHT11,mh-z19,PMS7003)
# This code do not include the code for PMS7003, you should import PMS7003.py
# You can download PMS7003.py at here
# https://github.com/eleparts/PMS7003/blob/master/PMS7003.py
# DHT11
import time
import board
import adafruit_dht
# mh_z19
import mh_z19
# PMS7003
import serial
from PMS7003 import PMS7003

dhtDevice = adafruit_dht.DHT11(board.D2)

dust = PMS7003()

# Baud Rate
Speed = 9600

# UART / USB Serial
USB0 = '/dev/ttyUSB0'
UART = '/dev/ttyAMA0'

# USE PORT
SERIAL_PORT = USB0


while True:
    try:
        # DHT11 sensor
        # Print the values to the serial port
        # float type(C)
        temperature_c = dhtDevice.temperature
        # float type(F)
        temperature_f = temperature_c * (9 / 5) + 32
        # float type, relative humidity(%)
        humidity = dhtDevice.humidity

        # PMS7003 sensor
        ser = serial.Serial(SERIAL_PORT, Speed, timeout=1)

        buffer = ser.read(1024)

        if (dust.protocol_chk(buffer)):
            data = dust.unpack_data(buffer)
            # int type
            DUST_PM1_0_ATM = data[dust.DUST_PM1_0_ATM]
            DUST_PM2_5_ATM = data[dust.DUST_PM2_5_ATM]
            DUST_PM10_0_ATM = data[dust.DUST_PM10_0_ATM]
        else:
            print("data read Err")

        ser.close()

        # mh_z19 sensor
        # dict type(co2_dict) [co2, value]
        co2_dict = mh_z19.read()
        # int type(co2,ppm)
        co2 = co2_dict.get('co2')

        print("final_result")
        print(temperature_c)
        print(humidity)
        print(DUST_PM1_0_ATM)
        print(DUST_PM2_5_ATM)
        print(DUST_PM10_0_ATM)
        print(co2)

    except RuntimeError as error:
        # Errors happen fairly often, DHT's are hard to read, just keep going
        print(error.args[0])
        time.sleep(2.0)
        continue
    except Exception as error:
        dhtDevice.exit()
        raise error

    time.sleep(5.0)

