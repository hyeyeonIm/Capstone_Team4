# This python file is for insert DHT11 sensor data into mysql
import sys
import time
import adafruit_dht
import pymysql
import board

# sensor
sensor = adafruit_dht.DHT11(board.D2)

# conn = pymysql.connect( connect option )
conn=pymysql.connect(host="localhost",
                     user="sensor_user",
                     passwd="987654321",
                     db="sensor_data")

# count fot test end point
# count = 0

try: 
    with conn.cursor() as cur :
        sql="insert into collect_data(sensor, collect_time, temp, humidity) values(%s, %s, %s, %s)"
        while True:
            # humidity, temperature = dht.read_retry(sensor, pin)
            temperature = sensor.temperature
            humidity = sensor.humidity
            if humidity is not None and temperature is not None:
                print('Temp=%0.1f*c Humidity=%0.1f'%(temperature, humidity))
                cur.execute(sql,('DHT11',
                                 time.strftime("%Y-%m-%d %H:%M:%S",time.localtime()),
                                 temperature,
                                 humidity))
                conn.commit()
                
                # count += 1
            else:
                print("Failed to get reading.")
                
            # end point setting
            # if count >= 100:
                #break;
            
            time.sleep(5)
except RuntimeError as e:
    print("Reading from DHT sensor failed:", e)
except KeyboardInterrupt:
    # end by Ctrl + C
    print("Terminated by Keyboard")
finally:
        conn.close()
