# This code stores json data received from Raspberry Pi ROS into mysql.
from __future__ import print_function
import roslibpy
import pymysql
import json

# MySQL connect
db_connection = pymysql.connect(
    host='localhost',
    user='sensor',
    password='987654321',
    database='sensor_data',
    cursorclass=pymysql.cursors.DictCursor
)

client = roslibpy.Ros(host='localhost', port=9090)
client.run()


def callback(message):
    json_str = message['data']
    print('Heard talking: ' + json_str)

    # JSON data to python dict
    json_data = json.loads(json_str)

    # insert to db
    try:
        with db_connection.cursor() as cursor:
            query = (
                "INSERT INTO sensors(room, temperature, humidity, DUST_PM1_0_ATM, DUST_PM2_5_ATM, DUST_PM10_0_ATM, co2) "
                "VALUES(%s, %s, %s, %s, %s, %s, %s)")
            cursor.execute(query, (
                'room1',  # Here is hardcoded right now need to fix
                json_data['temperature'],
                json_data['humidity'],
                json_data['DUST_PM1_0_ATM'],
                json_data['DUST_PM2_5_ATM'],
                json_data['DUST_PM10_0_ATM'],
                json_data['co2']
            ))
        db_connection.commit()
    except Exception as e:
        print(f"Error: {e}")


listener = roslibpy.Topic(client, '/matt', 'std_msgs/String')
listener.subscribe(callback)

try:
    while True:
        pass
except KeyboardInterrupt:
    client.terminate()
    db_connection.close()
