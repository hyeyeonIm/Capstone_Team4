# This code sends json data to Node.js from Ubuntu ROS.
from __future__ import print_function
import roslibpy
import requests
import json

client = roslibpy.Ros(host='localhost', port=9090)
client.run()

# send json data to Node,js
def callback(message):
    json_str = message['data']
    print('Heard talking: ' + json_str)

    try:
        url = 'http://localhost:8080/data'
        headers = {'Content-Type': 'application/json'}
        response = requests.post(url, data=json_str, headers=headers)
        print('Server response:', response.status_code, response.text)
    except Exception as e:
        print(f"Error: {e}")


listener = roslibpy.Topic(client, '/matt', 'std_msgs/String')
listener.subscribe(callback)

try:
    while True:
        pass
except KeyboardInterrupt:
    client.terminate()
