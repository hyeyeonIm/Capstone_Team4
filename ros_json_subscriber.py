from __future__ import print_function
import roslibpy
import json

client = roslibpy.Ros(host='localhost', port=9090)
client.run()


def callback(message):
    json_str = message['data']
    print('Heard talking: ' + json_str)

listener = roslibpy.Topic(client, '/matt', 'std_msgs/String')
listener.subscribe(callback)

try:
    while True:
        pass
except KeyboardInterrupt:
    client.terminate()
