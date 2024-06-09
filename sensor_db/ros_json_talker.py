# Description: ROS에 json 데이터를 전송하는 토픽 퍼블리셔
import time
import roslibpy
import json

# 테스트용 json 데이터
json_data = {"temperature": 26,
             "humidity": 40,
             "DUST_PM1_0_ATM": 2,
             "DUST_PM2_5_ATM": 4,
             "DUST_PM10_0_ATM": 6,
             "co2": 450
             }
json_str = json.dumps(json_data)

client = roslibpy.Ros(host='192.168.137.217', port=9090) # host 주소는 rosbridge가 실행되는 컴퓨터의 IP로 변경해야 함
client.run()

talker = roslibpy.Topic(client, '/matt', 'std_msgs/String') # /matt는 토픽 이름으로 변경 가능

while client.is_connected:
    talker.publish({'data':json_str})
    print('Sending message...')
    time.sleep(5) # 5초마다 메시지 전송

talker.unadvertise()

client.terminate()
