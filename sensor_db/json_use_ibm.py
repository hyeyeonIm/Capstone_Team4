import json
import requests

# API 키가 포함된 JSON 파일 읽기
with open('apikey.json', 'r') as file:
    api_data = json.load(file)

# JSON에서 API 키 값 추출
API_KEY = api_data['apikey']

# IBM Cloud에서 액세스 토큰 요청
token_response = requests.post('https://iam.cloud.ibm.com/identity/token', data={"apikey": API_KEY, "grant_type": 'urn:ibm:params:oauth:grant-type:apikey'})
mltoken = token_response.json()["access_token"]

header = {'Content-Type': 'application/json', 'Authorization': 'Bearer ' + mltoken}

# room_air_data.json 파일 읽기
with open('test.json', 'r') as file:
    room_air_data = json.load(file)

# payload_scoring에 room_air_data.json 파일의 값을 설정
payload_scoring = {
    "input_data": room_air_data["input_data"]
}

# API 요청 보내기
response_scoring = requests.post('https://private.jp-tok.ml.cloud.ibm.com/ml/v4/deployments/air_ubuntu/predictions?version=2021-05-01', json=payload_scoring, headers=header)
print("Scoring response")
print(response_scoring.json())
