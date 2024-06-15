import json
import requests

# JSON 파일 읽기
with open('apikey.json', 'r') as file:
    data = json.load(file)

# JSON에서 API 키 값 추출
API_KEY = data['apikey']

# IBM Cloud에서 액세스 토큰 요청
token_response = requests.post('https://iam.cloud.ibm.com/identity/token', data={"apikey": API_KEY, "grant_type": 'urn:ibm:params:oauth:grant-type:apikey'})
mltoken = token_response.json()["access_token"]

header = {'Content-Type': 'application/json', 'Authorization': 'Bearer ' + mltoken}

# NOTE: manually define and pass the array(s) of values to be scored in the next line
payload_scoring = {
    "input_data": [
        {
            "id" : "oberservation",
            "values": [["array_of_values_to_be_scored"], ["another_array_of_values_to_be_scored"]]
        }
    ]
}

# API 요청 보내기
response_scoring = requests.post('https://private.jp-tok.ml.cloud.ibm.com/ml/v4/deployments/air_ubuntu/predictions?version=2021-05-01', json=payload_scoring, headers=header)
print("Scoring response")
print(response_scoring.json())
