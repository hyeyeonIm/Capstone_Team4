# server.py
import http.server
import socketserver
import subprocess
from urllib.parse import urlparse, parse_qs

PORT = 8080

class Handler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        self.handle_http()

    def handle_http(self):
        parsed_path = urlparse(self.path)
        path = parsed_path.path

        if path == "/start_mapping":
            # 'hector' 명령 실행 예시. 실제 명령어로 교체 필요
            subprocess.Popen(["echo", "hector"])
            self.respond("매핑이 시작되었습니다.")
        elif path.startswith("/stop_and_save/"):
            map_name = path.split('/')[-1]
            # 지도 저장 명령 실행 예시. 실제 명령어로 교체 필요
            subprocess.Popen(["echo", f"rosrun map_server map_saver -f ~/catkin_ws/saveMap/{map_name}"])
            self.respond("맵이 저장되었습니다.")
        else:
            super().do_GET()

    def respond(self, message):
        self.send_response(200)
        self.send_header('Content-type', 'text/plain')
        self.end_headers()
        self.wfile.write(bytes(message, "utf8"))

with socketserver.TCPServer(("", PORT), Handler) as httpd:
    print(f"Server started at http://localhost:{PORT}")
    httpd.serve_forever()

# from flask import Flask, request
# import subprocess

# app = Flask(__name__)

# @app.route('/start_mapping', methods=['GET'])
# def start_mapping():
#     # ROS hector 매핑 시작 명령어 실행
#     subprocess.Popen(['roslaunch', 'hector_mapping', 'mapping_default.launch'])
#     return "매핑이 시작되었습니다."

# @app.route('/stop_and_save/<map_name>', methods=['GET'])
# def stop_and_save(map_name):
#     # ROS 맵 저장 명령어 실행
#     subprocess.Popen(['rosrun', 'map_server', 'map_saver', '-f', f'/home/username/catkin_ws/maps/{map_name}'])
#     return "맵이 저장되었습니다."

# if __name__ == '__main__':
#     app.run(host='0.0.0.0', port=5000, debug=True)
