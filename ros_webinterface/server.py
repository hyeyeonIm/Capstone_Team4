import http.server
import socketserver
import subprocess
from urllib.parse import urlparse, unquote
import subprocess

PORT = 8080

class Handler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        parsed_path = urlparse(self.path)
        path = parsed_path.path

        if path == "/start_hector":
            self.start_hector()
        elif path.startswith("/stop_and_save/"):
            map_name = path[len("/stop_and_save/"):]
            self.stop_and_save(map_name)
        elif path.startswith("/load_map/"):
            map_name = path[len("/load_map/"):]
            self.load_map(map_name)
        else:
            super().do_GET()

    def start_hector(self):
        try:
            subprocess.Popen(['roslaunch', 'hector_slam_launch', 'tutorial.launch'])
            response = "Hector SLAM launch started successfully."
        except FileNotFoundError as e:
            response = f"Error starting Hector SLAM launch: {e}"
        self.send_response(200)
        self.send_header('Content-type', 'text/plain')
        self.end_headers()
        self.wfile.write(bytes(response, "utf8"))

    def stop_and_save(self, map_name):
        map_name = unquote(map_name)  # URL 인코딩된 맵 이름을 디코드
        try:
            # 맵 저장 - subprocess.run 사용
            save_cmd = ['rosrun', 'map_server', 'map_saver', '-f', f'/home/haley/catkin_ws/saveMap/{map_name}']
            subprocess.run(save_cmd, check=True)  # 이 명령이 완료될 때까지 기다립니다.
            response = f"Map saved as {map_name}."

            # hector_slam 종료
            subprocess.call(['pkill', '-f', 'roslaunch hector_slam_launch tutorial.launch'])
            subprocess.call(['kill' '-f' 'hector_slam_launch'])

        except FileNotFoundError as e:
            response = f"Error saving map or stopping hector_slam: {e}"
        except subprocess.CalledProcessError as e:
            response = f"Error during command execution: {e}"
            
        self.send_response(200)
        self.send_header('Content-type', 'text/plain')
        self.end_headers()
        self.wfile.write(bytes(response, "utf8"))


    
    def load_map(self, map_name):
        map_path = f"/home/haley/catkin_ws/saveMap/{map_name}"
        try:
            print(f"Loading map: {map_path}")
            subprocess.Popen(['rosrun', 'map_server', 'map_server', f'{map_path}.yaml'])
            response = "Map loaded successfully."
        except Exception as e:
            response = f"Error loading map: {e}"
        self.send_response(200)
        self.send_header('Content-type', 'text/plain')
        self.end_headers()
        self.wfile.write(bytes(response, "utf8"))



with socketserver.TCPServer(("", PORT), Handler) as httpd:
    print(f"Serving at port: {PORT}")
    httpd.serve_forever()
