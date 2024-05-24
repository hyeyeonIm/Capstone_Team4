import http.server
import socketserver
import subprocess
from urllib.parse import urlparse, unquote, parse_qs
import os
import signal
import json
import time

PORT = 8080
BASE_DIR = "/home/haley/catkin_ws/roomInfo/"
WEB_DIR = "/home/haley/catkin_ws/webserver/"
running_processes = []
# Dictionary to store the latest data received via POST
latest_data = {}

class Handler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        parsed_path = urlparse(self.path)
        path_components = parsed_path.path.split('/')
        if parsed_path.path == "/start_hector":
            self.start_hector()
        elif parsed_path.path.startswith("/stop_and_save/"):
            map_name = parsed_path.path[len("/stop_and_save/"):]
            self.stop_and_save(map_name)
        elif parsed_path.path.startswith("/load_map/"):
            map_name = parsed_path.path[len("/load_map/"):]
            self.load_map(map_name)
        elif path_components[1] == "Segmentation":
            map_name = unquote(path_components[2])
            self.segmentation(map_name)
        elif path_components[1] == "room_coordinates.txt":
            self.serve_room_coordinates()
        elif path_components[1] == "save_coordinates.txt":
            self.serve_saved_coordinates()
        elif path_components[1] == "schedule.txt":
            self.serve_schedule()
        elif len(path_components) > 1 and path_components[1] == "Move":
            roomId = unquote(path_components[2])
            self.move_to_room(roomId)
        elif self.path == '/latest-data':
            self.serve_latest_data()
        elif self.path == '/dust_data.html':
            self.serve_file('dust_data.html')
        else:
            super().do_GET()

    def do_POST(self):
        parsed_path = urlparse(self.path)
        path_components = parsed_path.path.split('/')
        if self.path == '/data':
            self.receive_data()
        elif len(path_components) > 1:
            action = path_components[1]
            if action == "save_coordinates":
                content_length = int(self.headers['Content-Length'])
                post_data = self.rfile.read(content_length)
                post_params = parse_qs(post_data.decode('utf-8'))
                rooms = post_params.get('rooms', [None])[0]
                if rooms:
                    self.save_coordinates(rooms.split(','))
                else:
                    self.send_response(400)
                    self.end_headers()
                    self.wfile.write(bytes("No rooms provided", "utf8"))
            elif action == "execute_command":
                content_length = int(self.headers['Content-Length'])
                post_data = self.rfile.read(content_length)
                post_params = parse_qs(post_data.decode('utf-8'))
                command = post_params.get('command', [None])[0]
                if command:
                    self.execute_command(command)
                else:
                    self.send_response(400)
                    self.end_headers()
                    self.wfile.write(bytes("No command provided", "utf8"))
            else:
                self.send_response(404)
                self.end_headers()
        # elif path.startswith("/roomInfo/"):
        #     file_path = os.path.join(BASE_DIR, path.lstrip('/'))
        #     if os.path.isfile(file_path):
        #         self.send_response(200)
        #         self.send_header('Content-type', 'text/plain')
        #         self.end_headers()
        #         with open(file_path, 'rb') as file:
        #             self.wfile.write(file.read())
        #     else:
        #         self.send_response(404)
        #         self.send_header('Content-type', 'text/plain')
        #         self.end_headers()
        #         self.wfile.write(b'File not found')

    def receive_data(self):
        """Handle POST request to receive data and update the latest_data."""
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length)
        try:
            global latest_data
            latest_data = json.loads(post_data)
            print('Received JSON data:', latest_data)
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'Data received')
        except json.JSONDecodeError:
            self.send_response(400)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'Invalid JSON')

    def serve_latest_data(self):
        """Serve the latest data in JSON format."""
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(bytes(json.dumps(latest_data), 'utf8'))

    def serve_file(self, filename):
        """Serve a static file from the webserver directory."""
        try:
            file_path = os.path.join(WEB_DIR, filename)
            with open(file_path, 'rb') as file:
                self.send_response(200)
                self.send_header('Content-Type', 'text/html')
                self.end_headers()
                self.wfile.write(file.read())
        except Exception as e:
            self.send_response(500)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(bytes(f"Server Error: {e}", 'utf8'))

    def start_hector(self):
        try:
            # Start SLAM launch
            slam_process = subprocess.Popen(['roslaunch', 'omo_r1mini_slam', 'omo_r1mini_slam.launch'])
            time.sleep(5)  # Wait for 5 seconds before starting the next process

            # Start Explore launch
            explore_process = subprocess.Popen(['roslaunch', 'explore_lite', 'explore.launch'])
            time.sleep(5)  # Wait for 5 seconds before starting the next process

            # Start Navigation launch
            navigation_process = subprocess.Popen(['roslaunch', 'omo_r1mini_navigation', 'omo_r1mini_navigation.launch'])

            response = "SLAM launch started successfully."
        except FileNotFoundError as e:
            response = f"Error starting SLAM launch: {e}"
        self.send_response(200)
        self.send_header('Content-type', 'text/plain')
        self.end_headers()
        self.wfile.write(bytes(response, "utf8"))

    def stop_and_save(self, map_name):
        map_name = unquote(map_name)  # URL 인코딩된 맵 이름을 디코드
        try:
            save_cmd = ['rosrun', 'map_server', 'map_saver', '-f', f'/home/haley/catkin_ws/saveMap/{map_name}']
            subprocess.run(save_cmd, check=True)  # 명령어 완료까지 대기
            response = f"Map saved as {map_name}."

            # slam 종료
            subprocess.call(['rosnode', 'kill', '/omo_r1mini_slam_gmapping'])
            # 지도가 저장되었으므로 slam 관련 모든 명령어 종료
            # subprocess.call(['pkill', '-f', 'omo_r1mini_slam_launch'])
            response += "\nslam is over"

        except FileNotFoundError as e:
            response = f"Error saving map or stopping slam: {e}"
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
            # 지도 불러오는 ros 명령어
            subprocess.Popen(['rosrun', 'map_server', 'map_server', f'{map_path}.yaml'])
            response = "Map loaded successfully."
        except Exception as e:
            response = f"Error loading map: {e}"
        self.send_response(200)
        self.send_header('Content-type', 'text/plain')
        self.end_headers()
        self.wfile.write(bytes(response, "utf8"))

    def segmentation(self, map_name):
        global running_processes
        # 기존 프로세스를 종료합니다.
        for proc in running_processes:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except Exception as e:
                print(f"Error killing process: {e}")

        running_processes = []

        map_path = f"/home/haley/catkin_ws/saveMap/{map_name}"
        try:
            # subprocess.Popen(['roslaunch', 'ipa_room_segmentation', 'room_segmentation_action_server.launch'])
            # subprocess.Popen(['rosrun', 'ipa_room_segmentation', 'room_segmentation_client.py', f'{map_path}.pgm', '0.05'])
            proc1 = subprocess.Popen(['rosrun', 'omo_control', 'my_markers.py'], preexec_fn=os.setsid)
            proc2 = subprocess.Popen(['roslaunch', 'omo_r1mini_navigation', 'omo_r1mini_navigation.launch',
                                      'map_file:=' + f'{map_path}.yaml'], preexec_fn=os.setsid)

            running_processes.extend([proc1, proc2])

            response = "All segmentation and visualization commands executed."
        except Exception as e:
            response = f"Error during command execution: {e}"
        self.send_response(200)
        self.send_header('Content-type', 'text/plain')
        self.end_headers()
        self.wfile.write(bytes(response, "utf8"))

    def serve_room_coordinates(self):
        try:
            with open(os.path.join(BASE_DIR, 'room_coordinates.txt'), 'r') as file:
                content = file.read()

            self.send_response(200)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(bytes(content, "utf8"))
        except Exception as e:
            self.send_response(500)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(bytes(f"Error reading room coordinates: {e}", "utf8"))

    def serve_saved_coordinates(self):
        try:
            with open(os.path.join(BASE_DIR, 'save_coordinates.txt'), 'r') as file:
                content = file.read()

            self.send_response(200)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(bytes(content, "utf8"))
        except Exception as e:
            self.send_response(500)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(bytes(f"Error reading saved coordinates: {e}", "utf8"))

    def save_coordinates(self, selected_rooms):
        global running_processes
        try:
            with open(os.path.join(BASE_DIR, 'room_coordinates.txt'), 'r') as infile:
                lines = infile.readlines()

            selected_data = []
            for line in lines:
                if line.strip() == "" or line.startswith("RoomID"):
                    continue
                room_data = line.strip().split(',')
                if room_data[0] in selected_rooms:
                    selected_data.append(line.strip())

            # 디버깅 메시지 추가
            print(f"Selected Data to save: {selected_data}")

            # save_coordinates.txt 파일에 데이터를 저장합니다.
            save_coordinates_path = os.path.join(BASE_DIR, 'save_coordinates.txt')
            with open(save_coordinates_path, 'w') as outfile:
                outfile.write("RoomID, CenterX (meters), CenterY (meters)\n")
                for data in selected_data:
                    outfile.write(data + "\n")

            # schedule.txt 파일에 데이터를 저장합니다.
            schedule_path = os.path.join(BASE_DIR, 'schedule.txt')
            with open(schedule_path, 'w') as schedule_file:
                schedule_file.write("RoomID, CenterX (meters), CenterY (meters)\n")
                for data in selected_data:
                    schedule_file.write(data + "\n")

            # 디버깅 메시지 추가
            print(f"schedule.txt file created at: {schedule_path}")

            # 기존 my_markers.py 프로세스를 종료합니다.
            for proc in running_processes:
                try:
                    if 'my_markers.py' in proc.args:
                        print(f"Attempting to kill process {proc.pid} for my_markers.py")
                        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                        running_processes.remove(proc)
                        print("my_markers.py process killed.")
                except Exception as e:
                    print(f"Error killing process: {e}")

            # new_markers.py를 실행합니다.
            try:
                print("Starting new_markers.py process")
                new_proc = subprocess.Popen(['rosrun', 'omo_control', 'new_markers.py'], preexec_fn=os.setsid)
                running_processes.append(new_proc)
                print("new_markers.py process started.")
            except Exception as e:
                print(f"Error starting new_markers.py: {e}")

            self.send_response(200)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(bytes("Coordinates saved successfully", "utf8"))
        except Exception as e:
            print(f"Exception in save_coordinates: {e}")
            self.send_response(500)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(bytes(f"Error saving coordinates: {e}", "utf8"))

    def serve_schedule(self):
        try:
            with open(os.path.join(BASE_DIR, 'schedule.txt'), 'r') as file:
                content = file.read()

            self.send_response(200)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(bytes(content, "utf8"))
        except Exception as e:
            self.send_response(500)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(bytes(f"Error reading schedule: {e}", "utf8"))

    def execute_command(self):
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length)
        try:
            data = json.loads(post_data)
            command = data.get('command')
            if not command:
                raise ValueError("No command provided")

            print(f"Executing command: {command}")
            result = subprocess.run(command, check=True, capture_output=True, text=True)
            self.send_response(200)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(bytes(result.stdout, "utf8"))
        except Exception as e:
            print(f"Error executing command: {e}")
            self.send_response(400)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(bytes(f"Error executing command: {e}", "utf8"))

    def move_to_room(self, roomId):
        try:
            print(f"Moving to room {roomId}")
            subprocess.Popen(['rosrun', 'omo_control', 'move_to_room.py', roomId], preexec_fn=os.setsid)
            self.send_response(200)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(bytes(f"Moving to room {roomId}", "utf8"))
        except Exception as e:
            print(f"Error moving to room: {e}")
            self.send_response(500)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(bytes(f"Error moving to room: {e}", "utf8"))

    # def serve_room_coordinates(self):
    #     try:
    #         with open(os.path.join(BASE_DIR, 'room_coordinates.txt'), 'r') as file:
    #             content = file.read()

    #         self.send_response(200)
    #         self.send_header('Content-type', 'text/plain')
    #         self.end_headers()
    #         self.wfile.write(bytes(content, "utf8"))
    #     except Exception as e:
    #         self.send_response(500)
    #         self.send_header('Content-type', 'text/plain')
    #         self.end_headers()
    #         self.wfile.write(bytes(f"Error reading room coordinates: {e}", "utf8"))

    # def serve_saved_coordinates(self):
    #     try:
    #         with open(os.path.join(BASE_DIR, 'save_coordinates.txt'), 'r') as file:
    #             content = file.read()

    #         self.send_response(200)
    #         self.send_header('Content-type', 'text/plain')
    #         self.end_headers()
    #         self.wfile.write(bytes(content, "utf8"))
    #     except Exception as e:
    #         self.send_response(500)
    #         self.send_header('Content-type', 'text/plain')
    #         self.end_headers()
    #         self.wfile.write(bytes(f"Error reading saved coordinates: {e}", "utf8"))

    # def save_coordinates(self, selected_rooms):
    #     try:
    #         with open(os.path.join(BASE_DIR, 'room_coordinates.txt'), 'r') as infile:
    #             lines = infile.readlines()

    #         selected_data = []
    #         for line in lines:
    #             if line.strip() == "" or line.startswith("RoomID"):
    #                 continue
    #             room_data = line.strip().split(',')
    #             if room_data[0] in selected_rooms:
    #                 selected_data.append(line.strip())

    #         with open(os.path.join(BASE_DIR, 'save_coordinates.txt'), 'w') as outfile:
    #             outfile.write("RoomID, CenterX (meters), CenterY (meters)\n")
    #             for data in selected_data:
    #                 outfile.write(data + "\n")

    #         self.send_response(200)
    #         self.send_header('Content-type', 'text/plain')
    #         self.end_headers()
    #         self.wfile.write(bytes("Coordinates saved successfully", "utf8"))
    #     except Exception as e:
    #         self.send_response(500)
    #         self.send_header('Content-type', 'text/plain')
    #         self.end_headers()
    #         self.wfile.write(bytes(f"Error saving coordinates: {e}", "utf8"))

    # navigation
    # def navigation(self, map_name):
    #     map_path = f"/home/haley/catkin_ws/saveMap/{map_name}"
    #     try:
    #         # 모든 필요한 ROS 명령을 순차적으로 실행
    #         # "/home/haley/catkin_ws/roomInfo/save_coordinates.txt" 로부터 new_markers 실행
    #         # subprocess.Popen(['roslaunch', 'omo_r1mini_navigation', 'omo_r1mini_navigation.launch', 'map_file:=' + f'{map_path}.yaml'])
    #         # Add carebuddy icon on roslisb.js
    #         response = "Navigation is open"
    #     except Exception as e:
    #         response = f"Error during command execution: {e}"
    #     self.send_response(200)
    #     self.send_header('Content-type', 'text/plain')
    #     self.end_headers()
    #     self.wfile.write(bytes(response, "utf8"))

    # Move_Schedule
    # Basic : go to 1st location
    # Case 1 : make schedule.txt copy by save_coordinates.txt
    # Case 2 : change current location button
    #           -> move_to_room right after
    # Case 3: change next location button
    #           -> Add Room ID on schedule.txt
    # Robot always move using by "move_to_room.py {ROOM ID}"
    # And they stay 10s at location
    # rosrun omo_control move_to_room.py 3

with socketserver.TCPServer(("", PORT), Handler) as httpd:
    print(f"Serving at port: {PORT}")
    httpd.serve_forever()