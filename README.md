# CareBuddy

### Intro
사용자들이 공기 청정기의 최적의 사용법을 몰라도 자동으로 공간의 공기를 케어하는 이동용 홈 케어기

### 팀원
- 김민규
- 장하늘
- 임혜연

### 개발 환경
- VMware 17 player
- Ubuntu 20.04
- Raspbian Buster
- RoS Noetic(Ubuntu), Melodic(Raspberry pi)
- MySQL
- Python 2.7.16(Raspberry pi)
- Python 3.8.10(Ubuntu)
- nodejs 20.11.1

### 하드웨어
- Raspberry pi 4B 4GB/8GB
- YDLiDAR X4
- RPLiDAR A1M8
- DHT-11

### Carebuddy_ros github
https://github.com/hyeyeonIm/Carebuddy_ros.git

### 자동 매핑 영상
<iframe width="928" height="522" src="https://www.youtube.com/embed/p6YEz9cayZI" title="Automapping (Active Slam)" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### 매핑 영상
[![CareBuddy Mapping](https://img.youtube.com/vi/ebhQJ1M6wfo/0.jpg)](https://www.youtube.com/watch?v=ebhQJ1M6wfo)

### 실행 순서
// 우분투

    roscore

// 라즈베리파이

    roslaunch rplidar_ros rplidar_a1.launch 

// 우분투

    roslaunch hector_slam_launch tutorial.launch

// 우분투

    roslaunch rosbridge_server rosbridge_websocket.launch

    cd ~/catkin_ws/webserver/
    python3 server.py

    http://localhost:8080 들어가기

// 우분투 : map 저장

    rosrun map_server map_saver -f ~/catkin_ws/saveMap/

