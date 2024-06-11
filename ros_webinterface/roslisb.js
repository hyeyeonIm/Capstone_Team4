$(document).ready(function() {
    // ROS 연결 초기화
    var ros = new ROSLIB.Ros({
        url: 'ws://localhost:9090'
    });

    // ROS 연결 상태 업데이트
    var statusSpan = document.getElementById("status");
    ros.on('connection', function() {
        console.log('Connected to websocket server.');
        statusSpan.innerHTML = "Connected";

        var roomMarkerListener = new ROSLIB.Topic({
            ros: ros,
            name: '/room_markers',
            messageType: 'visualization_msgs/Marker'
        });

        console.log('Attempting to subscribe to /room_markers');

        roomMarkerListener.subscribe(function(message) {
            console.log('Received message on ' + roomMarkerListener.name + ': ', message);
            displayMarker(message);
        });
    });

    ros.on('error', function(error) {
        console.log('Error connecting to websocket server:', error);
        statusSpan.innerHTML = "Error";
    });

    ros.on('close', function() {
        console.log('Connection to websocket server closed.');
        statusSpan.innerHTML = "Closed";
    });

    // 페이지 로드 시 매핑 상태 확인
    if (localStorage.getItem("mappingFinished") === "true") {
        $(".Start_Map, .Stop_Map").hide();
        $(".Mapping_Again, .Segmenatation").show();
    } else {
        $(".Start_Map, .Stop_Map").show();
        $(".Mapping_Again, .Segmenatation").hide();
    }

    // 매핑 시작 버튼 클릭 이벤트
    $(".Start_Map").click(async function() {
        try {
            const data = await $.get("http://localhost:8080/start_hector");
            alert(`Response: ${data}\nStatus: success`);
        } catch (error) {
            console.error("An error occurred:", error);
            alert(`Error starting mapping: ${error.statusText}`);
        }
    });

    // 매핑 중지 및 저장 버튼 클릭 이벤트
    $('.Stop_Map').click(async function() {
        var mapName = prompt("지도의 이름을 입력해주세요:", "defaultMapName");
        if (mapName) {
            try {
                const saveResponse = await $.get(`http://localhost:8080/stop_and_save/${mapName}`);
                alert(saveResponse); // 저장 완료 알림

                const loadResponse = await $.get(`http://localhost:8080/load_map/${mapName}`);
                console.log(loadResponse); // 콘솔에 로딩 결과를 출력
                alert(loadResponse); // 로딩 완료 알림
            } catch (error) {
                console.error("An error occurred:", error);
                alert(`Error: ${error.statusText}`);
            }
        }
        // 매핑 상태 저장
        localStorage.setItem("mappingFinished", "true");
        $(".Start_Map").hide();
        $(".Stop_Map").hide();
        $(".Mapping_Again").show();
        $(".Segmentation").show();
    });

    // 다시 매핑하기 버튼 클릭 이벤트
    $(".Mapping_Again").click(function() {
        // 매핑 상태 초기화 및 저장
        localStorage.setItem("mappingFinished", "true");
        $(".Start_Map").show();
        $(".Stop_Map").show();
        $(this).hide(); // "다시 매핑하기" 버튼 숨기기

        // 서버로 요청 보내기
        $.get('http://localhost:8080/mapping_again', function(response) {
            alert('Mapping again process completed successfully.');
        }).fail(function() {
            alert('Error in mapping again process.');
        });
    });



    fetchSavedRooms();

    $('.Segmentation').off('click').on('click', async function() {
        var mapName = prompt("Enter the map name:", "defaultMapName");
        if (mapName) {
            try {
                console.log(`Starting segmentation for map: ${mapName}`);
                const response = await $.get(`http://localhost:8080/Segmentation/${mapName}`);
                console.log(response);
                alert(response);

                // Segmentation 완료 후 room_coordinates.txt 읽기
                console.log("Reading room coordinates");
                const roomResponse = await $.get('http://localhost:8080/room_coordinates.txt');
                console.log("Room coordinates received:", roomResponse);
                updateRoomList(roomResponse);
            } catch (error) {
                console.error("An error occurred:", error);
                alert(`Error: ${error.statusText}`);
            }
        }
    });

    // Save room info button click event
    $('.save_room_info').off('click').on('click', function() {
        var selectedRooms = [];
        document.querySelectorAll('input[name="rooms"]:checked').forEach((checkbox) => {
            selectedRooms.push(checkbox.value);
        });

        if (selectedRooms.length > 0) {
            $.post('http://localhost:8080/save_coordinates', { rooms: selectedRooms.join(',') })
                .done(function(response) {
                    alert("Coordinates saved successfully!");
                    console.log("Coordinates saved:", response);
                    location.reload(); // 화면 새로고침
                })
                .fail(function(error) {
                    console.error("Error saving coordinates:", error);
                    alert("Error saving coordinates.");
                });
        } else {
            alert('No rooms selected');
        }
    });

    // 지도 시각화 관련 코드
    var viewer = new ROS2D.Viewer({
        divID: 'map',
        width: 800,
        height: 600
    });

    var gridClient = new ROS2D.OccupancyGridClient({
        ros: ros,
        rootObject: viewer.scene,
        topic: '/map',
        continuous: true
    });

    gridClient.on('change', function() {
        viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
        viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
        console.log('Map loaded successfully.');
    });

    function displayMarker(marker) {
        const markerRoomID = marker.text.replace('Room ', '').trim();
        console.log("Displaying marker for room: ", markerRoomID);
        console.log("Current selected rooms: ", window.selectedRooms);


        var shape = new createjs.Shape();
        shape.graphics.beginFill("blue").drawCircle(0, 0, 0.1);
        shape.x = marker.pose.position.x;
        shape.y = -marker.pose.position.y;

        viewer.scene.addChild(shape);

        var text = new createjs.Text(marker.text, "0.5px Arial", "#005F73");
        text.x = marker.pose.position.x;
        text.y = -marker.pose.position.y + 0.1;

        viewer.scene.addChild(text);

        // if (window.selectedRooms && window.selectedRooms.includes(markerRoomID)) {
        //     var shape = new createjs.Shape();
        //     shape.graphics.beginFill("blue").drawCircle(0, 0, 0.1);
        //     shape.x = marker.pose.position.x;
        //     shape.y = -marker.pose.position.y;

        //     viewer.scene.addChild(shape);

        //     var text = new createjs.Text(marker.text, "0.5px Arial", "#005F73");
        //     text.x = marker.pose.position.x;
        //     text.y = -marker.pose.position.y + 0.1;

        //     viewer.scene.addChild(text);
        // } else {
        //     console.log(`Marker for Room ${markerRoomID} not displayed.`);
        // }
    }

    // 로봇 마커 생성 및 초기 설정
    var robotMarker = new createjs.Shape();
    robotMarker.graphics.beginFill("green").drawCircle(0, 0, 0.3); // 로봇을 나타내는 파란색 원
    robotMarker.regX = 0;
    robotMarker.regY = 0;

    // 로봇 마커 생성 및 초기 설정
    // var robotMarker = new createjs.Bitmap("./icons/buddy_icon.png");
    // robotMarker.regX = 0;
    // robotMarker.regY = 0;
    // robotMarker.scaleX = 0.002;
    // robotMarker.scaleY = 0.002;

    // 로봇 마커를 뷰어에 추가
    gridClient.rootObject.addChild(robotMarker);

    // odom 토픽 구독
    var odomListener = new ROSLIB.Topic({
        ros: ros,
        name: '/odom',
        messageType: 'nav_msgs/Odometry'
    });


    odomListener.subscribe(function(message) {
        console.log('Received odometry data:', message);
    
        var pose = message.pose.pose;
    
        // 로봇의 위치 및 방향 설정
        var x = pose.position.x;
        var y = pose.position.y;
    
        if (gridClient.currentGrid && gridClient.currentGrid.pose && gridClient.currentGrid.info) {
            // 좌표 변환을 통한 로봇 마커 위치 조정
            var transformedX = (x - gridClient.currentGrid.pose.position.x) / gridClient.currentGrid.info.resolution;
            var transformedY = -(y - gridClient.currentGrid.pose.position.y) / gridClient.currentGrid.info.resolution;
    
            // 로봇 마커 위치 업데이트
            robotMarker.x = transformedX * viewer.scene.scaleX + viewer.scene.x;
            robotMarker.y = transformedY * viewer.scene.scaleY + viewer.scene.y;
    
            var q0 = pose.orientation.w;
            var q1 = pose.orientation.x;
            var q2 = pose.orientation.y;
            var q3 = pose.orientation.z;
            var theta = Math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));
            robotMarker.rotation = theta * (180 / Math.PI);
    
            console.log(`Updated robot position: x=${robotMarker.x}, y=${robotMarker.y}, rotation=${robotMarker.rotation}`);
        } else {
            console.warn('Grid data not available yet. Using raw odometry data.');
            // 현재 맵 데이터가 없으므로 raw odometry data를 사용하여 로봇 마커 위치 업데이트
            robotMarker.x = x;
            robotMarker.y = -y;
    
            var q0 = pose.orientation.w;
            var q1 = pose.orientation.x;
            var q2 = pose.orientation.y;
            var q3 = pose.orientation.z;
            var theta = Math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));
            robotMarker.rotation = theta * (180 / Math.PI);
        }
    
        // 로봇 마커를 뷰어에 추가
        if (!viewer.scene.contains(robotMarker)) {
            viewer.scene.addChild(robotMarker);
        }
    });


});



    // // Example of subscribing to a topic
    // var moveBaseFeedbackListener = new ROSLIB.Topic({
    //     ros: ros,
    //     name: '/move_base/feedback',
    //     messageType: 'move_base_msgs/MoveBaseActionFeedback'
    // });

    // moveBaseFeedbackListener.subscribe(function(message) {
    //     console.log('Received move_base feedback: ', message);
    //     // Handle or visualize the feedback
    // });